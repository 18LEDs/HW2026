/*
  Children of the Pier - Capacitive Touch Pad Diagnostic
  ----------------------------------------------------------------------
  Prints live touchRead() values for the six touch-capable pins that
  are safe to use (the other four touch pins - GPIO 0, 2, 12, 15 - are
  strapping pins that can stop the board booting). Use this to validate
  pads as you build them, before writing any puzzle logic.

  Wire a pad to any of these pins and watch its column. No resistor,
  no ground connection to the pad, just the one wire.

  ----------------------------------------------------------------------
  WHY THE NUMBERS AREN'T 40-100
  ----------------------------------------------------------------------
  Most touch tutorials (and the first version of this script) describe
  the OLD ESP32 touch driver, where an untouched pad read roughly
  40-100. This project builds on Arduino-ESP32 core 3.3.x, which sits on
  ESP-IDF 5.5 - and from IDF 5.5 the core switches to the NEW touch
  driver (esp32-hal-touch-ng.c). On a classic ESP32 that driver:

    - counts charge/discharge cycles over a 5 ms window, so the values
      are on a completely different scale than the old 40-100
    - returns a SMOOTHED (filtered) reading, not a raw one
    - still DROPS when touched, same direction as before

  The absolute number therefore tells you nothing on its own. This
  sketch measures each pad's untouched baseline at startup and shows
  the percentage DROP from it, which is what actually matters.

  ----------------------------------------------------------------------
  HOW TO USE
  ----------------------------------------------------------------------
    1. Upload, open Serial Monitor at 115200 baud.
    2. KEEP HANDS OFF the pads and wires for the first couple of
       seconds while it measures baselines.
    3. Touch each pad and watch its drop% column. A '*' marks readings
       past TOUCH_DROP_PERCENT.
    4. Every 40 rows a "max" line shows the biggest drop each pad has
       seen. Touch every pad firmly a few times, then read that line -
       a puzzle threshold of roughly HALF a pad's max drop is a good
       starting point.

  Serial Monitor commands (type the letter, press Enter):
    b  re-measure baselines (hands off!)
    r  reset the max-drop tracking

  ----------------------------------------------------------------------
  WHAT TO LOOK FOR
  ----------------------------------------------------------------------
    - Untouched, drop% should hover near 0 - within a percent or two.
    - Touched, drop% should jump clearly positive.
    - Reads 0 at baseline: the channel isn't reporting, or the pad is
      shorted to ground.
    - Jumps around untouched: bad connection, or the wire runs alongside
      another pad's wire (they couple - keep pad wires apart).
    - Barely moves when touched: pad too small, covering too thick, or
      the wire to it is long (wire adds capacitance and dilutes the
      change your finger makes).
    - Touches feel weaker when the ESP32 runs from a laptop on battery
      than from a wall supply. That is real physics, not a fault -
      calibrate on the same power source the prop will use.

  An unconnected pin still reads a value and still responds a little to
  a hand near the board. Only trust columns that have a pad attached.
*/

#include "esp_arduino_version.h"

const uint8_t TOUCH_PINS[] = {4, 13, 14, 27, 32, 33};
const int PIN_COUNT = sizeof(TOUCH_PINS) / sizeof(TOUCH_PINS[0]);

// Drop from baseline, in percent, that counts as a touch for the '*'
// marker. A starting guess only - the "max" line tells you what your
// actual pads do, and that is what the puzzle threshold should be
// based on.
const float TOUCH_DROP_PERCENT = 5.0;

const int BASELINE_SAMPLES = 20;
const unsigned long PRINT_INTERVAL_MS = 250;
const int ROWS_BETWEEN_HEADERS = 40;

uint32_t baseline[PIN_COUNT];
float maxDropPercent[PIN_COUNT];
int rowsPrinted = 0;
unsigned long lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== Capacitive touch diagnostic ===");
  Serial.print("Arduino-ESP32 core ");
  Serial.print(ESP_ARDUINO_VERSION_STR);
  Serial.print(", ESP-IDF ");
  Serial.println(esp_get_idf_version());

  // The first touchRead() on each pin is what creates its touch
  // channel, and creating a channel stops and restarts the whole touch
  // controller. Doing that for all six pins inside the first pass of
  // loop() - as the original script did - meant the first rows were
  // read from a controller that had just been restarted five times and
  // had no measurements yet. Initialize everything up front instead,
  // then give the smoothing filter time to settle.
  Serial.println("Initializing touch channels...");
  for (int i = 0; i < PIN_COUNT; i++) {
    touchRead(TOUCH_PINS[i]);
  }

  // The driver returns SMOOTHED values, and the filter keeps drifting
  // for a few seconds after startup. One second was not enough: the
  // baseline got captured mid-drift, so every pad then showed a small
  // constant "drop" that had nothing to do with touch. Wait longer.
  // If a steady offset still shows up, type b to re-baseline.
  delay(4000);

  captureBaseline();
  resetMaxDrops();
  printHeader();
}

void loop() {
  handleSerialCommands();

  if (millis() - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = millis();

  for (int i = 0; i < PIN_COUNT; i++) {
    uint32_t value = touchRead(TOUCH_PINS[i]);
    float drop = dropPercent(i, value);

    if (drop > maxDropPercent[i]) {
      maxDropPercent[i] = drop;
    }

    // Each column is 16 characters wide so the header lines up.
    Serial.printf("  %6lu %5.1f%%%c", (unsigned long)value, drop,
                  drop >= TOUCH_DROP_PERCENT ? '*' : ' ');
  }
  Serial.println();

  if (++rowsPrinted >= ROWS_BETWEEN_HEADERS) {
    rowsPrinted = 0;
    printMaxDrops();
    printHeader();
  }
}

// Positive = value fell below baseline (a touch). Negative = rose.
float dropPercent(int i, uint32_t value) {
  if (baseline[i] == 0) {
    return 0.0;
  }
  return ((float)baseline[i] - (float)value) * 100.0 / (float)baseline[i];
}

void captureBaseline() {
  Serial.println("Measuring baselines - hands off the pads...");

  uint64_t sums[PIN_COUNT] = {0};
  for (int s = 0; s < BASELINE_SAMPLES; s++) {
    for (int i = 0; i < PIN_COUNT; i++) {
      sums[i] += touchRead(TOUCH_PINS[i]);
    }
    delay(20);
  }

  for (int i = 0; i < PIN_COUNT; i++) {
    baseline[i] = (uint32_t)(sums[i] / BASELINE_SAMPLES);
    Serial.printf("  GPIO%-2u baseline %lu", TOUCH_PINS[i], (unsigned long)baseline[i]);
    if (baseline[i] == 0) {
      Serial.print("   <-- reads 0: channel not reporting, or pad shorted to ground");
    }
    Serial.println();
  }
}

void resetMaxDrops() {
  for (int i = 0; i < PIN_COUNT; i++) {
    maxDropPercent[i] = 0.0;
  }
}

void printHeader() {
  Serial.println();
  for (int i = 0; i < PIN_COUNT; i++) {
    Serial.printf("  GPIO%-2u        ", TOUCH_PINS[i]);
  }
  Serial.println();
  for (int i = 0; i < PIN_COUNT; i++) {
    Serial.print("   value  drop  ");
  }
  Serial.println();
}

void printMaxDrops() {
  Serial.println();
  for (int i = 0; i < PIN_COUNT; i++) {
    Serial.printf("  max    %5.1f%%  ", maxDropPercent[i]);
  }
  Serial.println();
  Serial.println("  (biggest drop seen per pad - set puzzle thresholds to about half of this)");
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') {
      Serial.println();
      captureBaseline();
      resetMaxDrops();
      rowsPrinted = 0;
      printHeader();
    } else if (c == 'r' || c == 'R') {
      resetMaxDrops();
      Serial.println();
      Serial.println("Max-drop tracking reset.");
      rowsPrinted = 0;
      printHeader();
    }
  }
}
