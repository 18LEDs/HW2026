/*
  Children of the Pier - Capacitive Touch Sequence Puzzle - ESP32 Firmware
  ----------------------------------------------------------------------
  A set of touch pads, some of which must be touched in the correct
  order. Touch the right pads in the right order and TRIGGER_PIN fires.

  Out of the box: 6 pads, and a 3-touch secret sequence.

  ----------------------------------------------------------------------
  CHANGING THE PUZZLE
  ----------------------------------------------------------------------
  Everything lives in the CONFIG section below.

    Add/remove pads:   add or delete a row in PADS[]. Nothing else to
                       update - the pad count is worked out automatically.
    Change the order:  edit SEQUENCE[]. Each entry is a pad NUMBER, i.e.
                       its position in PADS[] (first row = 0).
    Change the length: add or remove entries in SEQUENCE[]. Any length
                       works, and a pad may appear more than once.
    Name the pads:     the names only appear in the Serial log, so name
                       them after whatever is painted/carved on each one.

  The sketch checks the config at startup and refuses to run with a
  broken one (non-touch pin, duplicate pin, sequence pointing at a pad
  that doesn't exist), printing exactly what's wrong.

  ----------------------------------------------------------------------
  HOW MATCHING WORKS
  ----------------------------------------------------------------------
  Touches are compared against the sequence as a sliding window, so a
  wrong touch doesn't always mean starting from nothing. With the
  sequence 3 -> 0 -> 5:
      3, 0, 5        solved
      3, 1           wrong, back to the start
      3, 3, 0, 5     solved - the second 3 simply restarts the attempt
  Guests who fumble don't have to deliberately "clear" anything first.
  Too long a pause between touches (MAX_GAP_MS) also resets.

  A touch is only counted once per press: the pad has to be released
  before it can register again. Holding a pad does nothing extra.

  ----------------------------------------------------------------------
  TUNING
  ----------------------------------------------------------------------
  Run touch_diagnostic.ino first with the real pads attached, and touch
  each one firmly several times. Its "max" line shows the biggest drop
  each pad produces; set that pad's threshold below to about HALF of it.
  Pads differ (size, covering, wire length), which is why each row gets
  its own threshold.

  This firmware is built for the classic ESP32 on Arduino-ESP32 core 3.3.x,
  where touch readings DROP when touched - see touch_diagnostic.ino for
  why the raw numbers look the way they do.

  ----------------------------------------------------------------------
  WIRING
  ----------------------------------------------------------------------
    Each pad -> its GPIO in PADS[] (one wire, no resistor, no ground)
    TRIGGER_PIN (GPIO 25) -> whatever the solve should drive (relay
                             module, LED, the next prop's input)
    Keep pad wires short and apart from each other.

  ----------------------------------------------------------------------
  SERIAL MONITOR (115200 baud, line ending: Newline)
  ----------------------------------------------------------------------
    0-9  simulate touching that pad number - test the logic and the
         trigger without touching anything
    v    toggle a live view of every pad's reading and drop
    b    re-measure baselines (hands off the pads!)
    h    show help
*/

#include "esp_arduino_version.h"

// ============================================================
// CONFIG
// ============================================================

// Starting touch threshold: how far (in percent) a pad's reading has to
// drop below its baseline to count as touched. Replace per pad once
// you've measured them with touch_diagnostic.ino.
const float DEFAULT_TOUCH_DROP_PERCENT = 5.0;

struct Pad {
  uint8_t pin;
  const char *name;
  float touchDropPercent;
};

// The pads. Row position is the pad NUMBER used in SEQUENCE[] below.
// Safe touch pins on the classic ESP32: 4, 13, 14, 27, 32, 33.
const Pad PADS[] = {
  //  pin   name       touch threshold (% drop)
  {    4,   "Pad 0",   41.0 },
  {   13,   "Pad 1",   41.0 },
  {   14,   "Pad 2",   41.0 },
  {   27,   "Pad 3",   41.0 },
  {   32,   "Pad 4",   41.0 },
  {   33,   "Pad 5",   41.0 },
};
const int PAD_COUNT = sizeof(PADS) / sizeof(PADS[0]);

// The secret order, as pad numbers from PADS[] above.
const int SEQUENCE[] = {3, 0, 5};
const int SEQUENCE_LENGTH = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

// Longest allowed pause between touches before the attempt resets.
const unsigned long MAX_GAP_MS = 10000;

const int TRIGGER_PIN = 25;
const bool TRIGGER_ACTIVE_HIGH = true;  // false for active-low relay modules
const unsigned long TRIGGER_HOLD_MS = 3000;

// A reading must stay past the threshold this long to count as a touch -
// filters out a single noisy sample or a sleeve brushing past.
const unsigned long TOUCH_CONFIRM_MS = 60;

// Release uses a LOWER threshold than touch (hysteresis). Without it, a
// finger resting right at the threshold would flicker touched/untouched
// and register as a stream of separate touches.
const float RELEASE_RATIO = 0.6;
const unsigned long RELEASE_CONFIRM_MS = 100;

const unsigned long SAMPLE_INTERVAL_MS = 20;

// Baselines slowly follow long-term drift (temperature, humidity, the
// room filling with people over a party) while a pad is untouched.
// Small = slow; at 20ms samples, 0.001 is a time constant of ~20 seconds.
// It also quietly repairs a baseline taken with a hand near a pad at boot.
const float BASELINE_TRACKING_ALPHA = 0.001;

const unsigned long SETTLE_MS = 4000;
const int BASELINE_SAMPLES = 20;

// ============================================================
// State
// ============================================================

struct PadState {
  float baseline;
  bool enabled;
  bool touched;
  bool pending;                  // threshold crossed, waiting to confirm
  unsigned long pendingSinceMs;
  uint32_t lastValue;
  float lastDrop;
  float maxDrop;                 // biggest drop seen since the last baseline
};

PadState padState[PAD_COUNT];

int history[SEQUENCE_LENGTH];    // most recent touches, oldest first
int historyLength = 0;
int progress = 0;                // how many steps of SEQUENCE are matched
unsigned long lastTouchMs = 0;

bool configValid = false;
bool triggerActive = false;
unsigned long triggerStartedMs = 0;

unsigned long lastSampleMs = 0;
bool liveView = false;
unsigned long lastLiveViewMs = 0;

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TRIGGER_PIN, OUTPUT);
  setTrigger(false);

  Serial.println();
  Serial.println("=== Capacitive Touch Sequence Puzzle ===");
  Serial.print("Arduino-ESP32 core ");
  Serial.println(ESP_ARDUINO_VERSION_STR);

  configValid = validateConfig();
  if (!configValid) {
    Serial.println();
    Serial.println("*** CONFIG ERRORS ABOVE - puzzle disabled until fixed. ***");
    return;
  }

  // Create every touch channel up front. The first touchRead() on a pin
  // creates its channel, which restarts the whole touch controller -
  // better to get all of that out of the way before measuring anything.
  Serial.println("Initializing touch channels...");
  for (int i = 0; i < PAD_COUNT; i++) {
    touchRead(PADS[i].pin);
  }

  Serial.println("Letting readings settle - hands off the pads...");
  delay(SETTLE_MS);
  captureBaselines();

  printSequence();
  Serial.println();
  Serial.println("Ready.");
  printHelp();
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  handleSerial();

  if (!configValid) {
    return;
  }

  unsigned long now = millis();

  if (triggerActive && now - triggerStartedMs >= TRIGGER_HOLD_MS) {
    triggerActive = false;
    setTrigger(false);
    Serial.println("Trigger released.");
  }

  if (progress > 0 && now - lastTouchMs > MAX_GAP_MS) {
    resetProgress("too long between touches");
  }

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = now;

  for (int i = 0; i < PAD_COUNT; i++) {
    updatePad(i, now);
  }

  if (liveView && now - lastLiveViewMs >= 250) {
    lastLiveViewMs = now;
    printLiveView();
  }
}

// ============================================================
// Touch sensing
// ============================================================

// Percent the reading has fallen below baseline. Positive = touch.
void updatePad(int i, unsigned long now) {
  PadState &s = padState[i];
  if (!s.enabled) {
    return;
  }

  uint32_t value = touchRead(PADS[i].pin);
  float drop = (s.baseline - (float)value) * 100.0f / s.baseline;
  s.lastValue = value;
  s.lastDrop = drop;
  if (drop > s.maxDrop) {
    s.maxDrop = drop;
  }

  float touchAt = PADS[i].touchDropPercent;
  float releaseAt = touchAt * RELEASE_RATIO;

  if (!s.touched) {
    if (drop >= touchAt) {
      if (!s.pending) {
        s.pending = true;
        s.pendingSinceMs = now;
      } else if (now - s.pendingSinceMs >= TOUCH_CONFIRM_MS) {
        s.touched = true;
        s.pending = false;
        onPadTouched(i);
      }
    } else {
      s.pending = false;
      // Follow slow drift, but only while clearly untouched - a hand
      // hovering close must not drag the baseline down to meet it.
      if (drop < touchAt * 0.5f) {
        s.baseline += ((float)value - s.baseline) * BASELINE_TRACKING_ALPHA;
      }
    }
  } else {
    if (drop < releaseAt) {
      if (!s.pending) {
        s.pending = true;
        s.pendingSinceMs = now;
      } else if (now - s.pendingSinceMs >= RELEASE_CONFIRM_MS) {
        s.touched = false;
        s.pending = false;
      }
    } else {
      s.pending = false;
    }
  }
}

void onPadTouched(int pad) {
  Serial.printf("[TOUCH] %s (GPIO %u)\n", PADS[pad].name, PADS[pad].pin);
  processStep(pad);
}

void captureBaselines() {
  Serial.println("Measuring baselines - hands off the pads...");

  uint64_t sums[PAD_COUNT] = {0};
  for (int s = 0; s < BASELINE_SAMPLES; s++) {
    for (int i = 0; i < PAD_COUNT; i++) {
      sums[i] += touchRead(PADS[i].pin);
    }
    delay(20);
  }

  for (int i = 0; i < PAD_COUNT; i++) {
    PadState &s = padState[i];
    s.baseline = (float)(sums[i] / BASELINE_SAMPLES);
    s.enabled = s.baseline > 0;
    s.touched = false;
    s.pending = false;
    s.maxDrop = 0.0;

    Serial.printf("  %-8s GPIO %-2u  baseline %lu", PADS[i].name, PADS[i].pin,
                  (unsigned long)s.baseline);
    if (!s.enabled) {
      Serial.print("   <-- reads 0, pad DISABLED (not reporting, or shorted to ground)");
    }
    Serial.println();
  }
}

// ============================================================
// Sequence logic
// ============================================================

void processStep(int pad) {
  unsigned long now = millis();

  if (progress > 0 && now - lastTouchMs > MAX_GAP_MS) {
    resetProgress("too long between touches");
  }
  lastTouchMs = now;

  // Append to the history, sliding the oldest touch out when full.
  if (historyLength == SEQUENCE_LENGTH) {
    for (int i = 0; i < SEQUENCE_LENGTH - 1; i++) {
      history[i] = history[i + 1];
    }
    history[SEQUENCE_LENGTH - 1] = pad;
  } else {
    history[historyLength++] = pad;
  }

  int previous = progress;
  progress = longestMatchedPrefix();

  if (progress == SEQUENCE_LENGTH) {
    solved();
  } else if (progress == previous + 1) {
    Serial.printf("  Correct - step %d / %d. Next: %s\n", progress, SEQUENCE_LENGTH,
                  PADS[SEQUENCE[progress]].name);
  } else if (progress == 0) {
    Serial.println(previous > 0 ? "  Wrong pad - back to the start."
                                : "  Not the start of the sequence.");
  } else {
    Serial.printf("  Wrong order - but that restarts the sequence at step %d / %d.\n",
                  progress, SEQUENCE_LENGTH);
  }
}

// The longest run at the END of the recent touches that matches the
// START of the sequence. That is how far along the guest currently is,
// and it's what lets a fumbled attempt recover without a manual reset.
int longestMatchedPrefix() {
  for (int k = historyLength; k > 0; k--) {
    bool match = true;
    for (int j = 0; j < k; j++) {
      if (history[historyLength - k + j] != SEQUENCE[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      return k;
    }
  }
  return 0;
}

void solved() {
  Serial.println();
  Serial.println("*** SEQUENCE CORRECT - TRIGGERING ***");
  setTrigger(true);
  triggerActive = true;
  triggerStartedMs = millis();
  historyLength = 0;
  progress = 0;
}

void resetProgress(const char *reason) {
  if (progress > 0) {
    Serial.printf("  (%s - resetting)\n", reason);
  }
  historyLength = 0;
  progress = 0;
}

void setTrigger(bool on) {
  digitalWrite(TRIGGER_PIN, (on == TRIGGER_ACTIVE_HIGH) ? HIGH : LOW);
}

// ============================================================
// Config validation
// ============================================================

bool validateConfig() {
  bool ok = true;

  for (int i = 0; i < PAD_COUNT; i++) {
    uint8_t pin = PADS[i].pin;

    if (digitalPinToTouchChannel(pin) < 0) {
      Serial.printf("  ERROR: %s uses GPIO %u, which is not a touch pin.\n", PADS[i].name, pin);
      ok = false;
    }
    if (pin == TRIGGER_PIN) {
      Serial.printf("  ERROR: %s uses GPIO %u, which is also TRIGGER_PIN.\n", PADS[i].name, pin);
      ok = false;
    }
    if (pin == 0 || pin == 2 || pin == 12 || pin == 15) {
      Serial.printf("  WARNING: %s uses strapping pin GPIO %u - it may affect booting.\n",
                    PADS[i].name, pin);
    }
    if (PADS[i].touchDropPercent <= 0) {
      Serial.printf("  ERROR: %s has a touch threshold of %.1f%% - must be above 0.\n",
                    PADS[i].name, PADS[i].touchDropPercent);
      ok = false;
    }
    for (int j = 0; j < i; j++) {
      if (PADS[j].pin == pin) {
        Serial.printf("  ERROR: %s and %s both use GPIO %u.\n", PADS[j].name, PADS[i].name, pin);
        ok = false;
      }
    }
  }

  for (int i = 0; i < SEQUENCE_LENGTH; i++) {
    if (SEQUENCE[i] < 0 || SEQUENCE[i] >= PAD_COUNT) {
      Serial.printf("  ERROR: SEQUENCE step %d is pad %d, but pads are numbered 0-%d.\n",
                    i + 1, SEQUENCE[i], PAD_COUNT - 1);
      ok = false;
    }
  }

  return ok;
}

// ============================================================
// Serial commands and output
// ============================================================

void handleSerial() {
  static String line = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      if (line.length() > 0) {
        handleCommand(line);
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 16) {
        line = "";  // runaway guard
      }
    }
  }
}

void handleCommand(String cmd) {
  cmd.toLowerCase();

  if (cmd == "h" || cmd == "?") {
    printHelp();
    return;
  }

  if (!configValid) {
    Serial.println("Puzzle disabled - fix the config errors printed at startup.");
    return;
  }

  if (cmd == "m") {
    printMaxDrops();
  } else if (cmd == "v") {
    liveView = !liveView;
    Serial.println(liveView ? "Live view ON (type v again to stop)." : "Live view OFF.");
  } else if (cmd == "b") {
    resetProgress("re-baselining");
    captureBaselines();
  } else if (isAllDigits(cmd)) {
    int pad = cmd.toInt();
    if (pad < 0 || pad >= PAD_COUNT) {
      Serial.printf("No pad %d - pads are numbered 0-%d.\n", pad, PAD_COUNT - 1);
      return;
    }
    Serial.printf("[TYPED] %s (GPIO %u)\n", PADS[pad].name, PADS[pad].pin);
    processStep(pad);
  } else {
    Serial.printf("Unknown command \"%s\" - type h for help.\n", cmd.c_str());
  }
}

bool isAllDigits(const String &s) {
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) {
      return false;
    }
  }
  return s.length() > 0;
}

void printSequence() {
  Serial.println();
  Serial.printf("Sequence (%d steps): ", SEQUENCE_LENGTH);
  for (int i = 0; i < SEQUENCE_LENGTH; i++) {
    Serial.printf("%s (GPIO %u)", PADS[SEQUENCE[i]].name, PADS[SEQUENCE[i]].pin);
    if (i < SEQUENCE_LENGTH - 1) {
      Serial.print(" -> ");
    }
  }
  Serial.println();
}

// Tuning aid for pads mounted behind a real surface. Press each pad the
// way a guest will - through the box, with a fingertip, not a whole
// palm - then run this. It reports the biggest drop each pad produced
// and suggests a threshold at half of it.
//
// Thresholds measured on bare foil are useless once the pad is behind
// cardboard: the material puts distance between finger and pad, so the
// drop shrinks a lot. That is why this measures in place.
void printMaxDrops() {
  Serial.println();
  Serial.println("Biggest drop seen per pad since the last baseline:");
  for (int i = 0; i < PAD_COUNT; i++) {
    const PadState &s = padState[i];
    Serial.print("  ");
    Serial.print(PADS[i].name);
    Serial.print("  GPIO ");
    Serial.print(PADS[i].pin);
    if (!s.enabled) {
      Serial.println("  disabled (baseline read 0)");
      continue;
    }
    Serial.print("  max ");
    Serial.print(s.maxDrop, 1);
    Serial.print("%   threshold now ");
    Serial.print(PADS[i].touchDropPercent, 1);
    Serial.print("%   suggested ");
    Serial.print(s.maxDrop / 2.0, 1);
    Serial.print("%");
    if (s.maxDrop < PADS[i].touchDropPercent) {
      Serial.print("   <-- never reaches the current threshold");
    }
    Serial.println();
  }
  Serial.println("  Put the suggested values into PADS[] and reflash.");
  Serial.println("  A max under about 3% is too close to the noise to be");
  Serial.println("  reliable - use a bigger pad, thinner material, or a tack.");
  Serial.println("  Type b to clear these and start measuring again.");
}

void printLiveView() {
  for (int i = 0; i < PAD_COUNT; i++) {
    const PadState &s = padState[i];
    if (!s.enabled) {
      Serial.printf("%d: off          ", i);
      continue;
    }
    Serial.printf("%d:%6lu %5.1f%%%c  ", i, (unsigned long)s.lastValue, s.lastDrop,
                  s.touched ? '*' : ' ');
  }
  Serial.println();
}

void printHelp() {
  Serial.println("Commands (line ending: Newline):");
  Serial.printf("  0-%d  simulate touching that pad\n", PAD_COUNT - 1);
  Serial.println("  v    toggle live readings (* = touched)");
  Serial.println("  m    biggest drop seen per pad + suggested thresholds");
  Serial.println("  b    re-measure baselines - hands off!");
  Serial.println("  h    this help");
}
