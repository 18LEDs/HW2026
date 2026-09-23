/*
  Children of the Pier - Morse Code Telegraph Puzzle - ESP32 Firmware
  ----------------------------------------------------------------------
  Reads a telegraph key (simple momentary switch) wired to a digital
  pin. Classifies each press as a dot or dash based on duration,
  decodes the resulting Morse sequence into letters, and checks the
  decoded message against a target phrase. Shows live feedback on an
  OLED display. Sets TRIGGER_PIN HIGH when the correct phrase is
  entered.

  Libraries needed (Arduino IDE > Tools > Manage Libraries):
    - "Adafruit SSD1306"
    - "Adafruit GFX Library"
  (Only needed if USE_OLED is true below - skip installing these if
  you decide to build without the display.)

  Wiring - telegraph key:
    One terminal -> GPIO 4 (or change KEY_PIN below)
    Other terminal -> GND
    10k resistor from GPIO 4 to 3.3V (pull-up) - see note below

  Wiring - OLED (I2C, SSD1306):
    VCC -> 3.3V
    GND -> GND
    SDA -> GPIO 21
    SCL -> GPIO 22

  Wiring - trigger output:
    TRIGGER_PIN -> whatever relay/MOSFET/servo circuit ends up
    driving the physical reveal, once that's decided.

  Pull-up vs pull-down note: this sketch assumes a pull-UP resistor
  (key reads HIGH when idle, LOW when pressed), since that's the more
  common wiring for a simple momentary switch like this and avoids
  a floating pin. If you'd rather wire it as a pull-down (idle LOW,
  pressed HIGH) from the kit you already bought, just flip the logic
  in readKeyState() below - it's one line.
*/

#include <Wire.h>

#define USE_OLED true

#if USE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

// ============================================================
// CONFIG
// ============================================================

const int KEY_PIN = 4;
const int TRIGGER_PIN = 5;

// The phrase guests must key in, in plain letters. Spaces are
// allowed and treated as word breaks. Keep it short for a party -
// something like "PIER" or "TIDE" rather than a full sentence.
const String TARGET_PHRASE = "PIER";

// Base timing unit in milliseconds. A dot is ~1 unit, a dash is ~3
// units. Real Morse operators vary a lot in speed - start around
// 150-200ms and adjust after watching someone actually use the key.
const unsigned long UNIT_MS = 150;

// Derived thresholds - shouldn't need to touch these directly, they
// scale off UNIT_MS above.
const unsigned long DOT_MAX_MS = UNIT_MS * 2;        // press shorter than this = dot
const unsigned long LETTER_GAP_MS = UNIT_MS * 3;     // silence longer than this = end of letter
const unsigned long WORD_GAP_MS = UNIT_MS * 7;       // silence longer than this = end of word
const unsigned long MESSAGE_TIMEOUT_MS = UNIT_MS * 15; // silence this long = check/reset attempt

// Mechanical switches "bounce" - the contacts physically vibrate for
// a few milliseconds on press/release before settling, which can
// register as several rapid presses instead of one. This is how
// long the pin must hold a stable state before we trust it. 20-30ms
// is a safe range for most switches; raise it if you still see
// double-registers, lower it if fast taps start getting missed.
const unsigned long DEBOUNCE_DELAY_MS = 25;

// ============================================================
// Morse lookup table
// ============================================================

struct MorseEntry {
  char letter;
  const char* code;
};

const MorseEntry MORSE_TABLE[] = {
  {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},
  {'E', "."},     {'F', "..-."},  {'G', "--."},   {'H', "...."},
  {'I', ".."},    {'J', ".---"},  {'K', "-.-"},   {'L', ".-.."},
  {'M', "--"},    {'N', "-."},    {'O', "---"},   {'P', ".--."},
  {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
  {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},
  {'Y', "-.--"},  {'Z', "--.."},
  {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"},
  {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."},
  {'9', "----."}, {'0', "-----"}
};
const int MORSE_TABLE_SIZE = sizeof(MORSE_TABLE) / sizeof(MorseEntry);

char decodeMorse(const String& symbol) {
  for (int i = 0; i < MORSE_TABLE_SIZE; i++) {
    if (symbol == MORSE_TABLE[i].code) {
      return MORSE_TABLE[i].letter;
    }
  }
  return '?';  // unrecognized sequence
}

// ============================================================
// State
// ============================================================

bool keyIsDown = false;
unsigned long pressStartMs = 0;
unsigned long releaseStartMs = 0;

// Debounce tracking - separate from keyIsDown, which only updates
// once a reading has been stable for DEBOUNCE_DELAY_MS.
bool lastRawReading = false;
unsigned long lastRawChangeMs = 0;

String currentSymbol = "";   // dots/dashes for the letter in progress
String decodedMessage = "";  // letters decoded so far this attempt

bool waitingForTimeout = false;

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

#if USE_OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found - check wiring/address (0x3C or 0x3D).");
  }
  display.clearDisplay();
#endif

  showMessage("Ready.\nAwaiting\nthe key...");
  Serial.println("Ready. Awaiting the key...");
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  bool keyDownNow = readKeyState();
  unsigned long now = millis();

  // --- Key just pressed ---
  if (keyDownNow && !keyIsDown) {
    keyIsDown = true;
    pressStartMs = now;
    waitingForTimeout = false;
  }

  // --- Key just released ---
  if (!keyDownNow && keyIsDown) {
    keyIsDown = false;
    unsigned long pressDuration = now - pressStartMs;
    releaseStartMs = now;

    if (pressDuration < DOT_MAX_MS) {
      currentSymbol += ".";
    } else {
      currentSymbol += "-";
    }
    showMessage(decodedMessage + "\n" + currentSymbol);
  }

  // --- Check gaps while key is up ---
  if (!keyIsDown && currentSymbol.length() > 0) {
    unsigned long silence = now - releaseStartMs;

    if (silence > LETTER_GAP_MS) {
      char letter = decodeMorse(currentSymbol);
      decodedMessage += letter;
      currentSymbol = "";
      Serial.println("Decoded letter: " + String(letter) + "   message so far: " + decodedMessage);
      showMessage(decodedMessage);
      waitingForTimeout = true;
    }
  }

  // --- Long pause: check the attempt ---
  if (waitingForTimeout && decodedMessage.length() > 0) {
    unsigned long silence = now - releaseStartMs;
    if (silence > MESSAGE_TIMEOUT_MS) {
      checkAttempt();
      waitingForTimeout = false;
    }
  }
}

// ============================================================
// Helpers
// ============================================================

bool readKeyState() {
  // INPUT_PULLUP: idle HIGH, pressed pulls to LOW.
  bool rawPressed = digitalRead(KEY_PIN) == LOW;
  unsigned long now = millis();

  // Whenever the raw reading changes, restart the debounce timer -
  // we're waiting for it to stay put, not just checking once.
  if (rawPressed != lastRawReading) {
    lastRawChangeMs = now;
    lastRawReading = rawPressed;
  }

  // Only trust the reading once it's held steady long enough to
  // rule out bounce. Until then, report whatever the last STABLE
  // state was (keyIsDown) rather than the possibly-bouncing raw value.
  if ((now - lastRawChangeMs) > DEBOUNCE_DELAY_MS) {
    return rawPressed;
  }
  return keyIsDown;
}

void checkAttempt() {
  Serial.println("Checking attempt: '" + decodedMessage + "' against target '" + TARGET_PHRASE + "'");

  if (decodedMessage == TARGET_PHRASE) {
    Serial.println("*** CORRECT ***");
    showMessage("Correct!\n\nUNDER");
    digitalWrite(TRIGGER_PIN, HIGH);
    delay(3000);  // hold trigger high briefly; adjust once you know what it's driving
    digitalWrite(TRIGGER_PIN, LOW);
  } else {
    Serial.println("Incorrect - resetting.");
    showMessage("Try again.");
    delay(1500);
  }

  decodedMessage = "";
  currentSymbol = "";
  showMessage("Ready.\nAwaiting\nthe key...");
}

void showMessage(const String& msg) {
#if USE_OLED
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
#else
  // No OLED - rely on Serial output for feedback instead.
  (void)msg;
#endif
}
