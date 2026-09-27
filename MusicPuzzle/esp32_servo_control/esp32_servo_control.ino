/*
  Children of the Pier - MIDI Box Puzzle - ESP32 (BLE MIDI + Servo)
  ---------------------------------------------------------------
  Listens to the Roland FP-30 directly over Bluetooth LE MIDI. When
  the correct sequence of notes is played, slowly sweeps the servo
  from CLOSED_ANGLE to OPEN_ANGLE to reveal the contents.

  This replaces the old setup where a PC ran midi_listener.py over USB
  MIDI and poked the ESP32 over serial. No computer needed at the party
  now - just the piano and this board.

  Libraries needed (install via Tools > Manage Libraries):
    - ESP32Servo          (Kevin Harrington / madhephaestus)
    - ESP32-BLE-MIDI      (Maxime ANDRE)  <- pulls in NimBLE-Arduino
    - NimBLE-Arduino 2.x  (installed automatically as a dependency)

  ----------------------------------------------------------------------
  HOW THE BLUETOOTH SIDE WORKS
  ----------------------------------------------------------------------
  BLE MIDI has two roles. The FP-30 is a PERIPHERAL: it advertises and
  waits. Phones and tablets are CENTRALS: they scan and connect. So the
  ESP32 has to be the central here - it does the scanning and initiates
  the connection, the same way an iPad would.

  FIRST RUN: leave PIANO_NAME_MATCH as-is and watch the Serial Monitor.
  Every BLE device in range is printed with its name and MAC address.
  Find the FP-30 in that list, then either:
    - set PIANO_NAME_MATCH to a distinctive part of its name, or
    - better, paste its MAC into PIANO_MAC_MATCH (names can be blank
      or duplicated; a MAC is unique).

  The piano's Bluetooth MIDI must be switched on first. On the FP-30:
  hold [Function] + lowest A key for MIDI mode, then [Function] +
  lowest B key for Bluetooth. It must NOT be already paired to a phone
  or tablet - BLE MIDI peripherals accept one central at a time, so
  close Piano Partner / disconnect the iPad before testing this.

  ----------------------------------------------------------------------
  SERIAL MONITOR TEST COMMANDS
  ----------------------------------------------------------------------
  Set the Serial Monitor to 115200 baud and "Newline" line ending, then
  type a command and press Enter. These work with or without the piano
  connected, so the servo and sequence logic can be tested on the bench.
    o      open the box now
    c      close the box now
    n60    pretend note 60 was played (any 0-127)
    s      show status
    l      toggle listen-only mode
    h      show this list
  While the piano isn't connected, each BLE scan blocks for a few
  seconds, so commands may take a moment to respond.

  Wiring:
    Servo signal wire -> GPIO 18 (or change SERVO_PIN below)
    Servo power (red)  -> 5V (see note below)
    Servo ground (brown/black) -> GND (shared with ESP32 GND)

  Power note: an SG90 can often draw enough current that it's worth
  powering it from a separate 5V supply rather than the ESP32's own
  5V pin, especially if anything else is drawing power at the same
  time. If you see the ESP32 resetting or acting glitchy when the
  servo moves, that's the classic symptom - move the servo to its
  own supply and just share ground with the ESP32. Radios make this
  worse, not better: BLE draws current in bursts too.
*/

#include <ESP32Servo.h>
#include <BLEMidi.h>

// ============================================================
// CONFIG
// ============================================================

// Identify the piano among the BLE devices in range. Matching is
// case-sensitive and looks for this text ANYWHERE in the advertised
// name, so a fragment is fine. Leave empty ("") to match on MAC only.
const char *PIANO_NAME_MATCH = "FP-30";

// Preferred once you know it: the piano's MAC address, exactly as
// printed in the scan list (lowercase, colon-separated). Takes priority
// over the name match. Empty string disables it.
const char *PIANO_MAC_MATCH = "e1:63:47:6f:61:0f";

// Set true to print every note you play and never check the sequence
// or move the servo. Use this to work out the MIDI note numbers for
// your chosen tune, then set it back to false. This is the starting
// value; the "l" serial command toggles it at runtime.
const bool LISTEN_ONLY_MODE = false;

// The secret sequence as MIDI note numbers. Middle C = 60, and each
// key going up (white or black) is +1.
const int SEQUENCE_LENGTH = 4;
const uint8_t TARGET_SEQUENCE[SEQUENCE_LENGTH] = {60, 57, 67, 64};  // C A G E

// If notes come in with too long a pause between them the buffer
// resets, so nobody can noodle through every key and stumble into it.
// Not so tight that a slow, deliberate performance gets punished.
const unsigned long MAX_GAP_MS = 4000;

const int SERVO_PIN = 18;

const int CLOSED_ANGLE = 0;
const int OPEN_ANGLE = 90;

// Pulse widths for 0 and 180 degrees. Positions are sent in
// microseconds rather than whole degrees, so the sweep has much finer
// steps than 1 degree.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

// How long the opening sweep takes, start to finish. Higher = slower,
// more dramatic reveal. Tune this once it's actually mounted in the box.
const unsigned long OPEN_SWEEP_MS = 5500;

// How long the box stays open before it closes itself again. Timer
// starts once the box has finished opening, not from the trigger.
const unsigned long OPEN_DURATION_MS = 15000;

// Closing can be faster than opening - the "reveal" is the slow part,
// the box quietly resetting itself doesn't need the same drama.
const unsigned long CLOSE_SWEEP_MS = 1500;

// A new position is sent every servo frame (50Hz = 20ms). Updating
// less often than this is what makes a slow sweep look steppy.
const unsigned long SERVO_FRAME_MS = 20;

// How long to wait between scan attempts when not connected.
const unsigned long RESCAN_DELAY_MS = 2000;

// ============================================================
// State
// ============================================================

Servo boxServo;
bool isOpen = false;
bool waitingToClose = false;
unsigned long openedAtMillis = 0;

uint8_t buffer[SEQUENCE_LENGTH];
int bufferLength = 0;
unsigned long lastNoteMs = 0;

unsigned long lastScanMs = 0;
bool wasConnected = false;

bool listenOnly = LISTEN_ONLY_MODE;

// Notes arrive on the BLE stack's own task, NOT on the loop() task.
// Doing anything slow in that callback - and the servo sweep takes
// several SECONDS - would stall the Bluetooth stack and drop the
// connection. So the callback does the absolute minimum: drop the note
// number into this queue. loop() picks it up and does the real work.
QueueHandle_t noteQueue;

// ============================================================
// BLE MIDI callbacks - keep these fast and non-blocking
// ============================================================

void onNoteOn(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp) {
  // Most keyboards, the FP-30 included, send "note off" as a note-on
  // with velocity 0. Treat those as releases and ignore them.
  if (velocity == 0) {
    return;
  }
  xQueueSend(noteQueue, &note, 0);  // 0 = never block in a BLE callback
}

void onBleConnect() {
  Serial.println("[BLE] Connected to the piano.");
}

void onBleDisconnect() {
  Serial.println("[BLE] Disconnected - will rescan.");
}

// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  noteQueue = xQueueCreate(32, sizeof(uint8_t));

  // Allows the servo to use standard 50Hz timing on the ESP32
  ESP32PWM::allocateTimer(0);
  boxServo.setPeriodHertz(50);
  boxServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  boxServo.write(CLOSED_ANGLE);
  delay(500);

  Serial.println();
  Serial.println("=== MIDI Box Puzzle (BLE) ===");
  if (listenOnly) {
    Serial.println("LISTEN ONLY mode - notes are printed, servo will not move.");
  } else {
    Serial.print("Target sequence: ");
    for (int i = 0; i < SEQUENCE_LENGTH; i++) {
      Serial.print(TARGET_SEQUENCE[i]);
      Serial.print(i < SEQUENCE_LENGTH - 1 ? ", " : "\n");
    }
  }

  Serial.println("Starting Bluetooth...");
  BLEMidiClient.begin("Pier Music Box");
  BLEMidiClient.setNoteOnCallback(onNoteOn);
  BLEMidiClient.setOnConnectCallback(onBleConnect);
  BLEMidiClient.setOnDisconnectCallback(onBleDisconnect);

  Serial.println("Make sure the piano's Bluetooth MIDI is ON and that no");
  Serial.println("phone or tablet is already connected to it.");
  Serial.println();
  printHelp();
}

void loop() {
  handleSerial();

  // Non-blocking check: has the box been open long enough to close
  // itself? Using millis() here instead of delay() means the ESP32
  // is still free to service Bluetooth while it waits. Checked before
  // the connection test so a box opened from the bench still closes.
  if (waitingToClose && (millis() - openedAtMillis >= OPEN_DURATION_MS)) {
    Serial.println("Auto-close timer elapsed - closing box.");
    closeBox();
  }

  if (!BLEMidiClient.isConnected()) {
    if (wasConnected) {
      wasConnected = false;
      resetAttempt("connection lost");
    }
    if (millis() - lastScanMs >= RESCAN_DELAY_MS) {
      lastScanMs = millis();
      scanAndConnect();
    }
    return;  // nothing else to do until we have a piano
  }

  wasConnected = true;

  // Drain any notes the BLE task queued up since the last pass.
  uint8_t note;
  while (xQueueReceive(noteQueue, &note, 0) == pdTRUE) {
    handleNote(note);
  }

  // Reset a stalled attempt after too long a gap between notes.
  if (bufferLength > 0 && (millis() - lastNoteMs > MAX_GAP_MS)) {
    resetAttempt("too slow");
  }
}

// Scans for BLE devices and connects to the piano.
//
// Since v0.4.0 this library returns EVERY BLE device it sees, not just
// MIDI ones - the filtering is our job. That is actually helpful here:
// printing the whole list is how you find the FP-30's name and MAC in
// the first place.
void scanAndConnect() {
  Serial.println();
  Serial.println("Scanning for BLE devices...");
  unsigned int count = BLEMidiClient.scan();

  if (count == 0) {
    Serial.println("  No devices found. Is the piano's Bluetooth switched on?");
    return;
  }

  Serial.printf("  Found %u device(s):\n", count);
  int matchIndex = -1;

  for (unsigned int i = 0; i < count; i++) {
    String name = BLEMidiClient.deviceName(i);
    String mac = BLEMidiClient.deviceMacAddress(i);

    Serial.printf("   [%u] \"%s\"  %s", i, name.c_str(), mac.c_str());

    bool macHit = (strlen(PIANO_MAC_MATCH) > 0 && mac.equalsIgnoreCase(PIANO_MAC_MATCH));
    bool nameHit = (strlen(PIANO_NAME_MATCH) > 0 && name.indexOf(PIANO_NAME_MATCH) >= 0);

    if (macHit || nameHit) {
      Serial.print("   <-- match");
      if (matchIndex < 0) {
        matchIndex = i;
      }
    }
    Serial.println();
  }

  if (matchIndex < 0) {
    Serial.println("  No match. Copy the piano's name or MAC from the list");
    Serial.println("  above into PIANO_NAME_MATCH / PIANO_MAC_MATCH.");
    return;
  }

  Serial.printf("Connecting to device [%d]...\n", matchIndex);
  if (BLEMidiClient.connect(matchIndex)) {
    Serial.println("Connected. Play the sequence.");
  } else {
    Serial.println("Connection failed. If the piano is paired to a phone or");
    Serial.println("tablet, disconnect that first - it accepts one at a time.");
  }
}

void handleNote(uint8_t note) {
  if (listenOnly) {
    Serial.print("Note played: ");
    Serial.println(note);
    return;
  }

  // Gap check happens here too, not just in loop(), so a note arriving
  // after a long pause starts a fresh attempt rather than appending to
  // a stale buffer.
  if (bufferLength > 0 && (millis() - lastNoteMs > MAX_GAP_MS)) {
    resetAttempt("too slow");
  }
  lastNoteMs = millis();

  // Keep the buffer to the sequence length by sliding it along, so a
  // guest who plays a few wrong notes and then the right phrase still
  // succeeds - no need to stop and start over cleanly.
  if (bufferLength == SEQUENCE_LENGTH) {
    for (int i = 0; i < SEQUENCE_LENGTH - 1; i++) {
      buffer[i] = buffer[i + 1];
    }
    buffer[SEQUENCE_LENGTH - 1] = note;
  } else {
    buffer[bufferLength++] = note;
  }

  Serial.print("Note played: ");
  Serial.print(note);
  Serial.print("   buffer: ");
  for (int i = 0; i < bufferLength; i++) {
    Serial.print(buffer[i]);
    Serial.print(' ');
  }
  Serial.println();

  if (bufferLength < SEQUENCE_LENGTH) {
    return;
  }

  for (int i = 0; i < SEQUENCE_LENGTH; i++) {
    if (buffer[i] != TARGET_SEQUENCE[i]) {
      return;
    }
  }

  Serial.println();
  Serial.println("*** CORRECT SEQUENCE - OPENING BOX ***");
  bufferLength = 0;

  if (!isOpen) {
    openBox();
  }
}

void resetAttempt(const char *reason) {
  if (bufferLength > 0) {
    Serial.print("  (");
    Serial.print(reason);
    Serial.println(" - resetting)");
  }
  bufferLength = 0;
}

// Opens the box and starts the auto-close timer.
void openBox() {
  openBoxSlowly();
  isOpen = true;
  waitingToClose = true;
  openedAtMillis = millis();
}

void closeBox() {
  closeBoxSlowly();
  isOpen = false;
  waitingToClose = false;
}

void openBoxSlowly() {
  sweepServo(CLOSED_ANGLE, OPEN_ANGLE, OPEN_SWEEP_MS);
  Serial.println("Box open.");
}

void closeBoxSlowly() {
  sweepServo(OPEN_ANGLE, CLOSED_ANGLE, CLOSE_SWEEP_MS);
  Serial.println("Box closed. Ready for next trigger.");
}

int angleToUs(int angle) {
  return SERVO_MIN_US + (long)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
}

// Moves the servo from one angle to another over durationMs, updating
// every servo frame. Position follows an ease-in-out curve so the lid
// starts and stops gently instead of lurching, and is timed off
// millis() so the total duration stays accurate.
void sweepServo(int fromAngle, int toAngle, unsigned long durationMs) {
  int fromUs = angleToUs(fromAngle);
  int toUs = angleToUs(toAngle);
  unsigned long start = millis();
  unsigned long elapsed;

  while ((elapsed = millis() - start) < durationMs) {
    float t = (float)elapsed / durationMs;
    float eased = 0.5f - 0.5f * cosf(PI * t);
    boxServo.writeMicroseconds(fromUs + lroundf((toUs - fromUs) * eased));
    delay(SERVO_FRAME_MS);
  }
  boxServo.writeMicroseconds(toUs);
}

// ============================================================
// Serial Monitor test commands
// ============================================================

// Collects typed characters into a line without blocking, then runs
// it as a command when Enter (newline) arrives.
void handleSerial() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line.trim();
      if (line.length() > 0) {
        runCommand(line);
      }
      line = "";
    } else {
      line += ch;
    }
  }
}

void runCommand(const String &cmd) {
  char c = tolower(cmd.charAt(0));

  if (c == 'o') {
    if (isOpen) {
      Serial.println("[CMD] Box is already open.");
      return;
    }
    Serial.println("[CMD] Opening box.");
    openBox();
  } else if (c == 'c') {
    if (!isOpen) {
      Serial.println("[CMD] Box is already closed.");
      return;
    }
    Serial.println("[CMD] Closing box.");
    closeBox();
  } else if (c == 'n') {
    String arg = cmd.substring(1);
    arg.trim();
    bool digitsOnly = arg.length() > 0;
    for (unsigned int i = 0; i < arg.length(); i++) {
      if (!isDigit(arg.charAt(i))) {
        digitsOnly = false;
      }
    }
    int note = arg.toInt();
    if (!digitsOnly || note > 127) {
      Serial.println("[CMD] Usage: n<note>, e.g. n60 (0-127).");
      return;
    }
    Serial.printf("[CMD] Simulating note %d.\n", note);
    handleNote((uint8_t)note);
  } else if (c == 's') {
    printStatus();
  } else if (c == 'l') {
    listenOnly = !listenOnly;
    resetAttempt("mode changed");
    Serial.println(listenOnly ? "[CMD] Listen-only mode ON - servo will not move."
                              : "[CMD] Listen-only mode OFF - puzzle active.");
  } else if (c == 'h' || c == '?') {
    printHelp();
  } else {
    Serial.print("[CMD] Unknown command: ");
    Serial.println(cmd);
    printHelp();
  }
}

void printStatus() {
  Serial.println("--- Status ---");
  Serial.printf("  Piano:  %s\n", BLEMidiClient.isConnected() ? "connected" : "not connected");
  Serial.printf("  Box:    %s", isOpen ? "open" : "closed");
  if (waitingToClose) {
    unsigned long elapsed = millis() - openedAtMillis;
    unsigned long remaining = elapsed < OPEN_DURATION_MS ? OPEN_DURATION_MS - elapsed : 0;
    Serial.printf(" (auto-close in %lu s)", remaining / 1000);
  }
  Serial.println();
  Serial.printf("  Mode:   %s\n", listenOnly ? "listen only" : "puzzle");
  Serial.print("  Target: ");
  for (int i = 0; i < SEQUENCE_LENGTH; i++) {
    Serial.print(TARGET_SEQUENCE[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("  Buffer: ");
  for (int i = 0; i < bufferLength; i++) {
    Serial.print(buffer[i]);
    Serial.print(' ');
  }
  Serial.println(bufferLength == 0 ? "(empty)" : "");
}

void printHelp() {
  Serial.println("Serial commands (Newline line ending):");
  Serial.println("  o      open the box now");
  Serial.println("  c      close the box now");
  Serial.println("  n60    pretend note 60 was played");
  Serial.println("  s      show status");
  Serial.println("  l      toggle listen-only mode");
  Serial.println("  h      show this list");
}
