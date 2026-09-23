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
// your chosen tune, then set it back to false.
const bool LISTEN_ONLY_MODE = false;

// The secret sequence as MIDI note numbers. Middle C = 60, and each
// key going up (white or black) is +1.
const int SEQUENCE_LENGTH = 4;
const uint8_t TARGET_SEQUENCE[SEQUENCE_LENGTH] = {60, 57, 67, 64};  // C E G C

// If notes come in with too long a pause between them the buffer
// resets, so nobody can noodle through every key and stumble into it.
// Not so tight that a slow, deliberate performance gets punished.
const unsigned long MAX_GAP_MS = 4000;

const int SERVO_PIN = 18;

const int CLOSED_ANGLE = 0;
const int OPEN_ANGLE = 90;

// Delay in milliseconds between each 1-degree step. Higher = slower,
// more dramatic reveal. Tune this once it's actually mounted in the
// box - 60ms/degree over a 90 degree sweep is about 5.5 seconds.
const int STEP_DELAY_MS = 60;

// How long the box stays open before it closes itself again. Timer
// starts once the box has finished opening, not from the trigger.
const unsigned long OPEN_DURATION_MS = 15000;

// Closing can be faster than opening - the "reveal" is the slow part,
// the box quietly resetting itself doesn't need the same drama.
const int CLOSE_STEP_DELAY_MS = 15;

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
  boxServo.attach(SERVO_PIN, 500, 2400);
  boxServo.write(CLOSED_ANGLE);
  delay(500);

  Serial.println();
  Serial.println("=== MIDI Box Puzzle (BLE) ===");
  if (LISTEN_ONLY_MODE) {
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
}

void loop() {
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

  // Non-blocking check: has the box been open long enough to close
  // itself? Using millis() here instead of delay() means the ESP32
  // is still free to service Bluetooth while it waits.
  if (waitingToClose && (millis() - openedAtMillis >= OPEN_DURATION_MS)) {
    Serial.println("Auto-close timer elapsed - closing box.");
    closeBoxSlowly();
    isOpen = false;
    waitingToClose = false;
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
  if (LISTEN_ONLY_MODE) {
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

  if (!isOpen && !waitingToClose) {
    openBoxSlowly();
    isOpen = true;
    waitingToClose = true;
    openedAtMillis = millis();
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

void openBoxSlowly() {
  for (int angle = CLOSED_ANGLE; angle <= OPEN_ANGLE; angle++) {
    boxServo.write(angle);
    delay(STEP_DELAY_MS);
  }
  Serial.println("Box open.");
}

void closeBoxSlowly() {
  for (int angle = OPEN_ANGLE; angle >= CLOSED_ANGLE; angle--) {
    boxServo.write(angle);
    delay(CLOSE_STEP_DELAY_MS);
  }
  Serial.println("Box closed. Ready for next trigger.");
}
