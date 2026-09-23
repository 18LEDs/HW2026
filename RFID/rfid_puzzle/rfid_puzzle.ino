/*
  Children of the Pier - RFID Relic Puzzle - ESP32 Firmware
  ----------------------------------------------------------------------
  Guests tap collected "relic" tags onto the reader one at a time, in
  the order they believe is correct. Checks each tag's written
  identifier (written ahead of time with rfid_tag_writer.ino) against
  the expected order. A correct full sequence opens the box with a
  servo and switches on a Kasa smart plug.

  Library needed: "MFRC522" by GithubCommunity (same as the tag
  writer sketch).

  Wiring - MFRC522 reader: identical to rfid_tag_writer.ino, see that
  file's header comment for the full pin list.

  Wiring - SG90 servo:
    Orange (signal) -> GPIO 26
    Red (+5V)       -> 5V on its OWN supply, not the ESP32's 5V pin
    Brown (ground)  -> that supply's ground AND the ESP32 GND

    The two grounds must be joined or the servo will not see a valid
    signal. The ESP32's 3.3V logic drives an SG90's signal input fine.

    Do NOT run an SG90 from the ESP32's 3.3V pin. Even on the 5V pin it
    can pull the rail down far enough to brown out the board when it
    stalls or starts moving - and here that also disturbs the MFRC522,
    which is a 3.3V part sharing the same regulator. A separate 5V
    supply with a 470-1000uF capacitor across it, close to the servo,
    is the reliable arrangement.

    Keep the servo's wires away from the reader's antenna. Motor noise
    next to a 13.56MHz coil costs you read range.

  Kasa smart plug: no wiring - it is switched over WiFi, and replaces
  the UV LED that used to run off a GPIO. Put your network details and
  the plug's IP in the CONFIG section below.

  On a solve the two outputs run strictly in sequence, never together:

      box sweeps open -> pause -> plug ON -> hold
      plug OFF (confirmed) -> pause -> box sweeps closed

  Keeping the motor and the radio apart matters on a shared 5V rail:
  their current peaks together are what resets the board.

  TESTING AIDS (Serial Monitor, 115200 baud, line ending Newline):
    TIDE etc   type an identifier to fake a tap - exercises the sequence
               logic and the reveal without the tags in hand
    Servo:
      O / C    jog to OPEN_ANGLE / CLOSED_ANGLE
      A45      jog to 45 degrees, for finding those angles
    Plug:
      ON/OFF   switch it directly
      TOGGLE   flip whatever it is doing now
      STATE    ask the plug what it is and whether its relay is on
      PING     just open a TCP connection - is anything there at all?
      FIND     broadcast-search the LAN for Kasa devices and list them
      IP <ip>  retarget the plug until reboot, e.g. IP 192.168.4.22
    Both:
      GO       run the whole reveal once: open, hold, close, plug off
      WIFI     report WiFi link status
      HELP     list these again
*/

#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <esp_system.h>

// The loop task's stack defaults to 8KB, and this sketch runs WiFi, SPI,
// the MFRC522 driver, Strings and the servo from it. Overflowing that
// stack corrupts whatever sits next to it in memory, and the usual way
// that shows up is a FreeRTOS assert deep inside an unrelated driver -
// for example "xTaskPriorityDisinherit ... pxTCB == pxCurrentTCBs" while
// the MFRC522 releases the SPI mutex. Type MEM to see how close to the
// limit it actually runs.
SET_LOOP_TASK_STACK_SIZE(16384);

// Set to 1 ONLY for testing in the Wokwi simulator. Wokwi's MFRC522
// exposes a card UID but no NTAG213 user memory, so the page-4
// identifier this puzzle relies on can't be read there. In SIM_MODE the
// reader is not initialized at all and taps come only from the Serial
// Monitor.
//
// MUST be 0 for the real hardware build - with it set, this sketch
// never talks to the reader and no amount of tapping will trigger it.
#define SIM_MODE 0

#define SS_PIN 5
#define RST_PIN 22

// SG90 signal wire. NOT 18/19/23 - those are the reader's SPI bus, and
// not 5 or 22 either. 26 is free on this build.
const int SERVO_PIN = 26;

MFRC522 mfrc522(SS_PIN, RST_PIN);

// ============================================================
// CONFIG
// ============================================================

// The page the identifier was written to (must match rfid_tag_writer.ino).
const byte READ_PAGE = 4;

// The correct order relics must be tapped in. These must exactly
// match the 4-character identifiers written onto each tag.
const int SEQUENCE_LENGTH = 3;
const String TARGET_SEQUENCE[SEQUENCE_LENGTH] = {"TIDE", "BONE", "SALT"};

// If too much time passes between taps, the attempt resets - stops
// someone from wandering off mid-attempt and coming back hours later
// to a half-completed sequence.
const unsigned long MAX_GAP_MS = 30000;

// Ignore a repeat read of the SAME tag within this window, so one long
// tap or a bit of hand-wobble is not counted as several taps.
//
// Why this is generous: a halted tag ignores REQA, but if it drifts out
// of the RF field for even an instant it resets to IDLE and answers
// again - so one unsteady hand becomes two reads. 800ms was not enough.
// Since consecutive relics in the sequence are different tags, locking
// out a repeat of the SAME tag for a few seconds costs nothing.
const unsigned long REMOVE_DEBOUNCE_MS = 3000;

// ---- Servo ----
// Find these two by experiment with the mechanism actually mounted:
// type "O" and "C" in the Serial Monitor to jog it open and closed.
const int CLOSED_ANGLE = 70;
const int OPEN_ANGLE = 115;

// Milliseconds per 1-degree step. Bigger = slower, more dramatic.
// 60ms across a 90 degree sweep is about 5.5 seconds.
const int OPEN_STEP_DELAY_MS = 10;

// Closing does not need the same drama as the reveal.
const int CLOSE_STEP_DELAY_MS = 10;

// How long it stays open before closing itself again. Counted from the
// moment it finishes opening, not from the solve.
const unsigned long OPEN_DURATION_MS = 15000;

// An SG90 buzzes and keeps drawing current while holding position. Once
// it has finished moving, cutting the signal lets it go quiet. The horn
// stays put unless something pushes it - fine for a lid, not for
// anything under load. Set false if yours sags when idle.
const bool DETACH_WHEN_IDLE = true;

// ---- WiFi / Kasa smart plug ----
const char *WIFI_SSID = "18LEDs";
const char *WIFI_PASSWORD = "HorseKickApple";

// The plug's address on your LAN. Give it a DHCP reservation in the
// router so it cannot move - if this IP changes, the puzzle silently
// stops firing.
IPAddress KASA_PLUG_IP(192, 168, 4, 21);

const uint16_t KASA_PORT = 9999;

// The plug has no duration of its own: it switches on once the box has
// finished opening and off again before the box starts closing, so
// OPEN_DURATION_MS above governs how long it stays lit.

// Give up on a plug that is not answering rather than stalling the
// reader. The whole exchange normally takes well under 100ms.
const int KASA_CONNECT_TIMEOUT_MS = 1500;
const unsigned long KASA_READ_TIMEOUT_MS = 1000;

// How many times to ask the plug before giving up on a switch command.
const int KASA_RETRIES = 2;

// How often to retry WiFi in the background after a drop.
const unsigned long WIFI_RETRY_INTERVAL_MS = 30000;

// ---- Brownout mitigation ----
// The servo and the radio are never active at the same moment. The
// reveal runs strictly in sequence:
//
//   open the box -> pause -> plug ON  -> hold
//   plug OFF (confirmed) -> pause -> close the box
//
// This is the quiet gap between each of those steps.
//
// Still a mitigation, not a cure. If it resets even with the two
// separated, the 5V rail is genuinely too weak - give the servo its own
// supply with a 470-1000uF capacitor across it at the servo end.
const unsigned long REVEAL_STAGGER_MS = 600;

// Let the radio settle before the motor is energized.
const unsigned long SERVO_ATTACH_SETTLE_MS = 150;

// Log the angle every N degrees during a sweep. If the board resets
// mid-move, the last angle printed is where it died - which separates
// "dies the instant the motor starts" (inrush / weak supply) from
// "dies at the same angle every time" (the mechanism is binding and the
// servo is stalling against it).
const int SWEEP_LOG_EVERY_DEG = 5;

// Cutting WiFi transmit power lowers the peak the regulator has to
// supply. 11dBm is plenty for a plug in the same room. Set false to
// leave the radio at full power.
const bool LIMIT_WIFI_TX_POWER = true;

// ============================================================
// State
// ============================================================

// The servo runs as a state machine stepped from loop(), not as a
// blocking for-loop. A 5-second sweep done with delay() would freeze
// the reader mid-reveal: taps would be missed and the auto-close timer
// could not run.
enum ServoPhase { SERVO_IDLE, SERVO_OPENING, SERVO_HOLDING, SERVO_CLOSING };

Servo boxServo;
ServoPhase servoPhase = SERVO_IDLE;
int servoAngle = CLOSED_ANGLE;
unsigned long servoLastStepMs = 0;
unsigned long servoOpenedAtMs = 0;
bool servoAttached = false;

// Runtime copies of the sweep speeds, so SWEEP <ms> can change them
// without reflashing. Slower stepping means the motor draws less, which
// is the quickest way to tell a current problem from a logic one.
int openStepDelayMs = OPEN_STEP_DELAY_MS;
int closeStepDelayMs = CLOSE_STEP_DELAY_MS;

bool plugIsOn = false;
unsigned long lastWifiRetryMs = 0;

// ---- Crash breadcrumbs ----
// RTC_NOINIT_ATTR memory survives a reset (but not a power cycle), so
// these record what the prop was doing at the instant it died. An
// intermittent fault is only "weird" until you can see whether it
// always dies in the same phase, at the same angle, or with the plug
// in the same state - then it is usually obvious.
#define BREADCRUMB_MAGIC 0xB0A5EEDu
RTC_NOINIT_ATTR uint32_t rtcMagic;
RTC_NOINIT_ATTR uint32_t rtcResetCount;
RTC_NOINIT_ATTR uint8_t rtcLastPhase;
RTC_NOINIT_ATTR int16_t rtcLastAngle;
RTC_NOINIT_ATTR uint8_t rtcPlugOn;
RTC_NOINIT_ATTR uint32_t rtcUptimeMs;

int progressIndex = 0;
unsigned long lastTapMs = 0;
bool tagPresent = false;
String lastUid = "";
unsigned long lastUidSeenMs = 0;

void setup() {
  Serial.begin(115200);
  delay(300);  // let the port settle so the first lines are not lost

  // Lets ESP32Servo use standard 50Hz servo timing.
  ESP32PWM::allocateTimer(0);
  boxServo.setPeriodHertz(50);
  attachServo();
  boxServo.write(CLOSED_ANGLE);
  servoAngle = CLOSED_ANGLE;
  delay(500);  // give it time to actually get there before we let go
  if (DETACH_WHEN_IDLE) {
    detachServo();
  }

  Serial.println();
  Serial.println("=== RFID Relic Puzzle ===");

#if SIM_MODE
  Serial.println("*** SIM_MODE IS ON - the reader is NOT initialized. ***");
  Serial.println("    Taps come only from the Serial Monitor. Set");
  Serial.println("    SIM_MODE to 0 for the real hardware build.");
#else
  Serial.println("[1/3] Starting SPI...");
  SPI.begin();

  Serial.println("[2/3] Initializing reader...");
  mfrc522.PCD_Init();
  delay(50);

  Serial.println("[3/3] Checking reader...");
  byte version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("      Reader firmware version: 0x");
  Serial.println(version, HEX);
  if (version == 0x00 || version == 0xFF) {
    Serial.println("*** SPI COMMUNICATION FAILURE - reader not responding. ***");
    Serial.println("    Check wiring (SS=5, SCK=18, MOSI=23, MISO=19, RST=22)");
    Serial.println("    and that the module is on 3.3V, NOT 5V.");
  }

  // Must come after PCD_Init, which resets the register bank.
  mfrc522.PCD_SetAntennaGain(MFRC522::RxGain_max);
#endif

  connectWifi(15000);

  // A brownout mid-reveal reboots the board before the close runs, which
  // would otherwise leave the plug switched on for the rest of the night.
  // Put it back to a known state on every boot, and say plainly if the
  // last reset was a brownout rather than a normal power-up.
  esp_reset_reason_t why = esp_reset_reason();
  Serial.print("Last reset reason: ");
  Serial.println(resetReasonName(why));
  reportBreadcrumbs(why);
  if (why == ESP_RST_TASK_WDT || why == ESP_RST_INT_WDT || why == ESP_RST_WDT) {
    Serial.println("    A watchdog reset during servo movement usually means the");
    Serial.println("    supply sagged rather than that code hung. If the reboots");
    Serial.println("    that follow show 'csum err', that is the ROM loader");
    Serial.println("    failing on an out-of-spec rail - which is conclusive.");
  }
  if (why == ESP_RST_BROWNOUT) {
    Serial.println();
    Serial.println("*** LAST RESET WAS A BROWNOUT ***");
    Serial.println("    The 5V rail collapsed. Give the servo its own supply");
    Serial.println("    and put a 470-1000uF cap across it at the servo end.");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Putting the plug into a known state (off)...");
    kasaSetRelay(false);
  }

  // Spell the reveal timing out, so a sweep that is accidentally too
  // fast to see is obvious here rather than on the night.
  {
    int travel = abs(OPEN_ANGLE - CLOSED_ANGLE);
    float openSecs = travel * OPEN_STEP_DELAY_MS / 1000.0;
    float closeSecs = travel * CLOSE_STEP_DELAY_MS / 1000.0;
    Serial.println();
    float stagger = REVEAL_STAGGER_MS / 1000.0;
    float hold = OPEN_DURATION_MS / 1000.0;
    Serial.printf("Reveal: %d degrees of travel (%d -> %d)\n", travel, CLOSED_ANGLE, OPEN_ANGLE);
    Serial.println("  order: open -> pause -> plug ON -> hold -> plug OFF -> pause -> close");
    Serial.printf("  open %.2fs, pause %.1fs, hold %.1fs, pause %.1fs, close %.2fs\n",
                  openSecs, stagger, hold, stagger, closeSecs);
    Serial.printf("  plug lit from %.1fs to %.1fs after the solve\n",
                  openSecs + stagger, openSecs + stagger + hold);
  }

  Serial.println();
  reportMemory();
  Serial.println();
  Serial.println("Ready. Waiting for relics...");
  printBenchHelp();
  printExpected();
}

void loop() {
  unsigned long now = millis();

  // Reset a stalled attempt after too long a gap between taps
  if (progressIndex > 0 && (now - lastTapMs > MAX_GAP_MS)) {
    Serial.println("Too much time passed - resetting attempt.");
    progressIndex = 0;
    printExpected();
  }

  serviceServo();
  serviceWifi(now);
  readTapFromSerial();

#if !SIM_MODE
  bool cardHere = mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial();

  if (cardHere) {
    // One physical tap should count once. A halted tag stops answering
    // REQA until it is lifted and re-presented, which handles most of
    // this, but the UID+time check below covers the rest.
    if (!tagPresent) {
      tagPresent = true;
      handleTap();
    }
  } else {
    tagPresent = false;
  }
#endif
}

// Treat each newline-terminated token typed into the Serial Monitor as
// one relic tap. Case-insensitive; whitespace trimmed. Available in both
// modes so the sequence logic can be tested without tags.
void readTapFromSerial() {
  static String line = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      if (line.length() > 0) {
        line.toUpperCase();
        if (!handleBenchCommand(line)) {
          Serial.print("[TYPED] ");
          Serial.println(line);
          processIdentifier(line);
        }
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 16) line = "";  // runaway guard
    }
  }
}

#if !SIM_MODE
void handleTap() {
  Serial.println();
  Serial.println("--- Tag tapped ---");

  // --- UID ---
  String uid = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) uid += '0';
    uid += String(mfrc522.uid.uidByte[i], HEX);
    if (i < mfrc522.uid.size - 1) uid += ' ';
  }
  uid.toUpperCase();
  Serial.print("  UID:  ");
  Serial.print(uid);
  Serial.print("  (");
  Serial.print(mfrc522.uid.size);
  Serial.println(" bytes)");

  // --- Type ---
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.print("  Type: ");
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

  if (piccType != MFRC522::PICC_TYPE_MIFARE_UL) {
    Serial.println("  *** Not an NTAG/Ultralight tag - this puzzle cannot");
    Serial.println("      read an identifier off it. (A 4-byte UID and");
    Serial.println("      'MIFARE 1KB' is the card bundled with the reader.)");
    mfrc522.PICC_HaltA();
    return;
  }

  // --- Debounce repeat reads of the same physical tag ---
  unsigned long now = millis();
  if (uid == lastUid && (now - lastUidSeenMs) < REMOVE_DEBOUNCE_MS) {
    Serial.println("  (same tag again within debounce window - ignoring)");
    mfrc522.PICC_HaltA();
    return;
  }
  lastUid = uid;
  lastUidSeenMs = now;

  // --- Read the identifier ---
  // MIFARE_Read returns 16 bytes: four consecutive pages starting at
  // READ_PAGE. Showing all four makes a mis-targeted write obvious -
  // if the identifier landed on page 5 you will see it sitting there.
  byte buffer[18];
  byte size = sizeof(buffer);
  MFRC522::StatusCode status = mfrc522.MIFARE_Read(READ_PAGE, buffer, &size);

  if (status != MFRC522::STATUS_OK) {
    Serial.print("  Read failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    Serial.println("  Hold the tag flat and still against the reader.");
    mfrc522.PICC_HaltA();
    return;
  }

  for (byte page = 0; page < 4; page++) {
    Serial.print("  Page ");
    Serial.print(READ_PAGE + page);
    Serial.print(": ");
    printPageAsText(&buffer[page * 4]);
  }

  mfrc522.PICC_HaltA();

  char idChars[5];
  for (byte i = 0; i < 4; i++) {
    idChars[i] = (char)buffer[i];
  }
  idChars[4] = '\0';

  processIdentifier(String(idChars));
}

// Prints a 4-byte page as hex plus its printable-ASCII interpretation.
void printPageAsText(const byte *page) {
  for (byte i = 0; i < 4; i++) {
    if (page[i] < 0x10) Serial.print('0');
    Serial.print(page[i], HEX);
    Serial.print(' ');
  }
  Serial.print(" \"");
  for (byte i = 0; i < 4; i++) {
    Serial.print((page[i] >= 32 && page[i] < 127) ? (char)page[i] : '.');
  }
  Serial.println("\"");
}
#endif

void processIdentifier(const String& identifier) {
  // Show exactly what is being compared, with delimiters, so a stray
  // space or a non-printable byte cannot hide inside the comparison.
  Serial.print("  Identifier read:  \"");
  Serial.print(identifier);
  Serial.println("\"");
  Serial.print("  Expected now:     \"");
  Serial.print(TARGET_SEQUENCE[progressIndex]);
  Serial.println("\"");
  // Second line of defence against one tap being read twice, independent
  // of timing: if this is the relic we just accepted, it is a duplicate
  // read, not a wrong answer. Ignoring it is right - resetting the whole
  // attempt because the reader saw the same tag twice would be maddening
  // for a guest who did nothing wrong.
  //
  // Skipped when the sequence genuinely asks for the same relic twice in
  // a row, since then a second read really is the expected next step.
  if (progressIndex > 0 &&
      identifier == TARGET_SEQUENCE[progressIndex - 1] &&
      TARGET_SEQUENCE[progressIndex] != TARGET_SEQUENCE[progressIndex - 1]) {
    Serial.println("  (duplicate read of the relic just accepted - ignoring)");
    return;
  }

  lastTapMs = millis();

  if (identifier == TARGET_SEQUENCE[progressIndex]) {
    progressIndex++;
    Serial.print("  MATCH - progress: ");
    Serial.print(progressIndex);
    Serial.print(" / ");
    Serial.println(SEQUENCE_LENGTH);

    if (progressIndex == SEQUENCE_LENGTH) {
      Serial.println("*** FULL SEQUENCE CORRECT - OPENING ***");
      // The plug is switched on once the box has finished opening -
      // see serviceServo(). Nothing here talks to the network, so the
      // motor never moves while the radio is transmitting.
      startOpening();
      progressIndex = 0;
      printExpected();
    } else {
      Serial.print("  Next expected relic: ");
      Serial.println(TARGET_SEQUENCE[progressIndex]);
    }
  } else {
    Serial.println("  NO MATCH - resetting attempt.");
    // Byte-level comparison, so an invisible difference (trailing null,
    // wrong length, a space) shows up instead of two identical-looking
    // strings that refuse to match.
    if (identifier.length() != TARGET_SEQUENCE[progressIndex].length()) {
      Serial.print("  (length differs: read ");
      Serial.print(identifier.length());
      Serial.print(", expected ");
      Serial.print(TARGET_SEQUENCE[progressIndex].length());
      Serial.println(")");
    }
    progressIndex = 0;
    printExpected();
  }
}

// Bench commands, checked before anything is treated as a relic
// identifier. Returns true if the line was a command.
//   O        jog the servo to OPEN_ANGLE
//   C        jog the servo to CLOSED_ANGLE
//   A<n>     jog the servo to n degrees, e.g. A45
//   ON/OFF   switch the plug directly
//   WIFI     report WiFi status and try the plug
//   GO       run the whole reveal: open, hold, close, plug off
bool handleBenchCommand(const String &cmd) {
  if (cmd == "ON") {
    kasaSetRelay(true);
    return true;
  }
  if (cmd == "OFF") {
    kasaSetRelay(false);
    return true;
  }
  if (cmd == "TOGGLE") {
    kasaSetRelay(!plugIsOn);
    return true;
  }
  if (cmd == "STATE") {
    kasaReportState();
    return true;
  }
  if (cmd == "PING") {
    kasaPing();
    return true;
  }
  if (cmd == "FIND") {
    kasaDiscover(3000);
    return true;
  }
  if (cmd.startsWith("IP ")) {
    setPlugIpAtRuntime(cmd.substring(3));
    return true;
  }
  if (cmd == "WIFI") {
    reportWifi();
    return true;
  }
  if (cmd.startsWith("SWEEP ")) {
    int ms = cmd.substring(6).toInt();
    if (ms < 1 || ms > 500) {
      Serial.println("SWEEP takes 1-500 ms per degree, e.g. SWEEP 40");
      return true;
    }
    openStepDelayMs = ms;
    closeStepDelayMs = ms;
    Serial.printf("Sweep speed now %d ms/degree (%.2fs per sweep). Not saved.\n",
                  ms, abs(OPEN_ANGLE - CLOSED_ANGLE) * ms / 1000.0);
    return true;
  }
  if (cmd == "MEM") {
    reportMemory();
    return true;
  }
  if (cmd == "HELP" || cmd == "?") {
    printBenchHelp();
    return true;
  }
  if (cmd == "O") {
    jogServo(OPEN_ANGLE);
    return true;
  }
  if (cmd == "C") {
    jogServo(CLOSED_ANGLE);
    return true;
  }
  if (cmd == "GO") {
    Serial.println("Running the full reveal...");
    startOpening();  // the plug follows once the sweep finishes
    return true;
  }
  if (cmd.length() > 1 && cmd[0] == 'A') {
    String number = cmd.substring(1);
    for (unsigned int i = 0; i < number.length(); i++) {
      if (!isDigit(number[i])) {
        return false;  // not A-followed-by-digits, treat as an identifier
      }
    }
    jogServo(number.toInt());
    return true;
  }
  return false;
}

// ============================================================
// Kasa smart plug (legacy local protocol, TCP port 9999)
// ============================================================
//
// The old Kasa firmware speaks a plain JSON command wrapped in an XOR
// autokey cipher - no authentication, no TLS, no cloud account. Each
// message is a 4-byte big-endian length followed by the ciphertext.
//
// Encrypting: start with key 171, XOR it with the plaintext byte; the
// RESULT is both the output byte and the next key. Decrypting runs the
// same chain backwards, using each ciphertext byte as the next key.
//
// Newer Kasa firmware replaced this with KLAP, which needs a handshake
// and account credentials. If the plug stops answering after a firmware
// update, that is why - and it is a good reason to leave this one's
// auto-update off.

// Fills buf with the framed, encrypted message. Returns its length.
size_t kasaEncrypt(const char *json, uint8_t *buf, size_t bufSize) {
  size_t n = strlen(json);
  if (n + 4 > bufSize) {
    return 0;
  }
  buf[0] = (uint8_t)(n >> 24);
  buf[1] = (uint8_t)(n >> 16);
  buf[2] = (uint8_t)(n >> 8);
  buf[3] = (uint8_t)(n);

  uint8_t key = 171;
  for (size_t i = 0; i < n; i++) {
    key = key ^ (uint8_t)json[i];
    buf[4 + i] = key;
  }
  return n + 4;
}

// Sends one command and returns the decrypted reply (empty on failure).
// Blocking, but only briefly and only at solve/close time - never
// during the servo sweep, so the reader is not starved.
String kasaSend(const char *json) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[KASA] WiFi is not connected - cannot reach the plug.");
    return "";
  }

  uint8_t out[192];
  size_t outLen = kasaEncrypt(json, out, sizeof(out));
  if (outLen == 0) {
    Serial.println("[KASA] Command too long for the buffer.");
    return "";
  }

  WiFiClient client;
  if (!client.connect(KASA_PLUG_IP, KASA_PORT, KASA_CONNECT_TIMEOUT_MS)) {
    Serial.print("[KASA] Could not connect to ");
    Serial.print(KASA_PLUG_IP);
    Serial.println(" - wrong IP, plug offline, or on another subnet.");
    return "";
  }

  client.write(out, outLen);
  client.flush();

  // Reply: 4-byte length, then that many ciphertext bytes.
  uint8_t header[4];
  if (!kasaReadBytes(client, header, 4)) {
    Serial.println("[KASA] No reply header - command may still have worked.");
    client.stop();
    return "";
  }

  uint32_t replyLen = ((uint32_t)header[0] << 24) | ((uint32_t)header[1] << 16)
                      | ((uint32_t)header[2] << 8) | header[3];
  if (replyLen == 0 || replyLen > 1024) {
    Serial.printf("[KASA] Reply length %u looks wrong - ignoring.\n", (unsigned)replyLen);
    client.stop();
    return "";
  }

  String reply;
  reply.reserve(replyLen);
  uint8_t key = 171;
  for (uint32_t i = 0; i < replyLen; i++) {
    uint8_t c;
    if (!kasaReadBytes(client, &c, 1)) {
      break;
    }
    reply += (char)(key ^ c);
    key = c;  // autokey: this ciphertext byte decrypts the next one
  }
  client.stop();
  return reply;
}

bool kasaReadBytes(WiFiClient &client, uint8_t *dest, size_t count) {
  unsigned long deadline = millis() + KASA_READ_TIMEOUT_MS;
  size_t got = 0;
  while (got < count && millis() < deadline) {
    if (client.available()) {
      int b = client.read();
      if (b < 0) {
        continue;
      }
      dest[got++] = (uint8_t)b;
    } else if (!client.connected()) {
      break;
    }
  }
  return got == count;
}

bool kasaSetRelay(bool on) {
  const char *cmd = on ? "{\"system\":{\"set_relay_state\":{\"state\":1}}}"
                       : "{\"system\":{\"set_relay_state\":{\"state\":0}}}";
  Serial.print("[KASA] Switching plug ");
  Serial.println(on ? "ON..." : "OFF...");

  String reply = kasaSend(cmd);

  // The plug answers with err_code 0 on success. Checking it means a
  // plug that is reachable but refusing the command gets reported,
  // instead of looking identical to a success.
  if (reply.length() > 0 && reply.indexOf("\"err_code\":0") >= 0) {
    plugIsOn = on;
    Serial.println("[KASA] Plug confirmed.");
    return true;
  }

  if (reply.length() > 0) {
    Serial.print("[KASA] Unexpected reply: ");
    Serial.println(reply);
  }
  Serial.println("[KASA] Plug did NOT confirm the command.");
  return false;
}

// kasaSend() already waits for the plug's reply, so kasaSetRelay()
// returning true means the plug confirmed with err_code 0 - not merely
// that the command was written to the socket. This retries a couple of
// times before giving up, because a single dropped packet should not
// leave the prop half-finished.
//
// If it never confirms, the reveal still continues. A dark box that
// opens beats a box that refuses to open because a plug is unplugged.
bool kasaSetRelayWithRetry(bool on, int attempts) {
  for (int attempt = 1; attempt <= attempts; attempt++) {
    if (kasaSetRelay(on)) {
      return true;
    }
    if (attempt < attempts) {
      Serial.printf("[KASA] Attempt %d of %d failed - retrying...\n", attempt, attempts);
      delay(250);
    }
  }
  Serial.println("[KASA] Never confirmed. Carrying on with the sequence anyway.");
  return false;
}

// Pulls a quoted string value out of the plug's JSON, e.g. "alias".
// Crude on purpose - a real JSON parser is not worth the flash here.
String kasaJsonString(const String &src, const char *key) {
  String needle = String("\"") + key + "\":\"";
  int start = src.indexOf(needle);
  if (start < 0) {
    return "";
  }
  start += needle.length();
  int end = src.indexOf('"', start);
  if (end < 0) {
    return "";
  }
  return src.substring(start, end);
}

// Pulls a numeric value out of the plug's JSON. Returns fallback if the
// key is missing.
int kasaJsonInt(const String &src, const char *key, int fallback) {
  String needle = String("\"") + key + "\":";
  int start = src.indexOf(needle);
  if (start < 0) {
    return fallback;
  }
  start += needle.length();
  return src.substring(start, start + 8).toInt();
}

// Asks the plug what it actually is and what its relay is doing. This
// is the command to reach for first: it proves addressing, the cipher
// and the relay state all in one go, without changing anything.
void kasaReportState() {
  Serial.print("[KASA] Querying ");
  Serial.print(KASA_PLUG_IP);
  Serial.println("...");

  String info = kasaSend("{\"system\":{\"get_sysinfo\":{}}}");
  if (info.length() == 0) {
    Serial.println("[KASA] No answer. Try PING, then FIND to locate it.");
    return;
  }

  String alias = kasaJsonString(info, "alias");
  String model = kasaJsonString(info, "model");
  int relay = kasaJsonInt(info, "relay_state", -1);
  int rssi = kasaJsonInt(info, "rssi", 0);

  Serial.print("  alias: ");
  Serial.println(alias.length() ? alias : String("(none)"));
  Serial.print("  model: ");
  Serial.println(model.length() ? model : String("(unknown)"));
  Serial.print("  relay: ");
  if (relay == 1) {
    Serial.println("ON");
  } else if (relay == 0) {
    Serial.println("OFF");
  } else {
    Serial.println("(not reported - is this a plug, or something else?)");
  }
  Serial.print("  plug's own WiFi RSSI: ");
  Serial.println(rssi);

  // Keep our idea of the plug's state honest: something else (the Kasa
  // app, a wall switch, a power cut) may have changed it behind us.
  if (relay == 0 || relay == 1) {
    bool actual = (relay == 1);
    if (actual != plugIsOn) {
      Serial.print("  NOTE: we thought the plug was ");
      Serial.print(plugIsOn ? "ON" : "OFF");
      Serial.println(" - correcting to match.");
      plugIsOn = actual;
    }
  }
}

// Just opens a TCP connection and drops it. Separates "cannot reach
// this address at all" from "reached it but the exchange went wrong".
void kasaPing() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[KASA] WiFi not connected - nothing to ping from.");
    return;
  }
  Serial.print("[KASA] Opening TCP ");
  Serial.print(KASA_PLUG_IP);
  Serial.print(":");
  Serial.print(KASA_PORT);
  Serial.print("... ");

  unsigned long started = millis();
  WiFiClient client;
  if (client.connect(KASA_PLUG_IP, KASA_PORT, KASA_CONNECT_TIMEOUT_MS)) {
    Serial.print("open in ");
    Serial.print(millis() - started);
    Serial.println("ms - something is listening on 9999.");
    client.stop();
  } else {
    Serial.println("REFUSED / timed out.");
    Serial.println("  Wrong IP, plug unplugged, different subnet, or a");
    Serial.println("  newer firmware that no longer serves port 9999.");
  }
}

// Broadcasts a sysinfo probe and lists every Kasa device that answers.
// This is how you find the plug's IP without digging through the router,
// and how you catch it having moved to a new address.
//
// Discovery goes over UDP, and the UDP form of this protocol has NO
// 4-byte length header - just the ciphertext.
void kasaDiscover(unsigned long waitMs) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[KASA] WiFi not connected - cannot search.");
    return;
  }

  const char *probe = "{\"system\":{\"get_sysinfo\":{}}}";
  size_t n = strlen(probe);
  uint8_t out[128];
  uint8_t key = 171;
  for (size_t i = 0; i < n; i++) {
    key = key ^ (uint8_t)probe[i];
    out[i] = key;
  }

  WiFiUDP udp;
  if (!udp.begin(0)) {  // any free local port
    Serial.println("[KASA] Could not open a UDP socket.");
    return;
  }

  Serial.println("[KASA] Broadcasting discovery probe...");
  udp.beginPacket(IPAddress(255, 255, 255, 255), KASA_PORT);
  udp.write(out, n);
  udp.endPacket();

  int found = 0;
  unsigned long deadline = millis() + waitMs;
  while (millis() < deadline) {
    int size = udp.parsePacket();
    if (size <= 0) {
      delay(10);
      continue;
    }

    IPAddress from = udp.remoteIP();
    // static, not on the stack: a kilobyte of local buffer inside a
    // function called from loop() is a large bite out of the task stack.
    static uint8_t in[1024];
    int len = udp.read(in, min(size, (int)sizeof(in)));

    String reply;
    reply.reserve(len);
    uint8_t k = 171;
    for (int i = 0; i < len; i++) {
      reply += (char)(k ^ in[i]);
      k = in[i];
    }

    found++;
    String alias = kasaJsonString(reply, "alias");
    String model = kasaJsonString(reply, "model");
    int relay = kasaJsonInt(reply, "relay_state", -1);

    Serial.print("  ");
    Serial.print(from);
    Serial.print("  \"");
    Serial.print(alias);
    Serial.print("\"  ");
    Serial.print(model);
    if (relay == 0 || relay == 1) {
      Serial.print(relay == 1 ? "  [ON]" : "  [OFF]");
    }
    if (from == KASA_PLUG_IP) {
      Serial.print("   <-- this is the configured one");
    }
    Serial.println();
  }
  udp.stop();

  if (found == 0) {
    Serial.println("  Nothing answered. Either no legacy Kasa devices are on");
    Serial.println("  this network, or the AP is blocking broadcast traffic");
    Serial.println("  (guest networks and client isolation usually do).");
  } else {
    Serial.print("  ");
    Serial.print(found);
    Serial.println(" device(s) answered. Use 'IP <address>' to retarget.");
  }
}

// Retarget the plug without reflashing - handy while hunting for the
// right address. NOT saved: a reboot goes back to KASA_PLUG_IP, so put
// the address you settle on into the config.
void setPlugIpAtRuntime(const String &text) {
  IPAddress candidate;
  if (!candidate.fromString(text)) {
    Serial.print("[KASA] \"");
    Serial.print(text);
    Serial.println("\" is not a valid IP address.");
    return;
  }
  KASA_PLUG_IP = candidate;
  Serial.print("[KASA] Now targeting ");
  Serial.print(KASA_PLUG_IP);
  Serial.println(" (until reboot - edit KASA_PLUG_IP to make it stick).");
}

// Stack headroom and heap, for chasing corruption-style crashes. The
// high-water mark is the SMALLEST amount of stack that has ever been
// free: if it approaches zero, the loop task is overflowing and the
// crash will surface somewhere unrelated and confusing.
void reportMemory() {
  Serial.println("Memory:");
  Serial.printf("  loop task stack free (low-water): %u bytes\n",
                (unsigned)uxTaskGetStackHighWaterMark(NULL));
  Serial.printf("  free heap: %u, lowest ever: %u\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  Serial.printf("  largest allocatable block: %u\n",
                (unsigned)ESP.getMaxAllocHeap());
}

const char *phaseName(uint8_t phase) {
  switch (phase) {
    case SERVO_IDLE:    return "idle";
    case SERVO_OPENING: return "OPENING";
    case SERVO_HOLDING: return "holding (open)";
    case SERVO_CLOSING: return "CLOSING";
    default:            return "unknown";
  }
}

// Records what we are doing right now, so that if the board dies the
// next boot can say where. Cheap enough to call on every servo step.
void dropBreadcrumb() {
  rtcMagic = BREADCRUMB_MAGIC;
  rtcLastPhase = (uint8_t)servoPhase;
  rtcLastAngle = (int16_t)servoAngle;
  rtcPlugOn = plugIsOn ? 1 : 0;
  rtcUptimeMs = millis();
}

// Prints what the previous run was doing when it stopped, and keeps a
// running count of unexpected resets. A clean power-up clears it.
void reportBreadcrumbs(esp_reset_reason_t why) {
  bool valid = (rtcMagic == BREADCRUMB_MAGIC);
  bool expected = (why == ESP_RST_POWERON || why == ESP_RST_SW || why == ESP_RST_EXT);

  if (!valid) {
    rtcResetCount = 0;
    rtcMagic = BREADCRUMB_MAGIC;
    return;  // first boot after a power cycle - nothing to report
  }

  if (expected) {
    rtcResetCount = 0;
    return;
  }

  rtcResetCount++;
  Serial.println();
  Serial.println("--- What the previous run was doing when it died ---");
  Serial.printf("  phase:  %s\n", phaseName(rtcLastPhase));
  Serial.printf("  angle:  %d\n", (int)rtcLastAngle);
  Serial.printf("  plug:   %s\n", rtcPlugOn ? "ON" : "off");
  Serial.printf("  uptime: %.1fs\n", rtcUptimeMs / 1000.0);
  Serial.printf("  unexpected resets since last power cycle: %u\n",
                (unsigned)rtcResetCount);
  Serial.println("  Same phase and a similar angle every time points at the");
  Serial.println("  mechanism or the rail. Scattered across phases, with the");
  Serial.println("  plug state varying, points at the supply being marginal.");
  Serial.println("---------------------------------------------------");
}

const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power-on (normal)";
    case ESP_RST_SW:       return "software restart";
    case ESP_RST_PANIC:    return "panic / exception";
    case ESP_RST_INT_WDT:  return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK WATCHDOG";
    case ESP_RST_WDT:      return "WATCHDOG (other)";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_EXT:      return "external reset pin";
    default:               return "unknown";
  }
}

void printBenchHelp() {
  Serial.println("Bench commands (line ending: Newline):");
  Serial.println("  Plug:  ON, OFF, TOGGLE, STATE, PING, FIND, IP <addr>");
  Serial.println("  Servo: O (open), C (closed), A<n> (angle), SWEEP <ms> (speed)");
  Serial.println("  Both:  GO (full reveal), WIFI (link status), MEM (stack/heap)");
  Serial.println("  Any other text is treated as a relic identifier.");
}

// A dropped connection would otherwise stay dropped for the rest of the
// night. Retry quietly in the background - WiFi.begin() is non-blocking,
// so this never stalls the reader.
void serviceWifi(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  if (now - lastWifiRetryMs < WIFI_RETRY_INTERVAL_MS) {
    return;
  }
  lastWifiRetryMs = now;
  Serial.println("[WiFi] Disconnected - retrying...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectWifi(unsigned long timeoutMs) {
  Serial.print("Connecting to WiFi \"");
  Serial.print(WIFI_SSID);
  Serial.print("\"...");
  WiFi.mode(WIFI_STA);
  if (LIMIT_WIFI_TX_POWER) {
    WiFi.setTxPower(WIFI_POWER_11dBm);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long deadline = millis() + timeoutMs;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("  Connected. ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("  Plug expected at:   ");
    Serial.println(KASA_PLUG_IP);
    if (WiFi.localIP()[0] != KASA_PLUG_IP[0] || WiFi.localIP()[1] != KASA_PLUG_IP[1]) {
      Serial.println("  WARNING: those look like different networks - the plug");
      Serial.println("  must be on the same LAN and subnet as the ESP32.");
    }
  } else {
    Serial.println("  FAILED. The puzzle still runs; the plug just will not fire.");
    Serial.println("  Check WIFI_SSID / WIFI_PASSWORD. Note the ESP32 needs 2.4GHz.");
  }
}

void reportWifi() {
  Serial.print("WiFi status: ");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("connected, IP ");
    Serial.print(WiFi.localIP());
    Serial.print(", RSSI ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("NOT connected.");
  }
  Serial.print("Querying plug at ");
  Serial.print(KASA_PLUG_IP);
  Serial.println("...");
  String info = kasaSend("{\"system\":{\"get_sysinfo\":{}}}");
  if (info.length() > 0) {
    Serial.println(info);
  }
}

// ============================================================
// Servo
// ============================================================

void attachServo() {
  if (!servoAttached) {
    boxServo.attach(SERVO_PIN, 500, 2400);  // SG90 pulse range, microseconds
    servoAttached = true;
  }
}

void detachServo() {
  if (servoAttached) {
    boxServo.detach();
    servoAttached = false;
  }
}

void startOpening() {
  attachServo();
  delay(SERVO_ATTACH_SETTLE_MS);  // let the attach inrush pass

  // Always start the reveal from closed. Without this, a servo left
  // part-open (by the O / A<n> jog commands while finding angles, or by
  // a guest nudging the lid) sweeps from there - and one already sitting
  // at OPEN_ANGLE does not move at all, which looks exactly like the
  // trigger failing.
  if (servoAngle != CLOSED_ANGLE) {
    Serial.print("  (servo was at ");
    Serial.print(servoAngle);
    Serial.println(" - returning to closed first)");
    boxServo.write(CLOSED_ANGLE);
    servoAngle = CLOSED_ANGLE;
    delay(400);  // let it actually get there before the sweep begins
  }

  Serial.println("Opening...");
  servoPhase = SERVO_OPENING;
  servoLastStepMs = millis();
}

// Called every loop(). Moves the servo at most one degree per call, so
// nothing here ever blocks the reader.
void serviceServo() {
  unsigned long now = millis();

  switch (servoPhase) {
    case SERVO_OPENING:
      if (now - servoLastStepMs < (unsigned long)openStepDelayMs) {
        return;
      }
      servoLastStepMs = now;
      if (servoAngle < OPEN_ANGLE) {
        servoAngle++;
        boxServo.write(servoAngle);
        dropBreadcrumb();
        logSweepAngle();
      } else {
        Serial.println("Open.");
        // Motor first, then the radio - and never together. Cutting the
        // servo signal before transmitting means the two peak loads
        // cannot land on the rail at the same moment.
        if (DETACH_WHEN_IDLE) {
          detachServo();
        }
        delay(REVEAL_STAGGER_MS);
        dropBreadcrumb();
        kasaSetRelayWithRetry(true, KASA_RETRIES);
        servoPhase = SERVO_HOLDING;
        dropBreadcrumb();
        // Fresh reading: the plug exchange above took real time, and the
        // hold should count from when the light actually came on.
        servoOpenedAtMs = millis();
      }
      break;

    case SERVO_HOLDING:
      if (now - servoOpenedAtMs >= OPEN_DURATION_MS) {
        // Closing reverses the order: the plug goes off and CONFIRMS
        // before the servo is energized at all.
        Serial.println("Hold finished - switching the plug off first.");
        dropBreadcrumb();
        kasaSetRelayWithRetry(false, KASA_RETRIES);
        delay(REVEAL_STAGGER_MS);
        Serial.println("Closing...");
        attachServo();
        delay(SERVO_ATTACH_SETTLE_MS);
        servoPhase = SERVO_CLOSING;
        servoLastStepMs = millis();
      }
      break;

    case SERVO_CLOSING:
      if (now - servoLastStepMs < (unsigned long)closeStepDelayMs) {
        return;
      }
      servoLastStepMs = now;
      if (servoAngle > CLOSED_ANGLE) {
        servoAngle--;
        boxServo.write(servoAngle);
        dropBreadcrumb();
        logSweepAngle();
      } else {
        // The plug was already switched off before this sweep started.
        Serial.println("Closed. Ready for the next attempt.");
        servoPhase = SERVO_IDLE;
        if (DETACH_WHEN_IDLE) {
          detachServo();
        }
      }
      break;

    case SERVO_IDLE:
    default:
      break;
  }
}

// Prints the angle at intervals during a sweep, and always flushes, so
// the line survives a reset that happens microseconds later.
void logSweepAngle() {
  if (SWEEP_LOG_EVERY_DEG <= 0) {
    return;
  }
  if (servoAngle % SWEEP_LOG_EVERY_DEG != 0) {
    return;
  }
  Serial.print("  at ");
  Serial.println(servoAngle);
  Serial.flush();
}

// Jog the servo straight to an angle, for finding OPEN_ANGLE and
// CLOSED_ANGLE with the mechanism mounted.
void jogServo(int angle) {
  angle = constrain(angle, 0, 180);
  attachServo();
  servoPhase = SERVO_IDLE;
  boxServo.write(angle);
  servoAngle = angle;
  Serial.print("Servo jogged to ");
  Serial.println(angle);
}

void printExpected() {
  Serial.print("Next expected relic: ");
  Serial.println(TARGET_SEQUENCE[0]);
}
