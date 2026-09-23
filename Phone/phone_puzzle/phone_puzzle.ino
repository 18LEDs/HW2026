/*
  Children of the Pier - Phone / DTMF Puzzle - ESP32 Firmware
  ----------------------------------------------------------------------
  Reads digits dialed on the gutted push-button phone via the MT8870
  DTMF decoder module. When the correct number has been dialed, plays
  a "correct" audio clip through the DFPlayer Mini -> PAM8403 amp ->
  phone's earpiece speaker. Wrong numbers get a randomly chosen
  "wrong number" clip instead.

  Library needed: "DFRobotDFPlayerMini" by DFRobot (search that exact
  name in Arduino IDE > Manage Libraries).

  ----------------------------------------------------------------------
  IMPORTANT - the module in this build is a CLONE
  ----------------------------------------------------------------------
  Board silkscreen: HW-247A.  Audio chip: TD5580A (not the genuine
  YX5200). It speaks the same serial frame format as a real DFPlayer
  (0x7E start, 0xEF end, same checksum), but it answers the library's
  RESET command with an undocumented frame the library cannot parse.
  That makes dfPlayer.begin() report failure on a module that is
  actually working fine. See setup() for how this is worked around.

  MicroSD card setup:
    Format as FAT32. Create a folder named exactly "mp3" in the ROOT
    of the card, and put the clips inside it with 4-digit names:
      /mp3/0001.mp3   <- the "correct number" message
      /mp3/0002.mp3   <- wrong-number variant 1
      /mp3/0003.mp3   <- wrong-number variant 2
      /mp3/0004.mp3   <- wrong-number variant 3

    This layout matters. The plain play(n) command plays the n-th file
    in the order files were WRITTEN to the card, which is not the same
    as the file's name - hidden junk like System Volume Information
    silently shifts every index. playMp3Folder(n) addresses the
    /mp3/000n.mp3 file by name instead, so it is the reliable option.
    Delete any hidden files from the card before use.

  Wiring - MT8870 (DTMF decoder):
    Audio input <- tapped from the phone's tone line BEFORE the point
                   where the DFPlayer's audio is injected, otherwise
                   playback feeds back into the decoder
    Q1 (D0) -> GPIO 32
    Q2 (D1) -> GPIO 33
    Q3 (D2) -> GPIO 25
    Q4 (D3) -> GPIO 26
    STD     -> GPIO 27   (goes HIGH briefly when a valid digit is decoded)
    VCC -> 3.3V or 5V per your module's spec (check the board silkscreen)
    GND -> GND

  Wiring - DFPlayer Mini / HW-247A (UART):
    DFPlayer TX (pin 3)  -> ESP32 GPIO 16 (RX2)
    DFPlayer RX (pin 2)  -> ESP32 GPIO 17 (TX2), through a ~1k resistor
                            (standard practice - also suppresses noise
                            that can corrupt the serial link)
    DFPlayer VCC (pin 1) -> 5V, ideally the same dedicated supply as the
                            ESP32, with a 1000uF cap across VCC/GND right
                            at the module. These boards brown themselves
                            out on a weak 5V rail.
    DFPlayer GND (pin 7) -> GND

  Wiring - audio out (NOTE: use the DAC pins, not SPK):
    DFPlayer DAC_R (pin 4) -> PAM8403 input (one channel)
    DFPlayer GND   (pin 7) -> PAM8403 input GND
    PAM8403 output         -> phone's earpiece speaker

    SPK_1 (pin 8) and SPK_2 (pin 6) are the module's own onboard 3W
    amplifier outputs - a differential pair with no ground reference.
    Feeding those into another amplifier's input gives distortion or
    silence. They are only for driving a small 4-8 ohm speaker directly.
    (Handy bench test: clip a bare speaker across SPK_1/SPK_2. If that
    plays, the ESP32, UART and SD card are all fine.)
*/

#include "DFRobotDFPlayerMini.h"

// Uses the ESP32's built-in Serial2 hardware UART to talk to the
// DFPlayer Mini - no extra serial library needed.
//
// Caveat: on ESP32-WROVER modules (the ones with PSRAM) GPIO16 and
// GPIO17 are wired to the PSRAM chip and are NOT usable as UART pins.
// If this is a WROVER, move to GPIO 18/19 or 21/22 and update both
// the constants below and the wiring.
const int DFPLAYER_RX_PIN = 16;  // ESP32 receives on this pin <- DFPlayer TX
const int DFPLAYER_TX_PIN = 17;  // ESP32 transmits on this pin -> DFPlayer RX

HardwareSerial dfPlayerSerial(2);
DFRobotDFPlayerMini dfPlayer;

// ============================================================
// CONFIG
// ============================================================

// Set to 1 ONLY for testing in the Wokwi simulator, which has no
// MT8870 DTMF decoder and no DFPlayer Mini. In SIM_MODE:
//   - "dialed" digits are typed into the Serial Monitor instead of
//     being decoded from the MT8870 pins
//   - audio playback is printed to Serial instead of sent to the
//     DFPlayer
// Leave this 0 for the real hardware build.
#define SIM_MODE 0

const int MT8870_D0 = 32;
const int MT8870_D1 = 33;
const int MT8870_D2 = 25;
const int MT8870_D3 = 26;
const int MT8870_STD = 27;

// The number guests need to dial. Digits only.
const String TARGET_NUMBER = "8675309";

// How long to wait with no new digit before treating dialing as
// finished and checking the attempt.
const unsigned long DIAL_TIMEOUT_MS = 3000;

// Track numbers - these are the /mp3/000N.mp3 file numbers.
const int CORRECT_TRACK = 1;
const int WRONG_TRACKS[] = {2, 3, 4};
const int WRONG_TRACK_COUNT = 3;

const int DFPLAYER_VOLUME = 18;  // 0-30. Start modest, earpiece is close to the ear.

// The TD5580A is noticeably slower than a genuine YX5200 and will drop
// commands sent too close together. Space them out.
const unsigned long DFPLAYER_CMD_GAP_MS = 500;

// Fastest a human can physically dial. Anything arriving quicker than
// this is electrical noise on the STD line, not a real keypress -
// without this, noise floods handleDigit(), the buffer hits full length
// over and over, and each new play command cuts off the clip that just
// started. That sounds like pops and clicks, not audio.
const unsigned long MIN_DIGIT_GAP_MS = 60;

// Once a clip is playing, refuse to start another for this long. Guests
// mashing the keypad would otherwise machine-gun the module the same way.
const unsigned long MIN_RETRIGGER_MS = 1500;

// ============================================================
// State
// ============================================================

String dialedDigits = "";
unsigned long lastDigitMs = 0;
bool waitingToCheck = false;
bool lastStd = false;
char pendingPlayMode = 0;  // 't' = /mp3 folder by name, 'p' = physical index
unsigned long lastPlayMs = 0;
bool hasPlayed = false;
unsigned long noiseRejectCount = 0;

#if !SIM_MODE
// ------------------------------------------------------------
// Raw frame helpers
//
// The library cannot express what this clone needs at startup, so the
// init handshake is done by hand. Frame layout is:
//   [0]=0x7E [1]=0xFF [2]=0x06 [3]=cmd [4]=ACK [5..6]=param
//   [7..8]=checksum [9]=0xEF,  checksum = -(sum of bytes 1..6)
// ------------------------------------------------------------

void sendRawCommand(uint8_t cmd, uint16_t param) {
  uint8_t frame[10] = {0x7E, 0xFF, 0x06, cmd, 0x00,
                       (uint8_t)(param >> 8), (uint8_t)(param & 0xFF),
                       0x00, 0x00, 0xEF};
  uint16_t sum = 0;
  for (int i = 1; i < 7; i++) {
    sum += frame[i];
  }
  sum = -sum;
  frame[7] = (sum >> 8) & 0xFF;
  frame[8] = sum & 0xFF;
  dfPlayerSerial.write(frame, 10);
}

// Collects one complete 10-byte frame. Returns false on timeout.
bool readFrame(uint8_t *out, unsigned long timeoutMs) {
  unsigned long started = millis();
  int idx = 0;
  while (millis() - started < timeoutMs) {
    while (dfPlayerSerial.available()) {
      uint8_t b = dfPlayerSerial.read();
      if (idx == 0) {
        if (b == 0x7E) out[idx++] = b;
      } else {
        out[idx++] = b;
        if (idx == 10) {
          if (out[9] == 0xEF) return true;
          idx = 0;  // resync on a malformed frame
        }
      }
    }
  }
  return false;
}

// Sends a query and returns the parameter from the reply whose COMMAND
// BYTE MATCHES, discarding anything else that arrives first.
//
// This matters more than it sounds. The TD5580A emits unsolicited 0x40
// error frames in bursts, and the library's readers just take the next
// frame off the wire whatever it is. That is where "volume: 126" came
// from - 126 is 0x7E, the frame header byte, read out of an error frame
// as if it were the volume. Matching on the command byte makes every
// reading trustworthy. Returns -1 on timeout.
long queryRaw(uint8_t cmd, unsigned long timeoutMs) {
  while (dfPlayerSerial.available()) {
    dfPlayerSerial.read();
  }
  sendRawCommand(cmd, 0);

  uint8_t frame[10];
  unsigned long deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    if (!readFrame(frame, deadline - millis())) {
      break;
    }
    uint16_t param = ((uint16_t)frame[5] << 8) | frame[6];
    if (frame[3] == cmd) {
      return param;
    }
    if (frame[3] == 0x40) {
      Serial.print("    (discarded error frame: ");
      printErrorCode(param);
    }
  }
  return -1;
}

// Error codes from the TD5580A manual's 0x40 table.
void printErrorCode(uint16_t code) {
  switch (code) {
    case 0x01: Serial.println("busy - still initializing the file system"); break;
    case 0x02: Serial.println("in sleep mode"); break;
    case 0x03: Serial.println("incomplete frame received"); break;
    case 0x04: Serial.println("checksum error"); break;
    case 0x05: Serial.println("file number out of range"); break;
    case 0x06: Serial.println("SPECIFIED FILE NOT FOUND"); break;
    case 0x07: Serial.println("current state does not accept interruption"); break;
    default:
      Serial.print("undocumented code 0x");
      Serial.print(code, HEX);
      Serial.println(" (known clone quirk)");
      break;
  }
}

// Listens for the 0x3F device-online frame for up to waitMs.
// Returns: 1 = card online, 0 = init frame arrived but no card,
//          -1 = no init frame at all.
int listenForInitFrame(unsigned long waitMs) {
  uint8_t frame[10];
  unsigned long deadline = millis() + waitMs;
  while (millis() < deadline) {
    if (!readFrame(frame, deadline - millis())) {
      break;
    }
    uint16_t param = ((uint16_t)frame[5] << 8) | frame[6];

    if (frame[3] == 0x3F) {  // initialization complete, devices online
      Serial.print("  module reports devices online: 0x");
      Serial.println(param, HEX);
      if (param & 0x02) {
        Serial.println("  -> TF card detected and mounted.");
        return 1;
      }
      Serial.println("  -> NO TF CARD in that list.");
      return 0;
    }

    if (frame[3] == 0x40) {
      Serial.print("  module error during init: ");
      printErrorCode(param);
    }
  }
  return -1;
}

// Resets the module and waits for it to report which storage devices it
// found. The datasheet is explicit about two things the old init got
// wrong: file-system init "usually 1.5~3S", and sending commands during
// that window breaks initialization outright. So rather than guessing at
// a delay, wait for the module's own 0x3F device-online frame - which
// doubles as a definitive answer to "did it see the SD card?".
bool resetAndWaitForCard() {
  // PHASE 1: listen only. On a cold boot the ESP32 is up in ~0.5s while
  // the module is still inside its own 1.5-3s power-on init, and the
  // datasheet says a command sent during that window DISRUPTS the init.
  // So say nothing and let the module announce itself.
  Serial.println("  listening for the module's own power-on init frame...");
  int result = listenForInitFrame(5000);
  if (result >= 0) {
    return result == 1;
  }

  // PHASE 2: nothing heard. Either the module booted long before we did
  // (ESP32 reflashed while the module stayed powered - its init frame is
  // long gone) or it is not talking at all. A reset is safe now: if the
  // module is idle it re-scans the card and re-announces; if it is absent
  // or dead, nothing changes.
  Serial.println("  silent - sending reset (0x0C) and listening again...");
  while (dfPlayerSerial.available()) {
    dfPlayerSerial.read();
  }
  sendRawCommand(0x0C, 0);
  result = listenForInitFrame(6000);
  if (result >= 0) {
    return result == 1;
  }

  Serial.println("  module never sent an init frame. It is unpowered,");
  Serial.println("  miswired (TX/RX not crossed?), or the ESP32's UART");
  Serial.println("  pins are bad - type u (with a 16-17 jumper) to test.");
  return false;
}
#endif

void setup() {
  Serial.begin(115200);

  // INPUT_PULLDOWN, not plain INPUT: with the MT8870 unplugged these pins
  // float and STD will trigger phantom digits from noise alone. The
  // MT8870 drives them push-pull, so the pulldowns do not fight it.
  pinMode(MT8870_D0, INPUT_PULLDOWN);
  pinMode(MT8870_D1, INPUT_PULLDOWN);
  pinMode(MT8870_D2, INPUT_PULLDOWN);
  pinMode(MT8870_D3, INPUT_PULLDOWN);
  pinMode(MT8870_STD, INPUT_PULLDOWN);

  randomSeed(esp_random());  // for picking a random wrong-number clip

#if SIM_MODE
  Serial.println("SIM_MODE on - type the dialed number into this Serial Monitor.");
#else
  dfPlayerSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);

  Serial.println("Initializing DFPlayer (HW-247A / TD5580A)...");

  // Bind the library to the port without letting it talk yet.
  //   isACK=false   - with ACK on, every command blocks in sendStack()
  //                   waiting for a 0x41 this chip does not reliably send.
  //   doReset=false - the library's reset handling cannot parse this
  //                   clone's reply. We do the reset by hand below.
  dfPlayer.begin(dfPlayerSerial, /*isACK=*/false, /*doReset=*/false);

  // Reset and wait for the module to actually mount the card. Skipping
  // the reset entirely (as we did before) left the module never having
  // scanned the SD card, which is why every file query returned -1.
  bool cardOnline = resetAndWaitForCard();

  if (!cardOnline) {
    Serial.println("*** SD CARD NOT MOUNTED - no audio is possible. ***");
    Serial.println("    Reseat the card. Must be FAT32 (or FAT16/exFAT),");
    Serial.println("    64GB or smaller, with /mp3/0001.mp3 etc on it.");
  }

  dfPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  delay(DFPLAYER_CMD_GAP_MS);  // datasheet: >=200ms after selecting a device

  dfPlayer.volume(DFPLAYER_VOLUME);
  delay(DFPLAYER_CMD_GAP_MS);

  Serial.println("DFPlayer configured (no ACK, no reset - clone workaround).");
  Serial.println("Bench test: t1..t9 = play /mp3/000N.mp3 by name,");
  Serial.println("p1..p9 = play Nth file by physical order (folder-independent),");
  Serial.println("d = interrogate module, r = raw probe,");
  Serial.println("l = loop forever (for multimeter tests), s = stop,");
  Serial.println("u = UART loopback self-test (jumper GPIO16 to GPIO17 first).");

#endif

  Serial.println("Ready. Waiting for a call...");
}

void loop() {
#if SIM_MODE
  // Digits are typed into the Serial Monitor. Non-digit characters
  // (spaces, newlines) are ignored; the dial timeout still does the
  // "done dialing" detection just like the real thing.
  while (Serial.available()) {
    char c = Serial.read();
    if ((c >= '0' && c <= '9') || c == '*' || c == '#') {
      handleDigit(c);
    }
  }
#else
  bool stdNow = digitalRead(MT8870_STD) == HIGH;

  // STD pulses HIGH when a new digit has been decoded and is stable.
  // Trigger on the rising edge only, so one dialed digit = one read.
  if (stdNow && !lastStd) {
    handleDigit(readDigit());
  }
  lastStd = stdNow;

  handleSerialTestCommands();
  reportDFPlayerMessages();
#endif

  // After a pause with no new digits, check what was dialed. This is
  // the fallback for short / incomplete entries - a full-length number
  // is checked immediately in handleDigit() without waiting this out.
  if (waitingToCheck && (millis() - lastDigitMs > DIAL_TIMEOUT_MS)) {
    finishDialing();
  }
}

#if !SIM_MODE
// Type "u": UART loopback self-test. Two modules failing identically
// makes the COMMON side suspect - the ESP32's own UART pins. Disconnect
// the DFPlayer and jumper GPIO16 directly to GPIO17; everything the
// ESP32 transmits should then arrive back on its own receive pin.
// A pass proves GPIO16/17 work and moves the fault back to module
// wiring/power. A fail (with the jumper in place) means these pins are
// unusable on this board - a WROVER symptom - so move to GPIO 18/19.
void uartLoopbackTest() {
  Serial.println("--- UART loopback self-test ---");
  Serial.println("  Requires: DFPlayer disconnected, jumper GPIO16 <-> GPIO17.");

  while (dfPlayerSerial.available()) {
    dfPlayerSerial.read();
  }

  const uint8_t pattern[4] = {0x55, 0xAA, 0x7E, 0xEF};
  dfPlayerSerial.write(pattern, 4);
  dfPlayerSerial.flush();
  delay(50);

  int got = 0;
  bool match = true;
  while (dfPlayerSerial.available() && got < 4) {
    uint8_t b = dfPlayerSerial.read();
    if (b != pattern[got]) match = false;
    got++;
  }

  if (got == 4 && match) {
    Serial.println("  PASS: GPIO16/17 send and receive correctly.");
    Serial.println("  The ESP32 is fine - the fault is module wiring or power.");
  } else if (got == 0) {
    Serial.println("  FAIL: nothing received. If the jumper really is in");
    Serial.println("  place, GPIO16/17 are not usable on this board (WROVER");
    Serial.println("  boards tie them to PSRAM). Move to GPIO 18/19 and");
    Serial.println("  update DFPLAYER_RX_PIN / DFPLAYER_TX_PIN.");
  } else {
    Serial.print("  PARTIAL: ");
    Serial.print(got);
    Serial.println(" of 4 bytes, or wrong values - noisy or marginal pins.");
  }
  Serial.println("-------------------------------");
}

// Sends a play command and then listens for whatever the module says
// about it. A silent failure and a "file not found" failure look
// identical from the outside otherwise.
void playAndReport(uint8_t cmd, uint16_t param) {
  while (dfPlayerSerial.available()) {
    dfPlayerSerial.read();
  }
  sendRawCommand(cmd, param);
  hasPlayed = true;
  lastPlayMs = millis();

  uint8_t frame[10];
  unsigned long deadline = millis() + 1200;
  bool anyReply = false;
  while (millis() < deadline) {
    if (!readFrame(frame, deadline - millis())) {
      break;
    }
    anyReply = true;
    uint16_t p = ((uint16_t)frame[5] << 8) | frame[6];
    Serial.print("    reply cmd 0x");
    Serial.print(frame[3], HEX);
    Serial.print(" param ");
    Serial.print(p);
    Serial.print(" -> ");
    if (frame[3] == 0x40) {
      Serial.print("ERROR: ");
      printErrorCode(p);
    } else {
      Serial.println("(non-error frame)");
    }
  }
  if (!anyReply) {
    Serial.println("    module said nothing at all about that command.");
  }
}

// Lets you prove out the audio chain on the bench without a working
// DTMF front end: type "t2" to play /mp3/0002.mp3.
void handleSerialTestCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 't' || c == 'T') {
      pendingPlayMode = 't';
    } else if (c == 'p' || c == 'P') {
      pendingPlayMode = 'p';
    } else if (pendingPlayMode && c >= '1' && c <= '9') {
      int n = c - '0';
      if (pendingPlayMode == 't') {
        // 0x12 - addresses /mp3/000N.mp3 BY NAME
        Serial.print("Test: /mp3/000");
        Serial.print(n);
        Serial.println(".mp3 via folder command (0x12)");
        playAndReport(0x12, n);
      } else {
        // 0x03 - addresses the Nth file in the card's PHYSICAL write
        // order, ignoring names and folders entirely. If this plays and
        // the 0x12 version does not, the fault is the /mp3 folder layout,
        // not the audio files or the hardware.
        Serial.print("Test: physical track ");
        Serial.print(n);
        Serial.println(" via plain play (0x03)");
        playAndReport(0x03, n);
      }
      pendingPlayMode = 0;
    } else if (c == 'l' || c == 'L') {
      // Continuous playback, so a multimeter has time to settle while you
      // probe BUSY / SPK / DAC. 0x11 param 1 = loop all tracks.
      pendingPlayMode = 0;
      Serial.println("LOOP: playing all tracks continuously. Type s to stop.");
      playAndReport(0x11, 1);
    } else if (c == 's' || c == 'S') {
      pendingPlayMode = 0;
      Serial.println("STOP.");
      sendRawCommand(0x16, 0);  // 0x16 = stop all playback
    } else if (c == 'r' || c == 'R') {
      pendingPlayMode = 0;
      rawProbe();
    } else if (c == 'u' || c == 'U') {
      pendingPlayMode = 0;
      uartLoopbackTest();
    } else if (c == 'd' || c == 'D') {
      pendingPlayMode = 0;
      runDiagnostics();
    } else if (c != '\r' && c != '\n') {
      pendingPlayMode = 0;
    }
  }
}

// Type "d" to interrogate the module. Every reading here is frame-matched
// via queryRaw(), so a number you see is genuinely the answer to the
// question asked - not whatever unsolicited frame happened to arrive.

void printQueryResult(long value) {
  if (value < 0) Serial.println("no reply");
  else Serial.println(value);
}

void runDiagnostics() {
  Serial.println("--- DFPlayer diagnostics ---");

  long status = queryRaw(0x42, 1500);
  Serial.print("  status:         ");
  if (status < 0) {
    Serial.println("no reply");
  } else {
    // High byte is the device, low byte the transport state.
    uint8_t device = (status >> 8) & 0xFF;
    uint8_t play = status & 0xFF;
    Serial.print("device ");
    Serial.print(device == 2 ? "TF card" : device == 1 ? "USB" : "other");
    Serial.print(", ");
    Serial.println(play == 1 ? "PLAYING" : play == 2 ? "paused" : "stopped");
  }

  Serial.print("  volume:         "); printQueryResult(queryRaw(0x43, 1500));
  Serial.print("  files on card:  "); printQueryResult(queryRaw(0x48, 1500));
  Serial.print("  current track:  "); printQueryResult(queryRaw(0x4C, 1500));
  Serial.print("  software ver:   "); printQueryResult(queryRaw(0x46, 1500));
  Serial.println("---------------------------");
}

// Type "r": bypass the library entirely, send a raw "query volume" frame
// and hex-dump whatever comes back. This is the one test that cannot be
// confused by library quirks - either bytes arrive on GPIO16 or they do
// not, which tells you whether the module is alive and wired at all.
void rawProbe() {
  Serial.println("--- raw UART probe ---");

  while (dfPlayerSerial.available()) {
    dfPlayerSerial.read();
  }

  sendRawCommand(0x48, 0);  // 0x48 = query total file count on the TF card
  Serial.println("  TX: query TF card file count (0x48)");

  uint8_t frame[10];
  int frames = 0;
  unsigned long deadline = millis() + 1500;
  while (millis() < deadline && readFrame(frame, deadline - millis())) {
    frames++;
    uint16_t param = ((uint16_t)frame[5] << 8) | frame[6];
    Serial.print("  RX cmd 0x");
    Serial.print(frame[3], HEX);
    Serial.print(" param ");
    Serial.print(param);
    Serial.print("  -> ");
    switch (frame[3]) {
      case 0x48:
        Serial.print("TF card holds ");
        Serial.print(param);
        Serial.println(" files.");
        break;
      case 0x43: Serial.print("volume is "); Serial.println(param); break;
      case 0x3F: Serial.print("devices online bitmask 0x"); Serial.println(param, HEX); break;
      case 0x3A: Serial.println("device inserted"); break;
      case 0x3B: Serial.println("device removed"); break;
      case 0x40: Serial.print("ERROR: "); printErrorCode(param); break;
      default:   Serial.println("(unhandled)"); break;
    }
  }

  if (frames == 0) {
    Serial.println("  NOTHING came back. The module is not transmitting:");
    Serial.println("    - check 5V and GND actually present AT the module pins");
    Serial.println("    - check module TX (pin 3) -> ESP32 GPIO16 continuity");
    Serial.println("    - or the module is dead: try the spare from the 2-pack");
  }
  Serial.println("----------------------");
}

// The module reports its own problems (no SD card, file not found,
// track finished) as serial frames. The old code never read them, so
// failures were invisible. This prints anything that comes back.
void reportDFPlayerMessages() {
  if (!dfPlayer.available()) {
    return;
  }

  uint8_t type = dfPlayer.readType();
  int value = dfPlayer.read();

  switch (type) {
    case DFPlayerPlayFinished:
      Serial.print("[DFPlayer] finished track ");
      Serial.println(value);
      break;
    case DFPlayerCardInserted:
      Serial.println("[DFPlayer] SD card inserted.");
      break;
    case DFPlayerCardRemoved:
      Serial.println("[DFPlayer] SD card REMOVED.");
      break;
    case DFPlayerCardOnline:
      Serial.println("[DFPlayer] SD card online.");
      break;
    case DFPlayerError:
      Serial.print("[DFPlayer] ERROR: ");
      switch (value) {
        case Busy:             Serial.println("card not found / module busy"); break;
        case Sleeping:         Serial.println("module is sleeping"); break;
        case SerialWrongStack: Serial.println("bad serial frame"); break;
        case CheckSumNotMatch: Serial.println("checksum mismatch"); break;
        case FileIndexOut:     Serial.println("track number out of range - is the file there?"); break;
        case FileMismatch:     Serial.println("file not found - check /mp3/000N.mp3 naming"); break;
        case Advertise:        Serial.println("in advertise mode"); break;
        default:               Serial.println(value); break;
      }
      break;
    case TimeOut:
      // Expected on this clone - it does not answer every query.
      break;
    case WrongStack:
      // Also expected: the TD5580A emits frames the library can't parse.
      break;
    default:
      break;
  }
}
#endif

void handleDigit(char digit) {
  unsigned long now = millis();

  // Noise rejection: a real dialer cannot produce digits this fast.
  // Report the count periodically rather than per-event, so a noisy STD
  // line is obvious in the log without drowning out everything else.
  if (dialedDigits.length() > 0 && (now - lastDigitMs) < MIN_DIGIT_GAP_MS) {
    noiseRejectCount++;
    if (noiseRejectCount % 50 == 1) {
      Serial.print("[NOISE] rejected ");
      Serial.print(noiseRejectCount);
      Serial.println(" impossibly-fast digits - check the MT8870 STD line.");
    }
    return;
  }

  Serial.print("Digit dialed: ");
  Serial.println(digit);

  dialedDigits += digit;
  lastDigitMs = millis();
  waitingToCheck = true;

  Serial.print("Number so far: ");
  Serial.println(dialedDigits);

  // Once a full-length number has been entered, check it right away
  // instead of waiting out the dial timeout.
  if (dialedDigits.length() >= TARGET_NUMBER.length()) {
    finishDialing();
  }
}

void finishDialing() {
  checkNumber();
  waitingToCheck = false;
}

char readDigit() {
  int d0 = digitalRead(MT8870_D0);
  int d1 = digitalRead(MT8870_D1);
  int d2 = digitalRead(MT8870_D2);
  int d3 = digitalRead(MT8870_D3);

  int value = d0 | (d1 << 1) | (d2 << 2) | (d3 << 3);

  // MT8870 BCD output mapping: 1-9 map directly, 0 is sent as 10 (0x0A).
  // *, #, and A-D use the remaining codes, included here for
  // completeness even though your target number likely only uses 0-9.
  switch (value) {
    case 1: return '1';
    case 2: return '2';
    case 3: return '3';
    case 4: return '4';
    case 5: return '5';
    case 6: return '6';
    case 7: return '7';
    case 8: return '8';
    case 9: return '9';
    case 10: return '0';
    case 11: return '*';
    case 12: return '#';
    default: return '?';
  }
}

void checkNumber() {
  Serial.print("Checking dialed number: ");
  Serial.println(dialedDigits);

  if (dialedDigits == TARGET_NUMBER) {
    Serial.println("*** CORRECT NUMBER ***");
    playTrack(CORRECT_TRACK);
  } else {
    int pick = WRONG_TRACKS[random(0, WRONG_TRACK_COUNT)];
    Serial.print("Wrong number - playing clip ");
    Serial.println(pick);
    playTrack(pick);
  }

  dialedDigits = "";
}

void playTrack(int track) {
#if SIM_MODE
  Serial.print("[AUDIO] would play track ");
  Serial.println(track);
#else
  // Don't stomp a clip that only just started. Back-to-back play
  // commands on this chip produce a stutter of pops rather than audio.
  if (hasPlayed && (millis() - lastPlayMs < MIN_RETRIGGER_MS)) {
    Serial.println("(retrigger ignored - a clip just started)");
    return;
  }
  hasPlayed = true;
  lastPlayMs = millis();

  // playMp3Folder() addresses /mp3/000N.mp3 by NAME. Do not switch this
  // back to play(N) - that indexes by write order on the card, which is
  // how you end up hearing the wrong clip or nothing at all.
  dfPlayer.playMp3Folder(track);
#endif
}
