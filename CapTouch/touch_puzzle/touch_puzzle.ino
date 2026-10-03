/*
  Children of the Pier - Capacitive Touch Sequence Puzzle - ESP32 Firmware
  ----------------------------------------------------------------------
  A set of touch pads, some of which must be touched in the correct
  order. Touch the right pads in the right order and a "solved" message
  is sent over ESP-NOW to the haunted clock (haunted_clock.ino).

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
  LINK TO THE HAUNTED CLOCK (ESP-NOW)
  ----------------------------------------------------------------------
  ESP-NOW sends short messages straight to the clock's ESP32 - no
  router or WiFi network. CLOCK_MAC must be the MAC the clock prints at
  startup, and ESPNOW_CHANNEL and the message format must match
  haunted_clock.ino exactly.

  The clock acknowledges every message, so this sketch knows whether
  it arrived. A message that isn't acknowledged is retried a few times,
  then reported as failed. The clock ignores retried copies of a solve
  it has already acted on.

  ----------------------------------------------------------------------
  WIRING
  ----------------------------------------------------------------------
    Each pad -> its GPIO in PADS[] (one wire, no resistor, no ground)
    Keep pad wires short and apart from each other.

  ----------------------------------------------------------------------
  SERIAL MONITOR (115200 baud, line ending: Newline)
  ----------------------------------------------------------------------
    0-9  simulate touching that pad number - test the logic and the
         clock link without touching anything
    p    ping the clock - tests the link without solving
    v    toggle a live view of every pad's reading and drop
    b    re-measure baselines (hands off the pads!)
    h    show help
*/

#include "esp_arduino_version.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

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
  {    4,   "Pad 0",   10.0 },
  {   13,   "Pad 1",   10.0 },
  {   14,   "Pad 2",   10.0 },
  {   27,   "Pad 3",   10.0 },
  {   32,   "Pad 4",   10.0 },
  {   33,   "Pad 5",   10.0 },
};
const int PAD_COUNT = sizeof(PADS) / sizeof(PADS[0]);

// The secret order, as pad numbers from PADS[] above.
const int SEQUENCE[] = {4, 5, 2, 0};
const int SEQUENCE_LENGTH = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

// Longest allowed pause between touches before the attempt resets.
const unsigned long MAX_GAP_MS = 10000;

// The haunted clock's MAC address, as printed by haunted_clock.ino.
const uint8_t CLOCK_MAC[6] = {0x20, 0x50, 0x0D, 0x4E, 0x05, 0x94};

// Must match ESPNOW_CHANNEL in haunted_clock.ino.
const uint8_t ESPNOW_CHANNEL = 1;

// Each message is tried this many times in total before giving up,
// waiting SEND_RETRY_MS between tries.
const int SEND_ATTEMPTS = 5;
const unsigned long SEND_RETRY_MS = 300;

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
// Message format - must be IDENTICAL in haunted_clock.ino
// ============================================================

const uint32_t PIER_MAGIC = 0x52454950;  // "PIER" - ignore anything else
const uint8_t MSG_SOLVED = 1;            // touch puzzle solved: run the haunt
const uint8_t MSG_PING = 2;              // link test: just log it

struct __attribute__((packed)) PierMessage {
  uint32_t magic;
  uint8_t type;
  uint32_t eventId;  // random per event, so a resent copy isn't acted on twice
};

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

bool espNowReady = false;

// The message currently being sent to the clock. Sending is
// asynchronous: esp_now_send() returns straight away and the result
// (acknowledged or not) arrives later in onEspNowSent(), on the WiFi
// task. serviceOutbox() in loop() handles retries from there.
struct Outbox {
  bool active;
  bool awaitingResult;
  PierMessage msg;
  int attempts;
  unsigned long nextAttemptMs;
  unsigned long sentAtMs;
};
Outbox outbox = {};

// Written by the WiFi task, read by loop(): 0 = no result yet,
// 1 = acknowledged, 2 = not acknowledged.
volatile uint8_t sendResult = 0;

unsigned long lastSampleMs = 0;
bool liveView = false;
unsigned long lastLiveViewMs = 0;

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== Capacitive Touch Sequence Puzzle ===");
  Serial.print("Arduino-ESP32 core ");
  Serial.println(ESP_ARDUINO_VERSION_STR);

  espNowReady = startEspNow();
  if (espNowReady) {
    Serial.printf("ESP-NOW ready on channel %u. Clock: ", ESPNOW_CHANNEL);
    printMac(CLOCK_MAC);
    Serial.println();
  } else {
    Serial.println("*** ESP-NOW failed to start - the clock will NOT be told about a solve. ***");
  }

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
  serviceOutbox();

  if (!configValid) {
    return;
  }

  unsigned long now = millis();

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
  Serial.println("*** SEQUENCE CORRECT - TELLING THE CLOCK ***");
  sendToClock(MSG_SOLVED);
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

// ============================================================
// ESP-NOW link to the haunted clock
// ============================================================

void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  sendResult = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 2;
}

bool startEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    return false;
  }
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, CLOCK_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// Queues a message for the clock. A new message replaces one still
// being retried - the latest event is the one that matters.
void sendToClock(uint8_t type) {
  if (!espNowReady) {
    Serial.println("  [ESP-NOW] Not running - message not sent.");
    return;
  }
  outbox.active = true;
  outbox.awaitingResult = false;
  outbox.msg.magic = PIER_MAGIC;
  outbox.msg.type = type;
  outbox.msg.eventId = esp_random();
  outbox.attempts = 0;
  outbox.nextAttemptMs = millis();
}

void serviceOutbox() {
  if (!outbox.active) {
    return;
  }
  unsigned long now = millis();
  const char *what = outbox.msg.type == MSG_SOLVED ? "Solve" : "Ping";

  if (outbox.awaitingResult) {
    uint8_t result = sendResult;
    // The result normally arrives within a few ms. No result at all
    // after a second is treated as a failure.
    if (result == 0 && now - outbox.sentAtMs < 1000) {
      return;
    }
    outbox.awaitingResult = false;

    if (result == 1) {
      Serial.printf("  [ESP-NOW] %s delivered to the clock.\n", what);
      outbox.active = false;
      return;
    }
    if (outbox.attempts >= SEND_ATTEMPTS) {
      Serial.printf("  [ESP-NOW] *** %s NOT delivered after %d tries. Is the clock\n", what,
                    SEND_ATTEMPTS);
      Serial.println("             powered, in range, and on the same channel? ***");
      outbox.active = false;
      return;
    }
    Serial.printf("  [ESP-NOW] Clock didn't answer (try %d/%d) - retrying.\n", outbox.attempts,
                  SEND_ATTEMPTS);
    outbox.nextAttemptMs = now + SEND_RETRY_MS;
    return;
  }

  if ((long)(now - outbox.nextAttemptMs) < 0) {
    return;
  }
  sendResult = 0;
  outbox.attempts++;
  outbox.sentAtMs = now;
  outbox.awaitingResult = true;
  esp_err_t err = esp_now_send(CLOCK_MAC, (const uint8_t *)&outbox.msg, sizeof(PierMessage));
  if (err != ESP_OK) {
    // Never left this board, so no callback will come - count it as
    // a failed try straight away.
    Serial.printf("  [ESP-NOW] Send error: %s\n", esp_err_to_name(err));
    sendResult = 2;
  }
}

void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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

  // Works even with a broken pad config - the link is separate.
  if (cmd == "p") {
    Serial.println("[CMD] Pinging the clock...");
    sendToClock(MSG_PING);
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
  Serial.println("  p    ping the clock (tests the ESP-NOW link)");
  Serial.println("  h    this help");
}
