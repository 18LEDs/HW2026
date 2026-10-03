/*
  Children of the Pier - Haunted Clock - ESP32 (ESP-NOW + Servo)
  ----------------------------------------------------------------------
  Second half of the capacitive touch puzzle. When touch_puzzle.ino is
  solved it sends a "solved" message over ESP-NOW. This board swings
  the clock hand around wildly for a few seconds, then lets it wind
  down and settle on FINAL_ANGLE, where the answer is painted. After
  RETURN_DELAY_MS it quietly returns to REST_ANGLE, ready for the next
  group.

  Library needed (install via Tools > Manage Libraries):
    - ESP32Servo  (Kevin Harrington / madhephaestus)
  ESP-NOW and WiFi come with the ESP32 board package. Built for the
  classic ESP32 on Arduino-ESP32 core 3.3.x.

  ----------------------------------------------------------------------
  ABOUT THE SG90
  ----------------------------------------------------------------------
  A standard SG90 only turns 180 degrees - it cannot spin all the way
  round. The "haunting" is fast, erratic swings across that range, and
  the hand can only ever point at half the clock face. If it needs to
  reach the whole face, gear the hand 2:1 off the servo horn.
  A "continuous rotation" servo (FS90R etc.) would spin freely but has
  no position feedback, so it could not reliably stop on the answer.

  ----------------------------------------------------------------------
  HOW THE LINK WORKS
  ----------------------------------------------------------------------
  ESP-NOW sends short messages directly between ESP32s - no router, no
  WiFi network. Both boards must use the same ESPNOW_CHANNEL and the
  same PierMessage layout (copy that section into both sketches).

  This board prints its MAC address at startup (and with the "i"
  command). The touch puzzle needs that MAC to send to it.

  ----------------------------------------------------------------------
  SERIAL MONITOR (115200 baud, line ending: Newline)
  ----------------------------------------------------------------------
    t      run the full haunt, exactly as a real solve does
    a90    move the hand smoothly to 90 degrees (any MIN-MAX angle),
           for finding where the answer sits on the face
    f135   set the final angle used by the haunt (until reset - copy
           the value you like into FINAL_ANGLE and reflash)
    r      return the hand to REST_ANGLE now
    i      info: MAC address, channel, messages received
    h      help

  Wiring:
    Servo signal wire          -> GPIO 18 (or change SERVO_PIN below)
    Servo power (red)          -> 5V
    Servo ground (brown/black) -> GND (shared with ESP32 GND)

  Power note: the haunt is the hardest thing a servo can do - constant
  fast reversals, each one a current spike. If the ESP32 resets or the
  hand stutters mid-haunt, give the servo its own 5V supply and just
  share ground with the ESP32.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// ============================================================
// CONFIG
// ============================================================

const int SERVO_PIN = 18;

// Pulse widths for 0 and 180 degrees.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

// Limits of travel. Pull these in a few degrees if the servo buzzes or
// strains at the ends, or if the hand would hit something.
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

// Where the hand sits before the puzzle is solved.
const int REST_ANGLE = 90;

// Where the hand settles after the haunt - pointing at the answer.
// Placeholder for now; find the real one with the "a" command.
const int FINAL_ANGLE = 43;

// How long the wild-swinging part lasts. The wind-down onto
// FINAL_ANGLE adds about another 2 seconds after this.
const unsigned long HAUNT_SPIN_MS = 3000;

// How long the hand stays on the answer before returning to REST_ANGLE,
// and how long that return move takes.
const unsigned long RETURN_DELAY_MS = 10000;
const unsigned long RETURN_MOVE_MS = 1500;

// A new position is sent every servo frame (50Hz = 20ms).
const unsigned long SERVO_FRAME_MS = 20;

// Both boards must be on the same channel. Neither joins a WiFi
// network, so any channel 1-13 works as long as they match.
const uint8_t ESPNOW_CHANNEL = 1;

// ============================================================
// Message format - must be IDENTICAL in touch_puzzle.ino
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

Servo handServo;
float currentAngle = REST_ANGLE;
int finalAngle = FINAL_ANGLE;

bool espNowReady = false;

// Set when the haunt finishes; loop() returns the hand to rest once
// RETURN_DELAY_MS has passed. Checked in loop() rather than waited out
// with delay(), so serial commands and messages still work meanwhile.
bool returnPending = false;
unsigned long settledAtMs = 0;

// Messages arrive on the WiFi task, not the loop() task, and the haunt
// takes several seconds. So the receive callback only queues the
// message; loop() picks it up and does the real work.
struct Incoming {
  PierMessage msg;
  uint8_t mac[6];
};
QueueHandle_t incomingQueue;

uint32_t lastEventId = 0;
unsigned long messagesReceived = 0;
unsigned long lastMessageMs = 0;

// ============================================================
// ESP-NOW
// ============================================================

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(PierMessage)) {
    return;
  }
  Incoming in;
  memcpy(&in.msg, data, sizeof(PierMessage));
  if (in.msg.magic != PIER_MAGIC) {
    return;
  }
  memcpy(in.mac, info->src_addr, 6);
  xQueueSend(incomingQueue, &in, 0);  // 0 = never block in a WiFi callback
}

bool startEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    return false;
  }
  esp_now_register_recv_cb(onEspNowReceive);
  return true;
}

void handleMessage(const Incoming &in) {
  messagesReceived++;
  lastMessageMs = millis();

  Serial.printf("[ESP-NOW] From %02x:%02x:%02x:%02x:%02x:%02x  ", in.mac[0], in.mac[1],
                in.mac[2], in.mac[3], in.mac[4], in.mac[5]);

  if (in.msg.type == MSG_PING) {
    Serial.println("ping - link OK.");
  } else if (in.msg.type == MSG_SOLVED) {
    if (in.msg.eventId == lastEventId) {
      Serial.println("solved (repeat of the last one - ignored).");
      return;
    }
    lastEventId = in.msg.eventId;
    Serial.println("SOLVED - starting the haunt.");
    haunt();
  } else {
    Serial.printf("unknown message type %u - ignored.\n", in.msg.type);
  }
}

// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  incomingQueue = xQueueCreate(8, sizeof(Incoming));

  ESP32PWM::allocateTimer(0);
  handServo.setPeriodHertz(50);
  handServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  handServo.writeMicroseconds(angleToUs(REST_ANGLE));
  currentAngle = REST_ANGLE;

  Serial.println();
  Serial.println("=== Haunted Clock ===");

  espNowReady = startEspNow();
  if (espNowReady) {
    Serial.printf("ESP-NOW listening on channel %u.\n", ESPNOW_CHANNEL);
  } else {
    Serial.println("*** ESP-NOW failed to start - serial commands still work. ***");
  }
  Serial.print("This clock's MAC address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("  (the touch puzzle needs this to send to it)");
  Serial.printf("Rest angle %d, final angle %d.\n", REST_ANGLE, finalAngle);
  Serial.println();
  printHelp();
}

void loop() {
  handleSerial();

  Incoming in;
  while (xQueueReceive(incomingQueue, &in, 0) == pdTRUE) {
    handleMessage(in);
  }

  if (returnPending && millis() - settledAtMs >= RETURN_DELAY_MS) {
    returnPending = false;
    Serial.printf("Returning to rest (%d degrees).\n", REST_ANGLE);
    moveTo(REST_ANGLE, RETURN_MOVE_MS);
  }
}

// ============================================================
// Servo motion
// ============================================================

int angleToUs(float angle) {
  return SERVO_MIN_US + lroundf((SERVO_MAX_US - SERVO_MIN_US) * angle / 180.0f);
}

int clampAngle(float angle) {
  return constrain((int)lroundf(angle), MIN_ANGLE, MAX_ANGLE);
}

// Moves the hand to an angle over durationMs, easing in and out so
// each swing starts and stops like something with weight. Blocks
// until the move is finished.
void moveTo(int target, unsigned long durationMs) {
  target = clampAngle(target);
  float from = currentAngle;
  unsigned long start = millis();
  unsigned long elapsed;

  while ((elapsed = millis() - start) < durationMs) {
    float t = (float)elapsed / durationMs;
    float eased = 0.5f - 0.5f * cosf(PI * t);
    handServo.writeMicroseconds(angleToUs(from + (target - from) * eased));
    delay(SERVO_FRAME_MS);
  }
  handServo.writeMicroseconds(angleToUs(target));
  currentAngle = target;
}

// The haunt: a burst of wild swings, then a wind-down that overshoots
// the final angle less and less each time, like a pendulum losing
// energy, until it stops on the answer.
void haunt() {
  Serial.println("  Haunting...");
  unsigned long start = millis();
  bool goHigh = currentAngle < (MIN_ANGLE + MAX_ANGLE) / 2;

  while (millis() - start < HAUNT_SPIN_MS) {
    int roll = random(100);
    if (roll < 60) {
      // Big lunge to the far side - the "spinning" look.
      int target = goHigh ? random(MAX_ANGLE - 40, MAX_ANGLE + 1)
                          : random(MIN_ANGLE, MIN_ANGLE + 41);
      moveTo(target, random(180, 350));
      goHigh = !goHigh;
    } else if (roll < 85) {
      // Nervous twitches where it is.
      for (int i = random(2, 5); i > 0; i--) {
        moveTo(currentAngle + random(-12, 13), random(40, 80));
      }
    } else {
      // Freeze for a beat, as if something is holding it.
      delay(random(150, 400));
    }
  }

  Serial.println("  Settling...");
  const float overshoot[] = {45, -30, 18, -10, 5, -2, 0};
  const int swings = sizeof(overshoot) / sizeof(overshoot[0]);
  for (int i = 0; i < swings; i++) {
    moveTo(finalAngle + overshoot[i], 220 + i * 50);
  }

  Serial.printf("  Hand stopped at %d degrees. Returning to rest in %lu s.\n", finalAngle,
                RETURN_DELAY_MS / 1000);
  returnPending = true;
  settledAtMs = millis();
}

// ============================================================
// Serial Monitor test commands
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
  char c = cmd.charAt(0);
  String arg = cmd.substring(1);
  arg.trim();

  if (c == 't' && arg.length() == 0) {
    Serial.println("[CMD] Test haunt.");
    haunt();
  } else if (c == 'a') {
    int angle;
    if (!parseAngle(arg, angle)) {
      return;
    }
    returnPending = false;
    Serial.printf("[CMD] Moving to %d degrees (auto-return cancelled).\n", angle);
    moveTo(angle, 600);
  } else if (c == 'f') {
    int angle;
    if (!parseAngle(arg, angle)) {
      return;
    }
    finalAngle = angle;
    Serial.printf("[CMD] Final angle set to %d (until reset - put it in FINAL_ANGLE to keep it).\n",
                  finalAngle);
  } else if (c == 'r' && arg.length() == 0) {
    returnPending = false;
    Serial.printf("[CMD] Returning to rest (%d degrees).\n", REST_ANGLE);
    moveTo(REST_ANGLE, RETURN_MOVE_MS);
  } else if (c == 'i' && arg.length() == 0) {
    printInfo();
  } else if ((c == 'h' || c == '?') && arg.length() == 0) {
    printHelp();
  } else {
    Serial.printf("Unknown command \"%s\" - type h for help.\n", cmd.c_str());
  }
}

// Reads an angle typed after a command letter. Prints why and returns
// false if it isn't a whole number inside MIN_ANGLE-MAX_ANGLE.
bool parseAngle(const String &arg, int &angle) {
  bool digitsOnly = arg.length() > 0;
  for (unsigned int i = 0; i < arg.length(); i++) {
    if (!isDigit(arg[i])) {
      digitsOnly = false;
    }
  }
  angle = arg.toInt();
  if (!digitsOnly || angle < MIN_ANGLE || angle > MAX_ANGLE) {
    Serial.printf("Need an angle %d-%d after the letter, e.g. a90.\n", MIN_ANGLE, MAX_ANGLE);
    return false;
  }
  return true;
}

void printInfo() {
  Serial.println("--- Info ---");
  Serial.print("  MAC:       ");
  Serial.println(WiFi.macAddress());
  Serial.printf("  ESP-NOW:   %s, channel %u\n", espNowReady ? "running" : "FAILED",
                ESPNOW_CHANNEL);
  Serial.printf("  Messages:  %lu received", messagesReceived);
  if (messagesReceived > 0) {
    Serial.printf(", last %lu s ago", (millis() - lastMessageMs) / 1000);
  }
  Serial.println();
  Serial.printf("  Hand:      %d degrees (rest %d, final %d)\n", clampAngle(currentAngle),
                REST_ANGLE, finalAngle);
  if (returnPending) {
    unsigned long elapsed = millis() - settledAtMs;
    unsigned long remaining = elapsed < RETURN_DELAY_MS ? RETURN_DELAY_MS - elapsed : 0;
    Serial.printf("  Returning to rest in %lu s\n", remaining / 1000);
  }
}

void printHelp() {
  Serial.println("Commands (line ending: Newline):");
  Serial.println("  t      run the full haunt (same as a real solve)");
  Serial.printf("  a90    move the hand to an angle (%d-%d)\n", MIN_ANGLE, MAX_ANGLE);
  Serial.println("  f135   set the final angle for the haunt (until reset)");
  Serial.println("  r      return to rest angle now");
  Serial.println("  i      MAC address, channel, messages received");
  Serial.println("  h      this help");
}
