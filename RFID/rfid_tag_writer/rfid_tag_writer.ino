/*
  Children of the Pier - RFID Relic Tag Writer
  ----------------------------------------------------------------------
  Run this ONCE PER RELIC to write a short identifier onto its
  NTAG213 tag. Not the puzzle firmware itself - just a setup tool.

  HOW TO USE:
    1. Set TAG_IDENTIFIER below to a short name for the relic you're
       about to write (4 characters or fewer - see note below).
    2. Upload this sketch, open Serial Monitor at 115200 baud.
    3. Tap the ONE tag you want to write onto the reader.
    4. Watch Serial for "Write successful." Remove the tag.
    5. Change TAG_IDENTIFIER to the next relic's name, re-upload,
       tap the NEXT tag. Repeat for all relics.

  Why 4 characters max: NTAG213 stores data in 4-byte pages, and this
  sketch writes to a single page to keep things simple (no multi-page
  NDEF formatting needed since we're reading these back with our own
  code, not a phone's NFC reader). "TIDE", "BONE", "SALT", "PIER" all
  fit. If you want longer names, tell me and I'll extend this to
  write across multiple pages instead.

  ----------------------------------------------------------------------
  TAG TYPE MATTERS
  ----------------------------------------------------------------------
  This writes with the Ultralight page-write command (0xA2), which only
  NTAG21x / MIFARE Ultralight tags understand. MIFARE Classic tags stay
  silent when they receive it, and the library reports that silence as
  "Timeout in communication" - which reads like a wiring fault but is
  not one.

  This matters because the white card and blue keyfob bundled with
  almost every RC522 reader are MIFARE Classic 1K, not NTAG. If you
  are testing with those, that timeout is exactly what you get. This
  sketch now identifies the tag before writing and says so plainly.

  Library needed: "MFRC522" by GithubCommunity (search that in
  Arduino IDE > Manage Libraries - it's the standard, widely used one).

  Wiring (ESP32 <-> MFRC522, SPI):
    SDA/SS -> GPIO 5
    SCK    -> GPIO 18
    MOSI   -> GPIO 23
    MISO   -> GPIO 19
    RST    -> GPIO 22
    3.3V   -> 3.3V   (NOT 5V - MFRC522 is 3.3V only)
    GND    -> GND
    IRQ    -> not connected (not used here)
*/

#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 5
#define RST_PIN 22

MFRC522 mfrc522(SS_PIN, RST_PIN);

// ============================================================
// CONFIG - change this before each tag you write
// ============================================================
const char TAG_IDENTIFIER[5] = "TIDE";  // 4 characters, edit per relic

// The page most NTAG213 tags have free for user data without
// disturbing the factory NDEF/lock structure in the first few pages.
const byte WRITE_PAGE = 4;

void setup() {
  Serial.begin(115200);
  delay(300);  // let the port settle so the first lines are not lost

  // The banner prints BEFORE anything touches the reader, on purpose.
  // "Banner, then silence" means the sketch is dying inside reader init
  // - which on this hardware is nearly always the 3.3V rail sagging when
  // the antenna switches on. "No banner at all" is a completely
  // different problem: the sketch never ran. Same symptom on the wire,
  // opposite causes, so make the log distinguish them.
  Serial.println();
  Serial.println("=== RFID Tag Writer ===");
  Serial.print("Identifier to write: ");
  Serial.println(TAG_IDENTIFIER);

  Serial.println("[1/4] Starting SPI...");
  SPI.begin();

  Serial.println("[2/4] Initializing reader (this powers the antenna)...");
  mfrc522.PCD_Init();  // PCD_Init already calls PCD_AntennaOn() itself
  delay(50);           // short settle after reset

  Serial.println("[3/4] Reading reader version...");

  // Reading back the firmware version is the cheapest possible proof
  // that SPI is actually working. 0x00 or 0xFF means the ESP32 is not
  // really talking to the reader - wiring or power, not tags.
  byte version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("      Reader firmware version: 0x");
  Serial.println(version, HEX);
  if (version == 0x00 || version == 0xFF) {
    Serial.println("*** SPI COMMUNICATION FAILURE ***");
    Serial.println("    The reader is not responding at all. Check wiring");
    Serial.println("    (SS=5, SCK=18, MOSI=23, MISO=19, RST=22) and that");
    Serial.println("    the module is on 3.3V, NOT 5V.");
    Serial.println("    No point tapping tags until this reads 0x91 or 0x92.");
  } else if (version == 0x91 || version == 0x92) {
    Serial.println("      (genuine MFRC522)");
  } else {
    Serial.println("      (clone chip - usually fine, just noting it)");
  }

  // Gain must be set AFTER PCD_Init, which resets the register bank.
  // Maximum receiver gain costs nothing and buys read range, which
  // matters with tags glued inside props rather than held flat.
  Serial.println("[4/4] Setting antenna gain to max...");
  mfrc522.PCD_SetAntennaGain(MFRC522::RxGain_max);

  Serial.println();
  Serial.println("Ready. Tap the tag you want to write onto the reader now...");
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  Serial.println();
  Serial.println("Tag detected.");

  // --- What did we actually pick up? ---
  Serial.print("  UID:  ");
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    Serial.print(' ');
  }
  Serial.print(" (");
  Serial.print(mfrc522.uid.size);
  Serial.println(" bytes)");

  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.print("  Type: ");
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

  if (piccType != MFRC522::PICC_TYPE_MIFARE_UL) {
    Serial.println();
    Serial.println("*** WRONG TAG TYPE - cannot write. ***");
    Serial.println("    This sketch writes with the Ultralight command (0xA2),");
    Serial.println("    which only NTAG21x / MIFARE Ultralight tags answer.");
    Serial.println("    The tag above is a different type, so it stays silent");
    Serial.println("    and the library calls that 'Timeout in communication'.");
    Serial.println();
    Serial.println("    An NTAG213 reports type 'MIFARE Ultralight or Ultralight C'");
    Serial.println("    and has a 7-byte UID. If you see 'MIFARE 1KB' with a");
    Serial.println("    4-byte UID, that is the card/fob bundled with the reader,");
    Serial.println("    not an NTAG - use one of your actual NTAG213 tags.");
    finishTag();
    return;
  }

  // --- Prove we can talk to it before trying to change it ---
  // A read that works but a write that fails means something specific
  // (locked pages, wrong page number). Both failing means comms.
  byte readBuffer[18];
  byte readSize = sizeof(readBuffer);
  MFRC522::StatusCode readStatus = mfrc522.MIFARE_Read(WRITE_PAGE, readBuffer, &readSize);

  if (readStatus != MFRC522::STATUS_OK) {
    Serial.print("  Read of page ");
    Serial.print(WRITE_PAGE);
    Serial.print(" failed: ");
    Serial.println(mfrc522.GetStatusCodeName(readStatus));
    Serial.println("  Cannot read either, so this is a communication problem,");
    Serial.println("  not a write-protection one. Hold the tag flat and still");
    Serial.println("  against the reader, and check the module's 3.3V supply.");
    finishTag();
    return;
  }

  Serial.print("  Page ");
  Serial.print(WRITE_PAGE);
  Serial.print(" currently reads: ");
  printPageAsText(readBuffer);

  // --- Write ---
  byte dataBlock[4] = {
    (byte)TAG_IDENTIFIER[0],
    (byte)TAG_IDENTIFIER[1],
    (byte)TAG_IDENTIFIER[2],
    (byte)TAG_IDENTIFIER[3]
  };

  Serial.println("Writing...");
  MFRC522::StatusCode status = mfrc522.MIFARE_Ultralight_Write(WRITE_PAGE, dataBlock, 4);

  if (status != MFRC522::STATUS_OK) {
    Serial.print("Write failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    if (status == MFRC522::STATUS_MIFARE_NACK) {
      Serial.println("  The tag refused the write - that page is probably");
      Serial.println("  locked. Once NTAG pages are locked it is permanent.");
    } else {
      Serial.println("  Try again - keep the tag still on the reader.");
    }
    finishTag();
    return;
  }

  // --- Verify by reading it back ---
  // A successful ACK is not proof the bytes landed. Read them back.
  readSize = sizeof(readBuffer);
  readStatus = mfrc522.MIFARE_Read(WRITE_PAGE, readBuffer, &readSize);

  if (readStatus != MFRC522::STATUS_OK) {
    Serial.println("Write reported OK but verification read failed.");
    Serial.println("  Re-tap the tag to check what is actually on it.");
    finishTag();
    return;
  }

  bool verified = true;
  for (byte i = 0; i < 4; i++) {
    if (readBuffer[i] != dataBlock[i]) {
      verified = false;
    }
  }

  if (verified) {
    Serial.print("Write successful and VERIFIED. Identifier on tag: ");
    Serial.println(TAG_IDENTIFIER);
  } else {
    Serial.print("*** MISMATCH - tag now reads: ");
    printPageAsText(readBuffer);
    Serial.println("    but we wrote something else. Try again.");
  }

  finishTag();
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

void finishTag() {
  mfrc522.PICC_HaltA();
  Serial.println("Remove the tag. Change TAG_IDENTIFIER and re-upload for the next relic.");
  delay(3000);
}
