/*
 * ESP32-CAM (AI-Thinker) — SD card IMU logger (camera not used)
 *
 * Receives IMU lines from the ESP32-S3 over UART and writes them to a CSV
 * on the SD card. Logs continuously from boot and flushes every second,
 * so a power cut loses at most ~1s of data.
 *
 * Wiring (unchanged):
 *   S3 GPIO17 ──[1k]── CAM IO13     IMU data
 *   S3 GPIO18 ──[1k]── CAM IO12     optional
 *   S3 GND   <-------> CAM GND
 *
 * Board: "AI Thinker ESP32-CAM"
 * Upload Speed: 115200
 * Core Debug Level: None
 *
 * LED (GPIO4):
 *   3 flashes           logging started
 *   short blip ~10s     still logging AND data is arriving
 *   no blips            logging, but nothing received from the S3
 *   2 flashes repeating SD card problem
 */

#include "FS.h"
#include "SD_MMC.h"

// ---------------- user config ----------------
#define LINK_RX_PIN   13
#define LINK_TX_PIN   12
#define LINK_BAUD     921600
#define FLUSH_MS      1000     // how often data is committed to the card
#define HEARTBEAT_MS  10000    // blip interval; 0 disables
// ---------------------------------------------

HardwareSerial Link(1);

static File     csv;
static char     lineBuf[192];
static uint8_t  linePos = 0;
static String   cache;

static uint32_t rows       = 0;
static uint32_t rowsAtBeat = 0;
static uint32_t lastFlush  = 0;
static uint32_t lastBeat   = 0;

// ---------------- LED ----------------
static void blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(4, HIGH); delay(100);
    digitalWrite(4, LOW);  delay(100);
  }
}

static void failForever(int code) {
  while (1) { blink(code); delay(800); }
}

// ---------------- SD ----------------
static void nextFilename(char *path) {
  for (int i = 1; i < 1000; i++) {
    sprintf(path, "/log%03d.csv", i);
    if (!SD_MMC.exists(path)) return;
  }
  strcpy(path, "/log999.csv");
}

static void writeCache() {
  if (cache.length() == 0) return;
  csv.print(cache);
  cache = "";
}

// ---------------- UART ----------------
static void pumpLink() {
  while (Link.available()) {
    char c = Link.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[linePos] = 0;
      if (linePos > 2 && lineBuf[0] == 'I' && lineBuf[1] == ',') {
        cache += String(millis());
        cache += ',';
        cache += (lineBuf + 2);      // strip the "I," tag
        cache += '\n';
        rows++;
      }
      linePos = 0;
    } else if (linePos < sizeof(lineBuf) - 1) {
      lineBuf[linePos++] = c;
    } else {
      linePos = 0;                   // overlong garbage, drop it
    }
  }
  if (cache.length() >= 4096) writeCache();
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[LOG] boot");

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  // Bigger receive buffer so nothing is lost while the SD card is busy
  Link.setRxBufferSize(4096);
  Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);

  if (!SD_MMC.begin("/sdcard", true)) {        // 1-bit mode frees IO12/13
    Serial.println("[LOG] SD mount failed");
    failForever(2);
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("[LOG] no SD card");
    failForever(2);
  }

  char path[24];
  nextFilename(path);
  csv = SD_MMC.open(path, FILE_WRITE);
  if (!csv) {
    Serial.println("[LOG] cannot create file");
    failForever(2);
  }
  csv.println("cam_ms,s3_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c");
  csv.flush();

  cache.reserve(8192);

  Serial.printf("[LOG] logging to %s\n", path);
  Link.print("START\n");
  blink(3);

  lastFlush = lastBeat = millis();
}

void loop() {
  pumpLink();

  uint32_t now = millis();

  if (now - lastFlush >= FLUSH_MS) {
    lastFlush = now;
    writeCache();
    csv.flush();                     // commit to the card
  }

  if (HEARTBEAT_MS && now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    bool dataFlowing = rows > rowsAtBeat;
    rowsAtBeat = rows;

    if (dataFlowing) {               // blip only when data is arriving
      digitalWrite(4, HIGH); delay(30);
      digitalWrite(4, LOW);
    }
    Serial.printf("[LOG] %lu rows%s\n", (unsigned long)rows,
                  dataFlowing ? "" : "  (no data from S3)");
  }
}l