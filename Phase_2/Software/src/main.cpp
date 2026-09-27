/*
 * ESP32-S3 + MPU6050 orientation tracking + WiFi + Webpage  + UART to SD module + Launch/Abort arming + relay fire on apogee
 *
 * Wiring:
 *   MPU6050 VCC -> 3V3        MPU6050 SDA -> S3 GPIO8
 *   MPU6050 GND -> GND        MPU6050 SCL -> S3 GPIO9
 *   MPU6050 AD0 -> GND        (address 0x68)
 *
 *   S3 GPIO17 ──[1k]── SD module RX   (data out, required)
 *   S3 GPIO18 ──[1k]── SD module TX   (optional, unused here)
 *   S3 GND <---------> SD module GND  (required)
 *
 *   S3 GPIO13 -> Relay module signal pin (fires 3s on apogee)
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ---------------- WiFi config ----------------
const char* ssid     = "MyESP32";
const char* password = "12345678";
WebServer server(80); // Port 80

// ---------------- UART to SD module ----------------
#define LINK_TX_PIN  17
#define LINK_RX_PIN  18
#define LINK_BAUD    921600
HardwareSerial Link(1);

// ---------------- Relay (NEW) ----------------
#define RELAY_PIN 13
static bool relayOn = false;
static uint32_t relayStartMillis = 0;
const uint32_t RELAY_ON_MS = 3000;

// ---------------- IMU config ----------------
#define SDA_PIN      8
#define SCL_PIN      9
#define SAMPLE_HZ    100
#define MPU_ADDR     0x68

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

static const float ACC_LSB  = 8192.0f;   // +/-4 g
static const float GYRO_LSB = 65.5f;     // +/-500 dps (matches mpuInit's config)

static bool     streaming = false;
static uint32_t t0        = 0;           // time reference, for the UART elapsed field
static uint32_t nextTick  = 0;
static const uint32_t PERIOD_US = 1000000UL / SAMPLE_HZ;
static uint32_t lastLoopMicros = 0;      // for measuring real dt

// Latest raw-derived readings, for display + logging
static float latestAx, latestAy, latestAz;
static float latestGx, latestGy, latestGz;
static float latestTemp;

// ---------------- Orientation state ----------------
static float qw = 1, qx = 0, qy = 0, qz = 0;
static float bias_x = 0, bias_y = 0, bias_z = 0;

const float DEG2RAD = 0.017453293f;
const float TRIGGER_ANGLE_DEG = 85.0f;
const float COS_TRIGGER = 0.08716f;      // cos(85 degrees) -- fixed to match TRIGGER_ANGLE_DEG

static float latestUpX, latestUpY, latestUpZ = 1.0f;
static float latestTiltDeg = 0;

// ---------------- Launch + apogee state ----------------
const float ACCEL_MAG_THRESHOLD_G = 3.0f;   // tune this once you see real launch data
static bool armed = false;                  // NEW: launch detection only runs once armed
static bool launched = false;
static bool apogeeDetected = false;

// ---------------- MPU6050 low-level ----------------
static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static bool mpuInit() {
  uint8_t who = mpuRead8(REG_WHO_AM_I);
  if (who != 0x68 && who != 0x70 && who != 0x72) {
    Serial.printf("[S3] WHO_AM_I = 0x%02X (unexpected)\n", who);
    return false;
  }
  mpuWrite(REG_PWR_MGMT_1, 0x80);          // reset
  delay(100);
  mpuWrite(REG_PWR_MGMT_1, 0x01);          // wake, PLL with X gyro
  delay(10);
  mpuWrite(REG_CONFIG, 0x03);              // DLPF ~44 Hz
  mpuWrite(REG_SMPLRT_DIV, (1000 / SAMPLE_HZ) - 1);
  mpuWrite(REG_GYRO_CONFIG, 0x08);         // +/-500 dps
  mpuWrite(REG_ACCEL_CONFIG, 0x08);        // +/-4 g
  return true;
}

static bool mpuReadAll(int16_t *a, int16_t *g, int16_t *t) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  a[0] = (int16_t)((b[0]  << 8) | b[1]);
  a[1] = (int16_t)((b[2]  << 8) | b[3]);
  a[2] = (int16_t)((b[4]  << 8) | b[5]);
  *t   = (int16_t)((b[6]  << 8) | b[7]);
  g[0] = (int16_t)((b[8]  << 8) | b[9]);
  g[1] = (int16_t)((b[10] << 8) | b[11]);
  g[2] = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

// Averages gyro readings while the board should be still, to find its resting offset
static void calibrateGyroBias() {
  const int N = 1000;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  int16_t a[3], g[3], t;

  Serial.println("[S3] Calibrating gyro bias - keep still...");
  for (int i = 0; i < N; i++) {
    if (mpuReadAll(a, g, &t)) {
      sum_x += g[0] / GYRO_LSB;
      sum_y += g[1] / GYRO_LSB;
      sum_z += g[2] / GYRO_LSB;
    }
    delay(2);
  }
  bias_x = sum_x / N;
  bias_y = sum_y / N;
  bias_z = sum_z / N;
  Serial.printf("[S3] Bias (dps): %.3f, %.3f, %.3f\n", bias_x, bias_y, bias_z);
}

// Rolls the gyro reading into the running quaternion
static void updateOrientation(int16_t raw_gx, int16_t raw_gy, int16_t raw_gz, float dt) {
  float wx = ((raw_gx / GYRO_LSB) - bias_x) * DEG2RAD;
  float wy = ((raw_gy / GYRO_LSB) - bias_y) * DEG2RAD;
  float wz = ((raw_gz / GYRO_LSB) - bias_z) * DEG2RAD;

  float hx = wx * dt * 0.5f;
  float hy = wy * dt * 0.5f;
  float hz = wz * dt * 0.5f;

  float new_w = qw       - qx*hx - qy*hy - qz*hz;
  float new_x = qx + qw*hx        + qy*hz - qz*hy;
  float new_y = qy + qw*hy - qx*hz        + qz*hx;
  float new_z = qz + qw*hz + qx*hy - qy*hx;

  float norm = sqrtf(new_w*new_w + new_x*new_x + new_y*new_y + new_z*new_z);
  qw = new_w / norm;
  qx = new_x / norm;
  qy = new_y / norm;
  qz = new_z / norm;

  latestUpX = 2.0f * (qx*qz + qw*qy);
  latestUpY = 2.0f * (qy*qz - qw*qx);
  latestUpZ = 1.0f - 2.0f * (qx*qx + qy*qy);
  latestTiltDeg = acosf(constrain(latestUpZ, -1.0f, 1.0f)) * (180.0f / PI);

  // Only check apogee once we know the rocket has actually launched
  if (launched && !apogeeDetected && latestUpZ < COS_TRIGGER) {
    apogeeDetected = true;
    Serial.println("[S3] APOGEE DETECTED");

    // NEW: fire the relay, non-blocking
    digitalWrite(RELAY_PIN, HIGH);
    relayOn = true;
    relayStartMillis = millis();
  }
}

// Checks total accel magnitude for a launch impulse (engine firing)
static void checkLaunch(int16_t *a) {
  if (!armed || launched) return;   // NEW: gated behind the Launch button now

  float ax = a[0] / ACC_LSB;
  float ay = a[1] / ACC_LSB;
  float az = a[2] / ACC_LSB;
  float mag = sqrtf(ax*ax + ay*ay + az*az);   // total accel, in g, includes gravity

  if (mag > ACCEL_MAG_THRESHOLD_G) {
    launched = true;
    t0 = micros();   // reset the elapsed-time reference at launch
    Serial.printf("[S3] LAUNCH DETECTED (accel mag = %.2fg)\n", mag);
  }
}

// NEW: non-blocking relay timeout check, call every loop
static void updateRelay() {
  if (relayOn && millis() - relayStartMillis >= RELAY_ON_MS) {
    digitalWrite(RELAY_PIN, LOW);
    relayOn = false;
  }
}

// Webpage loading from data/index.html
extern const uint8_t index_html_start[] asm("_binary_data_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_data_index_html_end");

// ---------------- Web request handlers ----------------
void handleRoot() {
  size_t len = index_html_end - index_html_start;
  server.send_P(200, "text/html", (const char*)index_html_start, len);
}

void handleData() {
  String data = String(latestAx, 4) + "," + String(latestAy, 4) + "," + String(latestAz, 4) + ","
              + String(latestGx, 2) + "," + String(latestGy, 2) + "," + String(latestGz, 2) + ","
              + String(latestTemp, 1) + ","
              + String(latestUpX, 3) + "," + String(latestUpY, 3) + "," + String(latestUpZ, 3) + ","
              + String(latestTiltDeg, 1) + ","
              + String(armed ? 1 : 0) + ","
              + String(launched ? 1 : 0) + ","
              + String(apogeeDetected ? 1 : 0);
  server.send(200, "text/plain", data);
}

// NEW: Launch button handler
void handleLaunch() {
  armed = true;
  Serial.println("[S3] ARMED via web button");
  server.send(200, "text/plain", "OK");
}

// NEW: Abort button handler — full reset
void handleAbort() {
  armed = false;
  launched = false;
  apogeeDetected = false;

  // if the relay happened to be firing, cut it immediately
  digitalWrite(RELAY_PIN, LOW);
  relayOn = false;

  Serial.println("[S3] ABORTED via web button - reset to standby");
  server.send(200, "text/plain", "OK");
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[S3] boot");

  pinMode(LINK_RX_PIN, INPUT);
  Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);

  pinMode(RELAY_PIN, OUTPUT);     // NEW
  digitalWrite(RELAY_PIN, LOW);   // NEW: make sure relay starts OFF

  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  if (!mpuInit()) {
    Serial.println("[S3] MPU init failed - check wiring/address");
  } else {
    calibrateGyroBias();       // keep the board still while this runs, ~2 seconds
    Serial.println("[S3] MPU ready, tracking orientation");
    t0 = micros();
    lastLoopMicros = micros();
    nextTick = lastLoopMicros;
    streaming = true;
  }

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());     // normally 192.168.4.1

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/launch", handleLaunch);   // NEW
  server.on("/abort", handleAbort);     // NEW
  server.begin();
}

void loop() {
  server.handleClient();

  updateRelay();   // NEW: check relay timeout every loop, cheap and non-blocking

  if (!streaming) return;

  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick += PERIOD_US;
  if ((int32_t)(micros() - nextTick) > (int32_t)PERIOD_US) nextTick = micros();

  float dt = (now - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = now;

  int16_t a[3], g[3], t;
  if (!mpuReadAll(a, g, &t)) return;

  latestAx = a[0] / ACC_LSB;
  latestAy = a[1] / ACC_LSB;
  latestAz = a[2] / ACC_LSB;
  latestGx = g[0] / GYRO_LSB;
  latestGy = g[1] / GYRO_LSB;
  latestGz = g[2] / GYRO_LSB;
  latestTemp = t / 340.0f + 36.53f;

  checkLaunch(a);
  updateOrientation(g[0], g[1], g[2], dt);

  // Send the same data over UART to the SD-logging module — unchanged
  char out[160];
  int n = snprintf(out, sizeof(out),
    "I,%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d\n",
    (unsigned long)(now - t0),
    latestAx, latestAy, latestAz,
    latestGx, latestGy, latestGz,
    latestTemp, latestTiltDeg,
    launched ? 1 : 0,
    apogeeDetected ? 1 : 0);
  Link.write((const uint8_t *)out, n);
}