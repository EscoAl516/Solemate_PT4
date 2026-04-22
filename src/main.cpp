#include <Arduino.h>
#include <LSM6DS3.h>
#include <bluefruit.h>

// ── IMU ──────────────────────────────────────────────────────────────────────
LSM6DS3 imu(I2C_MODE, 0x6A);

// ── FSR pins ─────────────────────────────────────────────────────────────────
#define FSR_BIG_TOE      0   // A0
#define FSR_BALL_INNER   1   // A1
#define FSR_BALL_OUTER   2   // A2
#define FSR_MIDFOOT      3   // A3
#define FSR_HEEL_MEDIAL  4   // A4
#define FSR_HEEL_LATERAL 5   // A5

#define FSR_MAX 3800   // clamp to avoid near-rail ADC distortion

// ── Complementary filter ─────────────────────────────────────────────────────
static const float ALPHA       = 0.98f;
static const float INTERVAL    = 10.0f;   // ms between samples (100 Hz)
static const int   CAL_SAMPLES = 50;

float pitch = 0.0f, roll = 0.0f;
float pitchOffset = 0.0f, rollOffset = 0.0f;
uint32_t lastMs = 0;

// ── BLE ───────────────────────────────────────────────────────────────────────
// Service  : 19b10000-e8f2-537e-4f6c-d104768a1214
// Char     : 19b10001-e8f2-537e-4f6c-d104768a1214
// Packet   : 9 × int16 LE (18 bytes)
//   [0] pitch      × 100
//   [1] roll       × 100
//   [2] aZ         × 1000
//   [3] bigToe
//   [4] ballInner
//   [5] ballOuter
//   [6] midfoot
//   [7] heelMedial
//   [8] heelLateral
const uint8_t SERVICE_UUID[] = {
  0x14, 0x12, 0x8a, 0x76, 0x04, 0xd1, 0x6c, 0x4f,
  0x7e, 0x53, 0xf2, 0xe8, 0x00, 0x00, 0xb1, 0x19
};
const uint8_t CHAR_UUID[] = {
  0x14, 0x12, 0x8a, 0x76, 0x04, 0xd1, 0x6c, 0x4f,
  0x7e, 0x53, 0xf2, 0xe8, 0x01, 0x00, 0xb1, 0x19
};

BLEService        soleService(SERVICE_UUID);
BLECharacteristic soleChar(CHAR_UUID);

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(soleService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ── Calibration ───────────────────────────────────────────────────────────────
void calibrate() {
  Serial.println("Calibrating — hold still...");
  float sumPitch = 0, sumRoll = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    float aX = imu.readFloatAccelX();
    float aY = imu.readFloatAccelY();
    float aZ = imu.readFloatAccelZ();
    sumPitch += degrees(atan2(-aX, sqrt(aY * aY + aZ * aZ)));
    sumRoll  += degrees(atan2(aY, aZ));
    delay(10);
  }

  pitchOffset = sumPitch / CAL_SAMPLES;
  rollOffset  = sumRoll  / CAL_SAMPLES;
  Serial.printf("Offsets — Pitch: %.2f  Roll: %.2f\n", pitchOffset, rollOffset);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  for (uint32_t t = millis(); !Serial && (millis() - t < 2000);) {}

  if (imu.begin() != 0) {
    Serial.println("IMU error!");
    while (1);
  }

  analogReadResolution(12);
  calibrate();
  pitch = 0.0f;
  roll  = 0.0f;

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("SoleMate");

  soleService.begin();

  soleChar.setProperties(CHR_PROPS_NOTIFY);
  soleChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  soleChar.setFixedLen(18);   // 9 × int16 = 18 bytes
  soleChar.begin();

  startAdv();

  Serial.println("BLE advertising as 'SoleMate'");
  Serial.println("pitch,roll,aZ,bigToe,ballInner,ballOuter,midfoot,heelMedial,heelLateral");
  lastMs = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();
  if (now - lastMs < (uint32_t)INTERVAL) return;
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  // IMU
  float aX = imu.readFloatAccelX();
  float aY = imu.readFloatAccelY();
  float aZ = imu.readFloatAccelZ();
  float gX = imu.readFloatGyroX();
  float gY = imu.readFloatGyroY();

  float accelPitch = degrees(atan2(-aX, sqrt(aY * aY + aZ * aZ))) - pitchOffset;
  float accelRoll  = degrees(atan2(aY, aZ))                        - rollOffset;

  pitch = ALPHA * (pitch + gX * dt) + (1.0f - ALPHA) * accelPitch;
  roll  = ALPHA * (roll  + gY * dt) + (1.0f - ALPHA) * accelRoll;

  // FSRs
  int bigToe      = constrain(analogRead(FSR_BIG_TOE),      0, FSR_MAX);
  int ballInner   = constrain(analogRead(FSR_BALL_INNER),   0, FSR_MAX);
  int ballOuter   = constrain(analogRead(FSR_BALL_OUTER),   0, FSR_MAX);
  int midfoot     = constrain(analogRead(FSR_MIDFOOT),      0, FSR_MAX);
  int heelMedial  = constrain(analogRead(FSR_HEEL_MEDIAL),  0, FSR_MAX);
  int heelLateral = constrain(analogRead(FSR_HEEL_LATERAL), 0, FSR_MAX);

  // Serial
  Serial.printf("%.2f,%.2f,%.3f,%d,%d,%d,%d,%d,%d\n",
    pitch, roll, aZ,
    bigToe, ballInner, ballOuter, midfoot, heelMedial, heelLateral);

  // BLE notify
  if (Bluefruit.connected() && soleChar.notifyEnabled()) {
    int16_t pkt[9];
    pkt[0] = (int16_t)(pitch  * 100.0f);
    pkt[1] = (int16_t)(roll   * 100.0f);
    pkt[2] = (int16_t)(aZ     * 1000.0f);
    pkt[3] = (int16_t)bigToe;
    pkt[4] = (int16_t)ballInner;
    pkt[5] = (int16_t)ballOuter;
    pkt[6] = (int16_t)midfoot;
    pkt[7] = (int16_t)heelMedial;
    pkt[8] = (int16_t)heelLateral;
    soleChar.notify((uint8_t*)pkt, sizeof(pkt));
  }
}
