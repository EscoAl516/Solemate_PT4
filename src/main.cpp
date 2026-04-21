#include <Arduino.h>
#include <LSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A);

//FSR Definitions
#define FSR_BIG_TOE 0

static const float ALPHA       = 0.98f;
static const float INTERVAL    = 10.0f;
static const int   CAL_SAMPLES = 50;

float pitch = 0.0f, roll = 0.0f;
float pitchOffset = 0.0f, rollOffset = 0.0f;
uint32_t lastMs = 0;

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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (imu.begin() != 0) {
    Serial.println("IMU error!");
    while (1);
  }

  analogReadResolution(12);
  calibrate();
  pitch = 0.0f;
  roll  = 0.0f;
  Serial.println("pitch,roll,aZ");
  lastMs = millis();
}

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

  // FSR — clamp to 3800 to avoid ADC near-rail distortion
  int bigToe = constrain(analogRead(FSR_BIG_TOE), 0, 3800);

  Serial.printf("%.2f,%.2f,%.3f,%d\n", pitch, roll, aZ, bigToe);
}
