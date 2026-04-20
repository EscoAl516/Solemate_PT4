#include <Arduino.h>
#include <LSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A);

// Complementary filter weight: 0.98 trusts gyro, 0.02 corrects drift via accel
static const float ALPHA    = 0.98f;
static const float INTERVAL = 10.0f; // ms between samples (~100 Hz)

float pitch = 0.0f;
float roll  = 0.0f;
uint32_t lastMs = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (imu.begin() != 0) {
    Serial.println("IMU error!");
    while (1);
  }

  Serial.println("pitch,roll,aZ");
  lastMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastMs < (uint32_t)INTERVAL) return;
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  float aX = imu.readFloatAccelX();
  float aY = imu.readFloatAccelY();
  float aZ = imu.readFloatAccelZ();
  float gX = imu.readFloatGyroX();
  float gY = imu.readFloatGyroY();

  // Pitch/roll from accelerometer (reliable when still, noisy when moving)
  float accelPitch = degrees(atan2(-aX, sqrt(aY * aY + aZ * aZ)));
  float accelRoll  = degrees(atan2(aY, aZ));

  // Blend gyro integration with accel correction
  pitch = ALPHA * (pitch + gX * dt) + (1.0f - ALPHA) * accelPitch;
  roll  = ALPHA * (roll  + gY * dt) + (1.0f - ALPHA) * accelRoll;

  Serial.printf("%.2f,%.2f,%.3f\n", pitch, roll, aZ);
}
