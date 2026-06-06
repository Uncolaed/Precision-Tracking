#include <Wire.h>

const uint8_t MPU6050_ADDRESS = 0x68;  // Use 0x69 if the AD0 pin is HIGH.

float mpuAccX, mpuAccY, mpuAccZ;        // g
float mpuGyroX, mpuGyroY, mpuGyroZ;     // deg/s
float mpuRoll, mpuPitch, mpuYaw;        // absolute filtered angles
float mpuGyroErrorX, mpuGyroErrorY, mpuGyroErrorZ;
float mpuRollOffset, mpuPitchOffset;
unsigned long mpuPreviousTime;
bool mpuReady = false;

int16_t readMpu16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

bool readMpuAccel() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)6, (uint8_t)true) != 6) {
    return false;
  }

  mpuAccX = readMpu16() / 16384.0;
  mpuAccY = readMpu16() / 16384.0;
  mpuAccZ = readMpu16() / 16384.0;
  return true;
}

bool readMpuGyro() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)6, (uint8_t)true) != 6) {
    return false;
  }

  mpuGyroX = readMpu16() / 131.0 - mpuGyroErrorX;
  mpuGyroY = readMpu16() / 131.0 - mpuGyroErrorY;
  mpuGyroZ = readMpu16() / 131.0 - mpuGyroErrorZ;
  return true;
}

bool calculateMpuGyroError() {
  mpuGyroErrorX = 0;
  mpuGyroErrorY = 0;
  mpuGyroErrorZ = 0;

  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    if (Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)6, (uint8_t)true) != 6) {
      return false;
    }

    mpuGyroErrorX += readMpu16() / 131.0;
    mpuGyroErrorY += readMpu16() / 131.0;
    mpuGyroErrorZ += readMpu16() / 131.0;
    delay(2);
  }

  mpuGyroErrorX /= samples;
  mpuGyroErrorY /= samples;
  mpuGyroErrorZ /= samples;
  return true;
}

bool captureMpuRestingAngle() {
  if (!readMpuAccel()) {
    return false;
  }

  mpuRollOffset = atan2(mpuAccY, mpuAccZ) * 180.0 / PI;
  mpuPitchOffset = atan2(-mpuAccX, sqrt(mpuAccY * mpuAccY + mpuAccZ * mpuAccZ)) * 180.0 / PI;
  mpuRoll = mpuRollOffset;
  mpuPitch = mpuPitchOffset;
  mpuYaw = 0;
  return true;
}

bool setupMpu6050() {
  Wire.begin();

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) {
    mpuReady = false;
    return false;
  }

  delay(100);

  if (!calculateMpuGyroError() || !captureMpuRestingAngle()) {
    mpuReady = false;
    return false;
  }

  mpuPreviousTime = millis();
  mpuReady = true;
  return true;
}

void updateMpu6050() {
  if (!mpuReady) {
    return;
  }

  if (!readMpuAccel() || !readMpuGyro()) {
    mpuReady = false;
    return;
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - mpuPreviousTime) / 1000.0;
  mpuPreviousTime = currentTime;

  if (dt <= 0.0 || dt > 0.25) {
    return;
  }

  float accRoll = atan2(mpuAccY, mpuAccZ) * 180.0 / PI;
  float accPitch = atan2(-mpuAccX, sqrt(mpuAccY * mpuAccY + mpuAccZ * mpuAccZ)) * 180.0 / PI;

  mpuRoll = 0.98 * (mpuRoll + mpuGyroX * dt) + 0.02 * accRoll;
  mpuPitch = 0.98 * (mpuPitch + mpuGyroY * dt) + 0.02 * accPitch;
  mpuYaw += mpuGyroZ * dt;
}

bool isMpuReady() {
  return mpuReady;
}

float getMpuRollDeg() {
  return mpuRoll - mpuRollOffset;
}

float getMpuPitchDeg() {
  return mpuPitch - mpuPitchOffset;
}

float getMpuYawDeg() {
  return mpuYaw;
}
