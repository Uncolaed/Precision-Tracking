/*
 * MPU6050 angle streamer for Arduino Uno
 * -------------------------------------------------
 * Streams Roll (X), Pitch (Y) and Yaw (Z) in degrees,
 * relative to the resting position at power-on.
 *
 * Wiring (Uno):
 *   MPU6050 VCC -> 5V
 *   MPU6050 GND -> GND
 *   MPU6050 SCL -> A5
 *   MPU6050 SDA -> A4
 *
 * No libraries to install: only the built-in Wire library.
 *
 * To view the stream:
 *   Tools > Serial Monitor   (set baud to 115200), or
 *   Tools > Serial Plotter   (set baud to 115200) for live graphs.
 *
 * Note: Roll and Pitch are stable (referenced to gravity).
 *       Yaw is gyro-only and will slowly drift — that is a
 *       hardware limitation of the MPU6050, not a code bug.
 */

#include <Wire.h>

const int MPU = 0x68;  // MPU6050 I2C address (0x69 if AD0 pin is HIGH)

// Raw sensor values, converted to physical units
float AccX, AccY, AccZ;     // in g
float GyroX, GyroY, GyroZ;  // in deg/s

// Filtered output angles
float roll, pitch, yaw;

// Gyro bias, measured at startup while the board is still
float GyroErrorX, GyroErrorY, GyroErrorZ;

// Zero reference captured from the resting position
float rollOffset, pitchOffset;

// Timing for integrating the gyro
unsigned long previousTime;

// Reads a signed 16-bit value (high byte first) from the I2C buffer.
// Done in two separate statements so byte order is guaranteed.
int16_t read16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void readAccel() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                 // ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = read16() / 16384.0;        // 16384 LSB/g for the default +/-2g range
  AccY = read16() / 16384.0;
  AccZ = read16() / 16384.0;
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);                 // GYRO_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = read16() / 131.0 - GyroErrorX;  // 131 LSB/(deg/s) for +/-250 deg/s
  GyroY = read16() / 131.0 - GyroErrorY;
  GyroZ = read16() / 131.0 - GyroErrorZ;
}

// Average a few hundred gyro readings while still to find the bias.
void calculateGyroError() {
  GyroErrorX = 0;
  GyroErrorY = 0;
  GyroErrorZ = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroErrorX += read16() / 131.0;
    GyroErrorY += read16() / 131.0;
    GyroErrorZ += read16() / 131.0;
    delay(2);
  }
  GyroErrorX /= samples;
  GyroErrorY /= samples;
  GyroErrorZ /= samples;
}

// Use the current orientation as the 0/0/0 reference.
void captureRestingAngle() {
  readAccel();
  rollOffset  = atan2(AccY, AccZ) * 180.0 / PI;
  pitchOffset = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;
  roll  = rollOffset;   // seed the filter so it starts settled
  pitch = pitchOffset;
  yaw   = 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake the MPU6050 up (it boots into sleep mode)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);   // PWR_MGMT_1 register
  Wire.write(0x00);   // clear sleep bit
  Wire.endTransmission(true);

  delay(100);

  Serial.println("Calibrating - keep the sensor still...");
  calculateGyroError();
  captureRestingAngle();
  Serial.println("Ready.");

  previousTime = millis();
}

void loop() {
  readAccel();
  readGyro();

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;  // seconds
  previousTime = currentTime;

  // Absolute roll/pitch from gravity (noisy but no drift)
  float accRoll  = atan2(AccY, AccZ) * 180.0 / PI;
  float accPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  // Complementary filter: trust gyro short-term, accel long-term
  roll  = 0.98 * (roll  + GyroX * dt) + 0.02 * accRoll;
  pitch = 0.98 * (pitch + GyroY * dt) + 0.02 * accPitch;
  yaw  += GyroZ * dt;   // no absolute reference -> slow drift

  // Print relative to the resting position.
  // "Label:value" format works in both Serial Monitor and Serial Plotter.
  Serial.print("Roll_X:");
  Serial.print(roll - rollOffset, 1);
  Serial.print("\tPitch_Y:");
  Serial.print(pitch - pitchOffset, 1);
  Serial.print("\tYaw_Z:");
  Serial.println(yaw, 1);

  delay(10);  // ~100 readings per second
}