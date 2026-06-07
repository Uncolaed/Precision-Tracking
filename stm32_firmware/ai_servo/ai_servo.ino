#include <Servo.h>

bool setupMpu6050();
void updateMpu6050();
bool isMpuReady();
float getMpuRawRollDeg();
float getMpuArmXDeg();
float getMpuArmXZeroOffsetDeg();
void setMpuArmXZeroOffsetDeg(float offsetDeg);
bool zeroMpuArmXAtCurrentRoll();
float getMpuRollDeg();
float getMpuPitchDeg();
float getMpuYawDeg();

// I am using PA0 for the base servo and PA1 for the arm servo.
#define BASE_SERVO_PIN PA0
#define ARM_SERVO_PIN  PA1
#define LASER_PIN      PA8
#define STM_LED_BUILTIN PC13

// I am using UART between Raspberry Pi and STM32.
// STM32 PA10 = RX, receives from Raspberry Pi TX.
// STM32 PA9  = TX, sends back to Raspberry Pi RX.
HardwareSerial PiSerial(PA10, PA9); // RX, TX

Servo baseServo;
Servo armServo;
bool laserIsOn = false;

// I use 1500 microseconds as stop because these are continuous rotation servos.
const int PWM_STOP = 1500;

// I use this as the full speed range around stop.
// 1500 + 500 = 2000, 1500 - 500 = 1000.
const int PWM_MAX_OFFSET = 500;

// If one servo moves opposite to what I expect, I only change this value.
bool reverseBaseS  ervo = false;
bool reverseArmServo = true;

// Change these after checking the live MPU debug output.
const bool ENABLE_ARM_X_MPU_LIMIT = true;
#define DEFAULT_ARM_X_ZERO_OFFSET_DEG 1.5
const float DEFAULT_ARM_X_MIN_LIMIT_DEG = -90.0;
const float DEFAULT_ARM_X_MAX_LIMIT_DEG = 90.0;
float armXMinLimitDeg = DEFAULT_ARM_X_MIN_LIMIT_DEG;
float armXMaxLimitDeg = DEFAULT_ARM_X_MAX_LIMIT_DEG;

// Tune these if your servo direction is opposite to the Roll_X movement.
const char ARM_NEGATIVE_X_DIRECTION[] = "CCW";
const char ARM_POSITIVE_X_DIRECTION[] = "CW";

// If this is true, the arm cannot move unless the MPU is working.
const bool BLOCK_ARM_WHEN_MPU_NOT_READY = true;

const bool PRINT_MPU_DEBUG = true;
const unsigned long MPU_DEBUG_INTERVAL_MS = 500;

String currentArmDirection = "STOP";
bool armMpuLimitWasReported = false;
unsigned long lastMpuDebugTime = 0;

void controlArmServo(String direction, int speedPercent);
bool isDirectionTowardNegativeX(String direction);
bool isDirectionTowardPositiveX(String direction);
bool isArmDirectionBlockedByXLimit(String direction);
void enforceArmXLimit();
void stopArmBecauseXLimitReached();
void printMpuDebug();
bool handleLaserCommand(String input);
bool handleLimitCommand(String input);
bool handleMpuCommand(String input);
bool isFloatText(String text);
void setLaser(bool enabled);
void toggleLaser();
void printLaserStatus();
void printLimitSettings();
void printMpuSample();
void respondLine(String message);

void setup() {
  Serial.begin(9600);      // USB Serial Monitor for debugging.
  PiSerial.begin(9600);    // UART from Raspberry Pi.

  pinMode(STM_LED_BUILTIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  setLaser(false);

  delay(1000);

  if (ENABLE_ARM_X_MPU_LIMIT) {
    Serial.println("Calibrating MPU6050 - keep the turret still...");
    if (setupMpu6050()) {
      Serial.println("MPU6050 ready.");
    }
    else {
      Serial.println("MPU6050 not detected or calibration failed.");
    }
  }

  baseServo.attach(BASE_SERVO_PIN);
  armServo.attach(ARM_SERVO_PIN);

  stopBase();
  stopArm();

  Serial.println("STM32 base and arm servo controller ready.");
  Serial.println("Commands:");
  Serial.println("base cw 50");
  Serial.println("base ccw 50");
  Serial.println("base stop");
  Serial.println("arm cw 50");
  Serial.println("arm ccw 50");
  Serial.println("arm stop");
  Serial.println("laser on");
  Serial.println("laser off");
  Serial.println("laser toggle");
  Serial.println("laser show");
  Serial.println("all stop");
  Serial.println("limit show");
  Serial.println("limit offset 1.5");
  Serial.println("limit zero");
  Serial.println("limit range -90 90");
  Serial.println("mpu show");
}

void loop() 
{
  digitalWrite(STM_LED_BUILTIN, !digitalRead(STM_LED_BUILTIN));
  updateMpu6050();
  enforceArmXLimit();
  printMpuDebug();
  handleManualSerial();
  handlePiUART();
}

// I convert direction and speed percentage into the correct PWM signal.
bool controlServo(Servo &servo, String servoName, String direction, int speedPercent, bool reverseDirection) {
  speedPercent = constrain(speedPercent, 0, 100);

  int offset = map(speedPercent, 0, 100, 0, PWM_MAX_OFFSET);
  int pwmSignal = PWM_STOP;

  if (direction == "CW") {
    pwmSignal = PWM_STOP + offset;
  }
  else if (direction == "CCW") {
    pwmSignal = PWM_STOP - offset;
  }
  else if (direction == "STOP") {
    pwmSignal = PWM_STOP;
  }
  else {
    Serial.println("Invalid direction. Use CW, CCW, or STOP.");
    return false;
  }

  if (reverseDirection && direction != "STOP") {
    pwmSignal = PWM_STOP - (pwmSignal - PWM_STOP);
  }

  servo.writeMicroseconds(pwmSignal);

  Serial.print(servoName);
  Serial.print(" | Direction: ");
  Serial.print(direction);
  Serial.print(" | Speed: ");
  Serial.print(speedPercent);
  Serial.print("% | PWM: ");
  Serial.println(pwmSignal);
  return true;
}

void moveBaseCW(int speedPercent) {
  controlServo(baseServo, "Base", "CW", speedPercent, reverseBaseServo);
}

void moveBaseCCW(int speedPercent) {
  controlServo(baseServo, "Base", "CCW", speedPercent, reverseBaseServo);
}

void stopBase() {
  controlServo(baseServo, "Base", "STOP", 0, reverseBaseServo);
}

void moveArmCW(int speedPercent) {
  controlArmServo("CW", speedPercent);
}

void moveArmCCW(int speedPercent) {
  controlArmServo("CCW", speedPercent);
}

void stopArm() {
  controlArmServo("STOP", 0);
}

void stopAllServos() {
  stopBase();
  stopArm();
  Serial.println("All servos stopped.");
}

void controlArmServo(String direction, int speedPercent) {
  if (isArmDirectionBlockedByXLimit(direction)) {
    stopArmBecauseXLimitReached();
    return;
  }

  if (controlServo(armServo, "Arm", direction, speedPercent, reverseArmServo)) {
    currentArmDirection = direction;
    if (!isArmDirectionBlockedByXLimit(direction)) {
      armMpuLimitWasReported = false;
    }
  }
}

bool isDirectionTowardNegativeX(String direction) {
  return direction == ARM_NEGATIVE_X_DIRECTION;
}

bool isDirectionTowardPositiveX(String direction) {
  return direction == ARM_POSITIVE_X_DIRECTION;
}

bool isArmDirectionBlockedByXLimit(String direction) {
  if (!ENABLE_ARM_X_MPU_LIMIT || direction == "STOP") {
    return false;
  }

  if (!isMpuReady()) {
    return BLOCK_ARM_WHEN_MPU_NOT_READY;
  }

  float armX = getMpuArmXDeg();
  if (isDirectionTowardNegativeX(direction) && armX <= armXMinLimitDeg) {
    return true;
  }
  if (isDirectionTowardPositiveX(direction) && armX >= armXMaxLimitDeg) {
    return true;
  }
  return false;
}

void enforceArmXLimit() {
  if (currentArmDirection == "STOP") {
    return;
  }

  if (isArmDirectionBlockedByXLimit(currentArmDirection)) {
    stopArmBecauseXLimitReached();
  }
}

void stopArmBecauseXLimitReached() {
  controlServo(armServo, "Arm", "STOP", 0, reverseArmServo);
  currentArmDirection = "STOP";

  if (!armMpuLimitWasReported) {
    Serial.print("Arm stopped by MPU Arm_X limit. Arm_X: ");
    if (isMpuReady()) {
      Serial.print(getMpuArmXDeg(), 1);
      Serial.print(" Range:");
      Serial.print(armXMinLimitDeg, 1);
      Serial.print(" to ");
      Serial.println(armXMaxLimitDeg, 1);
    }
    else {
      Serial.println("MPU not ready");
    }
    armMpuLimitWasReported = true;
  }
}

void printMpuDebug() {
  if (!PRINT_MPU_DEBUG || !ENABLE_ARM_X_MPU_LIMIT) {
    return;
  }

  unsigned long now = millis();
  if (now - lastMpuDebugTime < MPU_DEBUG_INTERVAL_MS) {
    return;
  }
  lastMpuDebugTime = now;

  if (!isMpuReady()) {
    Serial.println("MPU: not ready");
    return;
  }

  Serial.print("MPU RawRoll_X:");
  Serial.print(getMpuRawRollDeg(), 1);
  Serial.print(" RelRoll_X:");
  Serial.print(getMpuRollDeg(), 1);
  Serial.print(" Arm_X:");
  Serial.print(getMpuArmXDeg(), 1);
  Serial.print(" Pitch_Y:");
  Serial.print(getMpuPitchDeg(), 1);
  Serial.print(" Yaw_Z:");
  Serial.print(getMpuYawDeg(), 1);
  Serial.print(" XRange:");
  Serial.print(armXMinLimitDeg, 1);
  Serial.print("..");
  Serial.print(armXMaxLimitDeg, 1);
  Serial.print(" XOffset:");
  Serial.println(getMpuArmXZeroOffsetDeg(), 1);
}

// I keep USB Serial Monitor working so I can test without the Pi if needed.
void handleManualSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    parseCommand(input);
  }
}

// I read the Raspberry Pi command from UART.
void handlePiUART() {
  if (PiSerial.available()) {
    String input = PiSerial.readStringUntil('\n');
    parseCommand(input);
  }
}

// I parse commands like: base cw 40, arm ccw 30, base stop, all stop.
void parseCommand(String input) {
  input.trim();
  input.toUpperCase();

  if (input.length() == 0) {
    return;
  }

  if (input == "PING") {
    respondLine("PONG ai_servo");
    return;
  }

  if (input == "ALL STOP" || input == "STOP") {
    stopAllServos();
    respondLine("OK all stop");
    return;
  }

  if (handleLaserCommand(input)) {
    return;
  }

  if (handleLimitCommand(input)) {
    return;
  }

  if (handleMpuCommand(input)) {
    return;
  }

  int firstSpace = input.indexOf(' ');

  if (firstSpace == -1) {
    Serial.println("Invalid command.");
    Serial.println("Use: base cw 50, arm ccw 30, base stop, or all stop");
    return;
  }

  String servoName = input.substring(0, firstSpace);
  String remainingCommand = input.substring(firstSpace + 1);
  remainingCommand.trim();

  if (remainingCommand == "STOP") {
    if (servoName == "BASE") {
      stopBase();
    }
    else if (servoName == "ARM") {
      stopArm();
    }
    else {
      Serial.println("Invalid servo name. Use BASE or ARM.");
    }

    return;
  }

  int secondSpace = remainingCommand.indexOf(' ');

  if (secondSpace == -1) {
    Serial.println("Missing speed value.");
    Serial.println("Use: base cw 50 or arm ccw 30");
    return;
  }

  String direction = remainingCommand.substring(0, secondSpace);
  String speedText = remainingCommand.substring(secondSpace + 1);
  speedText.trim();

  int speedPercent = speedText.toInt();
  speedPercent = constrain(speedPercent, 0, 100);

  if (servoName == "BASE") {
    if (direction == "CW") {
      moveBaseCW(speedPercent);
    }
    else if (direction == "CCW") {
      moveBaseCCW(speedPercent);
    }
    else if (direction == "STOP") {
      stopBase();
    }
    else {
      Serial.println("Invalid base direction. Use CW, CCW, or STOP.");
    }
  }
  else if (servoName == "ARM") {
    if (direction == "CW") {
      moveArmCW(speedPercent);
    }
    else if (direction == "CCW") {
      moveArmCCW(speedPercent);
    }
    else if (direction == "STOP") {
      stopArm();
    }
    else {
      Serial.println("Invalid arm direction. Use CW, CCW, or STOP.");
    }
  }
  else {
    Serial.println("Invalid servo name. Use BASE or ARM.");
  }
}

bool handleLaserCommand(String input) {
  if (input == "LASER ON" || input == "LASER 1") {
    setLaser(true);
    printLaserStatus();
    return true;
  }

  if (input == "LASER OFF" || input == "LASER 0") {
    setLaser(false);
    printLaserStatus();
    return true;
  }

  if (input == "LASER TOGGLE") {
    toggleLaser();
    printLaserStatus();
    return true;
  }

  if (input == "LASER SHOW" || input == "LASER STATUS") {
    printLaserStatus();
    return true;
  }

  if (input.startsWith("LASER ")) {
    respondLine("Invalid laser command. Use: laser on, laser off, laser toggle, or laser show");
    return true;
  }

  return false;
}

bool handleLimitCommand(String input) {
  if (!input.startsWith("LIMIT ")) {
    return false;
  }

  String command = input.substring(6);
  command.trim();

  if (command == "SHOW") {
    printLimitSettings();
    return true;
  }

  if (command == "ZERO") {
    if (zeroMpuArmXAtCurrentRoll()) {
      respondLine("Arm_X zero offset set from current RawRoll_X.");
    }
    else {
      respondLine("Cannot zero Arm_X because MPU is not ready.");
    }
    printLimitSettings();
    return true;
  }

  int firstSpace = command.indexOf(' ');
  if (firstSpace == -1) {
    respondLine("Invalid limit command. Use: limit show, limit zero, limit offset 1.5, limit min -90, limit max 90, or limit range -90 90");
    return true;
  }

  String setting = command.substring(0, firstSpace);
  String valueText = command.substring(firstSpace + 1);
  valueText.trim();

  if (setting == "RANGE") {
    int secondSpace = valueText.indexOf(' ');
    if (secondSpace == -1) {
      respondLine("Missing range values. Use: limit range -90 90");
      return true;
    }

    String minText = valueText.substring(0, secondSpace);
    String maxText = valueText.substring(secondSpace + 1);
    minText.trim();
    maxText.trim();

    if (!isFloatText(minText) || !isFloatText(maxText)) {
      respondLine("Invalid range values. Use numbers like: limit range -90 90");
      return true;
    }

    float newMin = minText.toFloat();
    float newMax = maxText.toFloat();
    if (newMin >= newMax) {
      respondLine("Invalid range. Min must be smaller than max.");
      return true;
    }

    armXMinLimitDeg = newMin;
    armXMaxLimitDeg = newMax;
    printLimitSettings();
    return true;
  }

  if (!isFloatText(valueText)) {
    respondLine("Invalid limit value. Use a number like: limit offset 1.5");
    return true;
  }

  float value = valueText.toFloat();
  if (setting == "OFFSET") {
    setMpuArmXZeroOffsetDeg(value);
  }
  else if (setting == "MIN") {
    if (value >= armXMaxLimitDeg) {
      respondLine("Invalid min. It must be smaller than the current max.");
      return true;
    }
    armXMinLimitDeg = value;
  }
  else if (setting == "MAX") {
    if (value <= armXMinLimitDeg) {
      respondLine("Invalid max. It must be larger than the current min.");
      return true;
    }
    armXMaxLimitDeg = value;
  }
  else {
    respondLine("Invalid limit setting. Use OFFSET, MIN, MAX, RANGE, ZERO, or SHOW.");
    return true;
  }

  printLimitSettings();
  return true;
}

bool handleMpuCommand(String input) {
  if (input == "MPU SHOW") {
    printMpuSample();
    return true;
  }

  if (input.startsWith("MPU ")) {
    respondLine("Invalid MPU command. Use: mpu show");
    return true;
  }

  return false;
}

bool isFloatText(String text) {
  text.trim();
  if (text.length() == 0) {
    return false;
  }

  bool seenDigit = false;
  bool seenDot = false;
  for (unsigned int i = 0; i < text.length(); i++) {
    char c = text.charAt(i);
    if (c >= '0' && c <= '9') {
      seenDigit = true;
    }
    else if (c == '.' && !seenDot) {
      seenDot = true;
    }
    else if ((c == '-' || c == '+') && i == 0) {
      // Leading sign is allowed.
    }
    else {
      return false;
    }
  }
  return seenDigit;
}

void setLaser(bool enabled) {
  laserIsOn = enabled;
  digitalWrite(LASER_PIN, laserIsOn ? HIGH : LOW);
}

void toggleLaser() {
  setLaser(!laserIsOn);
}

void printLaserStatus() {
  String message = "Laser ";
  message += laserIsOn ? "ON" : "OFF";
  respondLine(message);
}

void printLimitSettings() {
  String message = "Limit settings | Offset:";
  message += String(getMpuArmXZeroOffsetDeg(), 2);
  message += " Min:";
  message += String(armXMinLimitDeg, 1);
  message += " Max:";
  message += String(armXMaxLimitDeg, 1);
  if (isMpuReady()) {
    message += " RawRoll_X:";
    message += String(getMpuRawRollDeg(), 1);
    message += " Arm_X:";
    message += String(getMpuArmXDeg(), 1);
  }
  else {
    message += " MPU:not ready";
  }
  respondLine(message);
}

void printMpuSample() {
  if (!isMpuReady()) {
    respondLine("MPU sample | MPU:not ready");
    return;
  }

  String message = "MPU sample | RawRoll_X:";
  message += String(getMpuRawRollDeg(), 1);
  message += " RelRoll_X:";
  message += String(getMpuRollDeg(), 1);
  message += " Arm_X:";
  message += String(getMpuArmXDeg(), 1);
  message += " Pitch_Y:";
  message += String(getMpuPitchDeg(), 1);
  message += " Yaw_Z:";
  message += String(getMpuYawDeg(), 1);
  message += " Offset:";
  message += String(getMpuArmXZeroOffsetDeg(), 2);
  message += " Min:";
  message += String(armXMinLimitDeg, 1);
  message += " Max:";
  message += String(armXMaxLimitDeg, 1);
  respondLine(message);
}

void respondLine(String message) {
  Serial.println(message);
  PiSerial.println(message);
}
