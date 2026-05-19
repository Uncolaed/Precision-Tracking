#include <Servo.h>

// I am using PA0 for the base servo and PA1 for the arm servo.
#define BASE_SERVO_PIN PA0
#define ARM_SERVO_PIN  PA1

// I am using UART between Raspberry Pi and STM32.
// STM32 PA10 = RX, receives from Raspberry Pi TX.
// STM32 PA9  = TX, sends back to Raspberry Pi RX.
HardwareSerial PiSerial(PA10, PA9); // RX, TX

Servo baseServo;
Servo armServo;

// I use 1500 microseconds as stop because these are continuous rotation servos.
const int PWM_STOP = 1500;

// I use this as the full speed range around stop.
// 1500 + 500 = 2000, 1500 - 500 = 1000.
const int PWM_MAX_OFFSET = 500;

// If one servo moves opposite to what I expect, I only change this value.
bool reverseBaseServo = false;
bool reverseArmServo = false;

void setup() {
  Serial.begin(9600);      // USB Serial Monitor for debugging.
  PiSerial.begin(9600);    // UART from Raspberry Pi.

  delay(1000);

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
  Serial.println("all stop");
}

void loop() {
  handleManualSerial();
  handlePiUART();
}

// I convert direction and speed percentage into the correct PWM signal.
void controlServo(Servo &servo, String servoName, String direction, int speedPercent, bool reverseDirection) {
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
    return;
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
  controlServo(armServo, "Arm", "CW", speedPercent, reverseArmServo);
}

void moveArmCCW(int speedPercent) {
  controlServo(armServo, "Arm", "CCW", speedPercent, reverseArmServo);
}

void stopArm() {
  controlServo(armServo, "Arm", "STOP", 0, reverseArmServo);
}

void stopAllServos() {
  stopBase();
  stopArm();
  Serial.println("All servos stopped.");
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

  if (input == "ALL STOP" || input == "STOP") {
    stopAllServos();
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