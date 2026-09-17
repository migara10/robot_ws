#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define NUM_SERVOS 6

// PCA9685 PWM Pulse Limits (Standard 50Hz)
#define SERVOMIN  125 // ~0 degrees
#define SERVOMAX  575 // ~180 degrees

// Calibrated Safe Physical Limits
const int MIN_ANGLES[NUM_SERVOS] = {0,   35,  0,   0,   0,   40};
const int MAX_ANGLES[NUM_SERVOS] = {160, 135, 180, 180, 180, 75};

// Current tracked Angles
float currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 40};

int angleToPulse(float angle) {
  return map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setServoAngle(int channel, float angle) {
  angle = constrain(angle, MIN_ANGLES[channel], MAX_ANGLES[channel]);
  pwm.setPWM(channel, 0, angleToPulse(angle));
  currentAngles[channel] = angle;
}

// Smooth move - single joint (calibration/manual testing වලට)
void moveServoSmooth(int channel, int targetAngle) {
  targetAngle = constrain(targetAngle, MIN_ANGLES[channel], MAX_ANGLES[channel]);
  int startAngle = (int)currentAngles[channel];
  int step = (targetAngle > startAngle) ? 1 : -1;
  for (int angle = startAngle; angle != targetAngle; angle += step) {
    pwm.setPWM(channel, 0, angleToPulse(angle));
    delay(15);
  }
  setServoAngle(channel, targetAngle);
}

void goHome() {
  Serial.println("Moving to Home position smoothly...");
  moveServoSmooth(5, 40);   // Gripper open first
  moveServoSmooth(4, 90);
  moveServoSmooth(3, 90);
  moveServoSmooth(2, 90);
  moveServoSmooth(1, 90);
  moveServoSmooth(0, 90);
  Serial.println("Robot Arm is in HOME position.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(500);

  goHome();

  Serial.println("\n==========================================");
  Serial.println("   6-DOF ROBOT ARM MASTER CONTROL READY   ");
  Serial.println("==========================================");
  Serial.println("Commands:");
  Serial.println("  - MoveIt trajectory: d1,d2,d3,d4,d5,d6");
  Serial.println("  - Manual single joint: <channel> <angle>");
  Serial.println("  - Reset Position: 'home'");
  Serial.println("==========================================\n");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("home")) {
      goHome();
      return;
    }

    // CSV format check (MoveIt/hardware_bridge_node trajectory commands)
    if (input.indexOf(',') > 0) {
      float angles[NUM_SERVOS];
      int idx = 0;
      int lastComma = -1;

      for (int i = 0; i <= input.length(); i++) {
        if (i == input.length() || input.charAt(i) == ',') {
          if (idx < NUM_SERVOS) {
            angles[idx] = input.substring(lastComma + 1, i).toFloat();
            idx++;
          }
          lastComma = i;
        }
      }

      if (idx == NUM_SERVOS) {
        for (int i = 0; i < NUM_SERVOS; i++) {
          setServoAngle(i, angles[i]);
        }
      }
      return;
    }

    // "channel angle" format check (manual calibration testing)
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      int ch = input.substring(0, spaceIndex).toInt();
      int angle = input.substring(spaceIndex + 1).toInt();

      if (ch >= 0 && ch <= 5) {
        Serial.print("Command: Channel ");
        Serial.print(ch);
        Serial.print(" -> Target Angle: ");
        Serial.print(angle);
        Serial.print("° (Enforced Range: ");
        Serial.print(MIN_ANGLES[ch]);
        Serial.print("°-");
        Serial.print(MAX_ANGLES[ch]);
        Serial.println("°)");
        moveServoSmooth(ch, angle);
      } else {
        Serial.println("Error: Channel must be between 0 and 5.");
      }
    }
  }
}
