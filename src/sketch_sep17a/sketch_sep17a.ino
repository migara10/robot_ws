#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 
#define NUM_SERVOS 6

// PCA9685 PWM Pulse Limits (Standard 50Hz)
#define SERVOMIN  125 // ~0 degrees
#define SERVOMAX  575 // ~180 degrees

// Channels
#define BASE_CH      0
#define SHOULDER_CH  1
#define ELBOW_CH     2
#define WRIST_P_CH   3
#define WRIST_R_CH   4
#define GRIPPER_CH   5

// Table එකට අනුව Safe Physical Limits
const int MIN_ANGLES[NUM_SERVOS] = {0,   0,   0,   25,  0,   40};
const int MAX_ANGLES[NUM_SERVOS] = {180, 180, 180, 180, 180, 100};

// Current track කරන Angles (මුලින් 90/40 ලෙස තබයි)
int currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 40};

// Degree එක PCA9685 Pulse එකට හැරවීම
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Smooth Motion Function (එකපාර ගැස්සීම වැළැක්වීමට)
void moveServoSmooth(int channel, int targetAngle) {
  // Safe limit Enforcement
  targetAngle = constrain(targetAngle, MIN_ANGLES[channel], MAX_ANGLES[channel]);
  
  int startAngle = currentAngles[channel];
  int step = (targetAngle > startAngle) ? 1 : -1;

  for (int angle = startAngle; angle != targetAngle; angle += step) {
    pwm.setPWM(channel, 0, angleToPulse(angle));
    delay(15); // Motion Speed Delay
  }

  pwm.setPWM(channel, 0, angleToPulse(targetAngle));
  currentAngles[channel] = targetAngle;
}

// Home Position එකට මාරු වීම
void goHome() {
  Serial.println("Moving to Home position smoothly...");
  moveServoSmooth(GRIPPER_CH, 40);   // Gripper Open පළමුව
  moveServoSmooth(WRIST_R_CH, 90);
  moveServoSmooth(WRIST_P_CH, 90);
  moveServoSmooth(ELBOW_CH, 90);
  moveServoSmooth(SHOULDER_CH, 90);
  moveServoSmooth(BASE_CH, 90);
  Serial.println("Robot Arm is in HOME position.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Hardware I2C initialization (Arduino Mega SDA=20, SCL=21)
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(500);

  // Arm එක Home position එකට පත් කිරීම
  goHome();

  Serial.println("\n==========================================");
  Serial.println("   6-DOF ROBOT ARM MASTER CONTROL READY   ");
  Serial.println("==========================================");
  Serial.println("Commands:");
  Serial.println("  - Single Joint: <channel> <angle> (e.g., '0 45' or '5 100')");
  Serial.println("  - Reset Position: 'home'");
  Serial.println("==========================================\n");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("home")) {
      goHome();
    } else {
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
}
