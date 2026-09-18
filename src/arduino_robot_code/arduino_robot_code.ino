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

// Non-blocking Serial Buffer setup
char serialBuffer[64];
byte bufferIdx = 0;

int angleToPulse(float angle) {
  float pulse = SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
  return (int)(pulse + 0.5f);  // Rounding
}

void setServoAngle(int channel, float angle) {
  angle = constrain(angle, MIN_ANGLES[channel], MAX_ANGLES[channel]);
  pwm.setPWM(channel, 0, angleToPulse(angle));
  currentAngles[channel] = angle;
}

// MoveIt CSV command ekakata - channels 6ma EKAVARA (parallel) interpolate karanawa.
// Per-channel sequential loop eken command ekakata gaththa time eka (144ms wage)
// Python side MIN_INTERVAL (120ms) ekata vada wadi wela, backlog + snapping una -
// meken hema channel ekakma same steps loop eke, ekama delay set ekakin update wenawa.
void moveAllServosSmooth(float targetAngles[NUM_SERVOS]) {
  float diffs[NUM_SERVOS];
  int maxSteps = 1;

  for (int i = 0; i < NUM_SERVOS; i++) {
    float target = constrain(targetAngles[i], MIN_ANGLES[i], MAX_ANGLES[i]);
    diffs[i] = target - currentAngles[i];
    int stepsNeeded = constrain((int)round(abs(diffs[i])), 1, 8);
    if (stepsNeeded > maxSteps) maxSteps = stepsNeeded;
  }

  for (int s = 1; s <= maxSteps; s++) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (abs(diffs[i]) < 0.3) continue;  // negligible change - skip
      float stepAngle = currentAngles[i] + (diffs[i] * s / maxSteps);
      pwm.setPWM(i, 0, angleToPulse(stepAngle));
    }
    delay(2);  // channels okkoma ekata - one delay per step, sum nowei
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    float target = constrain(targetAngles[i], MIN_ANGLES[i], MAX_ANGLES[i]);
    currentAngles[i] = target;
  }
}

// Smooth move for home & manual single-joint commands
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
  Serial.println(F("Moving to Home position..."));
  moveServoSmooth(5, 40);   // Gripper open first
  moveServoSmooth(4, 90);
  moveServoSmooth(3, 90);
  moveServoSmooth(2, 90);
  moveServoSmooth(1, 90);
  moveServoSmooth(0, 90);
  Serial.println(F("Robot Arm is in HOME position."));
}

void processCommand(char* str) {
  // Check for "home" command
  if (strcasecmp(str, "home") == 0) {
    goHome();
    return;
  }

  // Fast CSV Parsing for MoveIt stream (e.g. "90.0,90.0,90.0,90.0,90.0,40.0")
  if (strchr(str, ',') != NULL) {
    float angles[NUM_SERVOS];
    int parsedCount = 0;
    char* token = strtok(str, ",");

    while (token != NULL && parsedCount < NUM_SERVOS) {
      angles[parsedCount++] = atof(token);
      token = strtok(NULL, ",");
    }

    if (parsedCount == NUM_SERVOS) {
      moveAllServosSmooth(angles);
    }
    return;
  }

  // Manual single joint command (e.g. "0 90")
  char* spacePtr = strchr(str, ' ');
  if (spacePtr != NULL) {
    int ch = atoi(str);
    int angle = atoi(spacePtr + 1);

    if (ch >= 0 && ch < NUM_SERVOS) {
      moveServoSmooth(ch, angle);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // High-Speed I2C Communication (400kHz Fast Mode)
  Wire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  goHome();

  Serial.println(F("\n=========================================="));
  Serial.println(F("   6-DOF ROBOT ARM HIGH-SPEED READY       "));
  Serial.println(F("==========================================\n"));
}

void loop() {
  // Non-blocking serial stream read
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferIdx > 0) {
        serialBuffer[bufferIdx] = '\0';
        processCommand(serialBuffer);
        bufferIdx = 0;
      }
    } else if (bufferIdx < sizeof(serialBuffer) - 1) {
      serialBuffer[bufferIdx++] = c;
    }
  }
}
