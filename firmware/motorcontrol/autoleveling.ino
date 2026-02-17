#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo backHipRight;
Servo frontHipRight;
Servo frontHipLeft;
Servo backHipLeft;

Servo backArmLeft;
Servo backArmRight;
Servo frontArmRight;
Servo frontArmLeft;

Servo frontKneeRight;
Servo frontKneeLeft;
Servo backKneeLeft;
Servo backKneeRight;

struct ServoMap {
  const char* name;
  int angle;
};

ServoMap config[] = {
  {"backHipRight", 90},
  {"frontHipRight", 95},
  {"frontHipLeft", 100},
  {"backHipLeft", 80},
  {"backArmLeft", 100},
  {"backArmRight", 90},
  {"frontArmRight", 50},
  {"frontArmLeft", 90},
  {"frontKneeRight", 90},
  {"frontKneeLeft", 90},
  {"backKneeLeft", 15},
  {"backKneeRight", 90}
};

const int NUM_SERVOS = sizeof(config) / sizeof(config[0]);
int currentAngle[NUM_SERVOS];

const int pinBackHipRight = 2;
const int pinFrontHipRight = 3;
const int pinFrontHipLeft = 4;
const int pinBackHipLeft = 5;
const int pinBackArmLeft = 6;
const int pinBackArmRight = 7;
const int pinFrontArmRight = 8;
const int pinFrontArmLeft = 9;
const int pinFrontKneeRight = 10;
const int pinFrontKneeLeft = 11;
const int pinBackKneeLeft = 12;
const int pinBackKneeRight = 13;

bool setServoAngle(String name, int angle) {
  if (name == "backHipRight") backHipRight.write(angle);
  else if (name == "frontHipRight") frontHipRight.write(angle);
  else if (name == "frontHipLeft") frontHipLeft.write(angle);
  else if (name == "backHipLeft") backHipLeft.write(angle);
  else if (name == "backArmLeft") backArmLeft.write(angle);
  else if (name == "backArmRight") backArmRight.write(angle);
  else if (name == "frontArmRight") frontArmRight.write(angle);
  else if (name == "frontArmLeft") frontArmLeft.write(angle);
  else if (name == "frontKneeRight") frontKneeRight.write(angle);
  else if (name == "frontKneeLeft") frontKneeLeft.write(angle);
  else if (name == "backKneeLeft") backKneeLeft.write(angle);
  else if (name == "backKneeRight") backKneeRight.write(angle);
  else return false;

  for (int i = 0; i < NUM_SERVOS; i++)
    if (strcmp(config[i].name, name.c_str()) == 0)
      currentAngle[i] = angle;

  return true;
}

int getCurrentAngle(const char* name) {
  for (int i = 0; i < NUM_SERVOS; i++)
    if (strcmp(config[i].name, name) == 0)
      return currentAngle[i];
  return 90;
}

int getConfigAngle(const char* name) {
  for (int i = 0; i < NUM_SERVOS; i++)
    if (strcmp(config[i].name, name) == 0)
      return config[i].angle;
  return 90;
}

void applyConfig() {
  for (int i = 0; i < NUM_SERVOS; i++)
    setServoAngle(config[i].name, config[i].angle);
}


void getRollPitch(float &roll, float &pitch) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  roll  = euler.z();
  pitch = euler.y();

  Serial.print("Roll: ");
  Serial.println(roll);

  Serial.print("Pitch: ");
  Serial.println(pitch);
}

void autoLevel(float Kp = 1.4, int maxCorr = 10) {
  float roll, pitch;
  getRollPitch(roll, pitch);

  int rollCorr  = constrain((int)(roll  * Kp), -maxCorr, maxCorr);
  int pitchCorr = constrain((int)(pitch * Kp), -maxCorr, maxCorr);

  setServoAngle("frontHipLeft", getConfigAngle("frontHipLeft") + pitchCorr);
  setServoAngle("frontHipRight", getConfigAngle("frontHipRight") - pitchCorr);
  setServoAngle("backHipLeft", getConfigAngle("backHipLeft") + pitchCorr);
  setServoAngle("backHipRight", getConfigAngle("backHipRight") - pitchCorr);

  setServoAngle("frontHipLeft", getCurrentAngle("frontHipLeft") + rollCorr);
  setServoAngle("backHipLeft", getCurrentAngle("backHipLeft") + rollCorr);
  setServoAngle("frontHipRight", getCurrentAngle("frontHipRight") - rollCorr);
  setServoAngle("backHipRight", getCurrentAngle("backHipRight") - rollCorr);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!bno.begin()) {
    Serial.println("IMU not detected");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  backHipRight.attach(pinBackHipRight);
  frontHipRight.attach(pinFrontHipRight);
  frontHipLeft.attach(pinFrontHipLeft);
  backHipLeft.attach(pinBackHipLeft);

  backArmLeft.attach(pinBackArmLeft);
  backArmRight.attach(pinBackArmRight);
  frontArmRight.attach(pinFrontArmRight);
  frontArmLeft.attach(pinFrontArmLeft);

  frontKneeRight.attach(pinFrontKneeRight);
  frontKneeLeft.attach(pinFrontKneeLeft);
  backKneeLeft.attach(pinBackKneeLeft);
  backKneeRight.attach(pinBackKneeRight);

  applyConfig();

  for (int i = 0; i < NUM_SERVOS; i++) currentAngle[i] = config[i].angle;

}

void handleSerial(String cmd) {
  if (cmd == "level") {
    autoLevel();
    Serial.println("OK AUTOLEVEL");
  }
}

void loop() {
  autoLevel();     
  delay(1000);
}
