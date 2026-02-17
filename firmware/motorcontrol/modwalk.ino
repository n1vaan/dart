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

const int L1 = 75; 
const int L2 = 80;

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

const int NUM_SERVOS = sizeof(config)/sizeof(config[0]);

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

bool setServoAngle(String name, int angle);
void applyConfig();


int getCurrentAngle(const char* name) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (strcmp(config[i].name, name) == 0) {
      return currentAngle[i];
    }
  }
  return 90;
}

bool setServoAngle(String name, int angle) {

  // Move servo
  if      (name == "backHipRight")  backHipRight.write(angle);
  else if (name == "frontHipRight") frontHipRight.write(angle);
  else if (name == "frontHipLeft")  frontHipLeft.write(angle);
  else if (name == "backHipLeft")   backHipLeft.write(angle);
  else if (name == "backArmLeft")   backArmLeft.write(angle);
  else if (name == "backArmRight")  backArmRight.write(angle);
  else if (name == "frontArmRight") frontArmRight.write(angle);
  else if (name == "frontArmLeft")  frontArmLeft.write(angle);
  else if (name == "frontKneeRight") frontKneeRight.write(angle);
  else if (name == "frontKneeLeft")  frontKneeLeft.write(angle);
  else if (name == "backKneeLeft")   backKneeLeft.write(angle);
  else if (name == "backKneeRight")  backKneeRight.write(angle);
  else return false;

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (strcmp(config[i].name, name.c_str()) == 0) {
      currentAngle[i] = angle;
      break;
    }
  }

  return true;
}

void applyConfig() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(config[i].name, config[i].angle);
  }
}

int getConfigAngle(const char* name) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (strcmp(config[i].name, name) == 0) {
      return config[i].angle;
    }
  }
  return 90;  
}

void setGradualServo(String servo, int current, int angle, int delayTime, int offset) {
  int diff = angle - current;
  int stepDir = (diff >= 0) ? 1 : -1;
  int steps = abs(diff) / offset;

  for (int i = 0; i <= steps; i++) {
    int newAngle = current + stepDir * i * offset;
    setServoAngle(servo, newAngle);
    delay(delayTime);
  }

  setServoAngle(servo, angle);
}

void shiftCOM(String legName, int offset, int delayTime = 5) {

  if (legName == "frontLeft") {
    setGradualServo("frontHipLeft",  getCurrentAngle("frontHipLeft"),  getCurrentAngle("frontHipLeft")  + offset/2, delayTime, 1);
    setGradualServo("frontArmLeft",  getCurrentAngle("frontArmLeft"),  getCurrentAngle("frontArmLeft")  + offset/2, delayTime, 1);
    setGradualServo("frontKneeLeft",  getCurrentAngle("frontKneeLeft"),  getCurrentAngle("frontKneeLeft")  + offset/2, delayTime, 1);
  }

  else if (legName == "frontRight") {
    setGradualServo("frontHipRight", getCurrentAngle("frontHipRight"), getCurrentAngle("frontHipRight") - offset, delayTime, 1);
    setGradualServo("frontArmRight",  getCurrentAngle("frontArmRight"),  getCurrentAngle("frontArmRight")  + offset/2, delayTime, 1);
    setGradualServo("frontKneeRight",  getCurrentAngle("frontKneeRight"),  getCurrentAngle("frontKneeRight")  - offset/2, delayTime, 1);


  }

  else if (legName == "backLeft") {
    setGradualServo("backHipLeft",   getCurrentAngle("backHipLeft"),   getCurrentAngle("backHipLeft")   - offset, delayTime, 1);
    setGradualServo("backArmLeft",   getCurrentAngle("backArmLeft"),   getCurrentAngle("backArmLeft")   + offset, delayTime, 1);
    setGradualServo("backKneeLeft",   getCurrentAngle("backKneeLeft"),   getCurrentAngle("backKneeLeft")   + offset, delayTime, 1);
  }

  else if (legName == "backRight") {
    setGradualServo("backHipRight",  getCurrentAngle("backHipRight"),  getCurrentAngle("backHipRight")  + offset, delayTime, 1);
    setGradualServo("backArmRight",   getCurrentAngle("backArmRight"),   getCurrentAngle("backArmRight")   - offset, delayTime, 1);
    setGradualServo("backKneeRight",   getCurrentAngle("backKneeRight"),   getCurrentAngle("backKneeRight")   - offset, delayTime, 1);

  }
}

void walkForward(int steps, int delayTime, int dxFrontKnee, int dxBackKnee, int dxFrontArm, int dxBackArm) {
  for (int i = 0; i < steps; i++) {

    shiftCOM("backRight", 10);
    setServoAngle("frontKneeLeft", getCurrentAngle("frontKneeLeft") + dxFrontKnee);
    delay(delayTime);
    setServoAngle("frontArmLeft", getCurrentAngle("frontArmLeft") - dxFrontArm);
    delay(delayTime);
    shiftCOM("backRight", -10);

    shiftCOM("frontLeft", 5);
    setServoAngle("backKneeRight", getCurrentAngle("backKneeRight") - dxBackKnee);
    delay(delayTime);
    setServoAngle("backArmRight", getCurrentAngle("backArmRight") + dxBackArm);
    delay(delayTime);
    shiftCOM("frontLeft", -5);

    setServoAngle("frontKneeLeft", getConfigAngle("frontKneeLeft"));
    delay(delayTime);
    setServoAngle("frontArmLeft", getConfigAngle("frontArmLeft"));
    delay(delayTime);
    setServoAngle("backKneeRight", getConfigAngle("backKneeRight"));
    delay(delayTime);
    setServoAngle("backArmRight", getConfigAngle("backArmRight"));
    delay(delayTime);


    shiftCOM("backLeft", 15);
    setServoAngle("frontKneeRight", getCurrentAngle("frontKneeRight") - dxFrontKnee);
    delay(delayTime);
    setServoAngle("frontArmRight", getCurrentAngle("frontArmRight") + dxFrontArm);
    delay(delayTime);
    shiftCOM("backLeft", -15);

    shiftCOM("frontRight", 5);
    setServoAngle("backKneeLeft", getCurrentAngle("backKneeLeft") + dxBackKnee);
    delay(delayTime);
    setServoAngle("backArmLeft", getCurrentAngle("backArmLeft") - dxBackArm);
    delay(delayTime);
    shiftCOM("frontRight", -5);

    setServoAngle("frontKneeRight", getConfigAngle("frontKneeRight"));
    delay(delayTime);
    setServoAngle("frontArmRight", getConfigAngle("frontArmRight"));
    delay(delayTime);
    setServoAngle("backKneeLeft", getConfigAngle("backKneeLeft"));
    delay(delayTime);
    setServoAngle("backArmLeft", getConfigAngle("backArmLeft"));
    delay(delayTime);

    delay(delayTime * 2);
  }
}

void setup() {
  Serial.begin(9600);  
  while(!Serial);

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

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = config[i].angle;
  }

}

void handleSerial(String cmd) {
  Serial.println("CMD: " + cmd);

  if (cmd.startsWith("set ")) {
    cmd.remove(0, 4);
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("ERR: invalid set format");
      return;
    }
    String servo = cmd.substring(0, spaceIndex);
    int angle = cmd.substring(spaceIndex + 1).toInt();

    if (setServoAngle(servo, angle)) {
      Serial.println("OK SET " + servo + " = " + String(angle));
    } else {
      Serial.println("ERR: servo not found");
    }
  }

  else if (cmd.startsWith("get ")) {
    cmd.remove(0, 4);
    int angle = getCurrentAngle(cmd.c_str());
    Serial.println("ANGLE " + cmd + " = " + String(angle));
  }

  else if (cmd.startsWith("walk ")) {
    int steps = cmd.substring(5).toInt();
    walkForward(steps, 200, 10, 10, 20, 10);
    Serial.println("OK WALK " + String(steps));
  }

  else if (cmd == "reset") {
    applyConfig();
    Serial.println("OK RESET");
  }

  else if (cmd.startsWith("shift ")) {
    cmd.remove(0, 6); // remove "shift "
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("ERR: invalid shift format");
      return;
    }

    String leg = cmd.substring(0, spaceIndex);
    int offset = cmd.substring(spaceIndex + 1).toInt();

    shiftCOM(leg, offset);
    Serial.println("OK SHIFT " + leg + " by " + String(offset));
  }

  else if (cmd == "list") {
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.println(String(config[i].name) + " = " + String(currentAngle[i]));
    }
  }

  else {
    Serial.println("ERR: unknown command");
  }

}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    handleSerial(command);
  } 
}