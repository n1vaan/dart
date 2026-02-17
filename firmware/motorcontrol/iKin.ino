#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

class LegIK;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const float L1 = 75.0;   
const float L2 = 80.0;   

Servo backHipRight, frontHipRight, frontHipLeft, backHipLeft;
Servo backArmLeft, backArmRight, frontArmRight, frontArmLeft;
Servo frontKneeRight, frontKneeLeft, backKneeLeft, backKneeRight;

struct ServoMap {
  const char* name;
  int angle;
};

ServoMap config[] = {
  {"backHipRight", 90},
  {"frontHipRight", 90},
  {"frontHipLeft", 90},
  {"backHipLeft", 90},
  {"backArmLeft", 90},
  {"backArmRight", 90},
  {"frontArmRight", 90},
  {"frontArmLeft", 90},
  {"frontKneeRight", 90},
  {"frontKneeLeft", 90},
  {"backKneeLeft", 90},
  {"backKneeRight", 90}
};

const int NUM_SERVOS = sizeof(config) / sizeof(config[0]);

const int pinBackHipRight   = 2;
const int pinFrontHipRight  = 3;
const int pinFrontHipLeft   = 4;
const int pinBackHipLeft    = 5;
const int pinBackArmLeft    = 6;
const int pinBackArmRight   = 7;
const int pinFrontArmRight  = 8;
const int pinFrontArmLeft   = 9;
const int pinFrontKneeRight = 10;
const int pinFrontKneeLeft  = 11;
const int pinBackKneeLeft   = 12;
const int pinBackKneeRight  = 13;

int getZero(const char* name) {
  for (int i = 0; i < NUM_SERVOS; i++)
    if (!strcmp(config[i].name, name))
      return config[i].angle;
  return 90;
}

void setServo(const char* name, int angle) {
  angle = constrain(angle, 0, 180);
  if      (!strcmp(name,"backHipRight"))   backHipRight.write(angle);
  else if (!strcmp(name,"frontHipRight"))  frontHipRight.write(angle);
  else if (!strcmp(name,"frontHipLeft"))   frontHipLeft.write(angle);
  else if (!strcmp(name,"backHipLeft"))    backHipLeft.write(angle);
  else if (!strcmp(name,"backArmLeft"))    backArmLeft.write(angle);
  else if (!strcmp(name,"backArmRight"))   backArmRight.write(angle);
  else if (!strcmp(name,"frontArmRight"))  frontArmRight.write(angle);
  else if (!strcmp(name,"frontArmLeft"))   frontArmLeft.write(angle);
  else if (!strcmp(name,"frontKneeRight")) frontKneeRight.write(angle);
  else if (!strcmp(name,"frontKneeLeft"))  frontKneeLeft.write(angle);
  else if (!strcmp(name,"backKneeLeft"))   backKneeLeft.write(angle);
  else if (!strcmp(name,"backKneeRight"))  backKneeRight.write(angle);
}

void applyConfig() {
  for (int i = 0; i < NUM_SERVOS; i++)
    setServo(config[i].name, config[i].angle);
}

class LegIK {
public:
  float L1;
  float L2;
  const char* armName; 
  const char* kneeName;
  int armDir;
  int kneeDir;
  LegIK(float l1, float l2, const char* arm, const char* knee, int armd, int kneed){
    L1 = l1; 
    L2 = l2; 
    armName = arm;
    kneeName = knee;
    armDir = armd;
    kneeDir = kneed;
  }
  const char* getArm(){
    return armName;
  }
  const char* getKnee(){
    return kneeName; 
  }
  int getArmDir() {
    return armDir; 
  }
  int getKneeDir() {
    return kneeDir; 
  }
  bool solve(float x, float y, float &arm, float &knee) {
    float r = sqrt(x*x + y*y);
    if (r < 1.0) return false;

    float c1 = (L1*L1 + r*r - L2*L2) / (2*L1*r);
    float c2 = (L1*L1 + L2*L2 - r*r) / (2*L1*L2);

    c1 = constrain(c1, -1.0, 1.0);
    c2 = constrain(c2, -1.0, 1.0);

    float beta  = acos(c1);
    float alpha = acos(c2);
    float phi   = atan2(y, x);

    arm  = phi - beta;     
    knee = PI - alpha;     
    float arm1 = arm * 180.0 / PI; 
    float knee1 = knee * 180.0 / PI;
    return true;
  }
  void apply(float x, float y) {
    float armIK, kneeIK;
    if (!solve(x, y, armIK, kneeIK)) return;

    int armCmd =
      getZero(getArm()) + getArmDir() * (armIK * 180.0 / PI + 90);
    int kneeCmd =
      getZero(getKnee()) + getKneeDir() * (kneeIK) * 180.0 / PI;

    //     print(np.degrees(t1) + 90, np.degrees(t1+t2)+90)

    setServo(armName,  armCmd);
    setServo(kneeName, kneeCmd);
  }
};

// LegIK(float l1, float l2, const char* arm, const char* knee, int armd, int kneed){

LegIK frontLeftIK (L1, L2, "frontArmLeft", "frontKneeLeft", -1, +1);
LegIK frontRightIK(L1, L2, "frontArmRight", "frontKneeRight", +1, -1);
LegIK backLeftIK  (L1, L2, "backArmLeft", "backKneeLeft", -1, +1);
LegIK backRightIK (L1, L2, "backArmRight", "backKneeRight", +1, -1);

void setHipRoll(float rollDeg) {
  setServo("frontHipLeft",  getZero("frontHipLeft")  + rollDeg);
  setServo("backHipLeft",   getZero("backHipLeft")   + rollDeg);
  setServo("frontHipRight", getZero("frontHipRight") - rollDeg);
  setServo("backHipRight",  getZero("backHipRight")  - rollDeg);
}

void stand(float height) {
  float x = 0;
  float y = -height;

  frontLeftIK.apply(x, y);
  frontRightIK.apply(x, y);
  backLeftIK.apply(x, y);
  backRightIK.apply(x, y);
}

void tilt(float height, float delta, bool bias) {
  float x = 0;
  float y = -height;
  if (bias) {
    frontLeftIK.apply(x, y+delta);
    frontRightIK.apply(x, y+delta);
    backLeftIK.apply(x, y);
    backRightIK.apply(x, y);
  } else {
    frontLeftIK.apply(x, y);
    frontRightIK.apply(x, y);
    backLeftIK.apply(x, y+delta);
    backRightIK.apply(x, y+delta);
  }
}

void knees(float x, float y, bool bias) {
  if (bias) {
    frontLeftIK.apply(x, y);
    frontRightIK.apply(x, y);
  } else {
    backLeftIK.apply(x, y);
    backRightIK.apply(x, y);
  }
}

void walk(LegIK& leg,  float x0, float y0, float frequency, float duration, float xMax, float yMax, float del, float dt) {
  // frequency = b / 2pi
  float t = 0.0;
  float b = 2 * frequency * PI; 
  float xpos, ypos;
  while (t<duration) {
    t+=dt;
    xpos = x0 + xMax * sin(b * t);
    ypos = y0 + yMax * cos(b * t);

    Serial.print("X: ");
    Serial.println(xpos);

    Serial.print("Y: ");
    Serial.println(ypos);
    
    leg.apply(xpos, ypos);
    delay((int)(dt*1000));
  } 
}

void jump(int delta, int del) {
  
}

void setup() {
  Serial.begin(9600);

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

  if (bno.begin())
    bno.setExtCrystalUse(true);
}

void loop() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("stand ")) {
    stand(cmd.substring(6).toFloat());
  } 
  else if (cmd.startsWith("ik ")) {
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);
    if (s2 < 0 || s3 < 0) return;

    String leg = cmd.substring(s1 + 1, s2);
    float x = cmd.substring(s2 + 1, s3).toFloat();
    float y = cmd.substring(s3 + 1).toFloat();

    if (leg == "frontLeft")
      frontLeftIK.apply(x,y);
    else if (leg == "frontRight")
      frontRightIK.apply(x,y);
    else if (leg == "backLeft")
      backLeftIK.apply(x,y);
    else if (leg == "backRight")
      backRightIK.apply(x,y);
  }
  else if (cmd.startsWith("tilt ")) {
    // Expected: tilt <height> <delta> <front|back>
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);

    if (s2 < 0 || s3 < 0) return;

    float height = cmd.substring(s1 + 1, s2).toFloat();
    float delta  = cmd.substring(s2 + 1, s3).toFloat();
    String biasStr = cmd.substring(s3 + 1);
    biasStr.trim();

    bool biasFront;
    if (biasStr == "front") {
      biasFront = true;
    } else if (biasStr == "back") {
      biasFront = false;
    } else {
      Serial.println("tilt usage: tilt <height> <delta> <front|back>");
      return;
    }
    tilt(height, delta, biasFront);
  }
  else if (cmd.startsWith("jump ")) {
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);

    float delta = cmd.substring(s1 + 1, s2).toFloat();
    int delay = cmd.substring(s2 + 1).toInt();

    jump(delta, delay);
  }
  else if (cmd.startsWith("knees ")) {
    // Expected: knees <length> <height> <front|back>
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);

    if (s2 < 0 || s3 < 0) {
      Serial.println("knees usage: knees <length> <height> <front|back>");
      return;
    }

    float length = cmd.substring(s1 + 1, s2).toFloat();
    float height = cmd.substring(s2 + 1, s3).toFloat();
    String biasStr = cmd.substring(s3 + 1);
    biasStr.trim();

    bool biasFront;
    if (biasStr == "front") {
      biasFront = true;
    } else if (biasStr == "back") {
      biasFront = false;
    } else {
      Serial.println("knees usage: knees <length> <height> <front|back>");
      return;
    }

    knees(length, height, biasFront);
  }
  else if (cmd.startsWith("walk ")) {

    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);
    int s4 = cmd.indexOf(' ', s3 + 1);
    int s5 = cmd.indexOf(' ', s4 + 1);
    int s6 = cmd.indexOf(' ', s5 + 1);

    if (s6 < 0) {
      Serial.println("walk usage: walk <leg> <freq> <duration> <xMax> <yMax> <dt>");
      return;
    }

    String legName = cmd.substring(s1 + 1, s2);

    float freq     = cmd.substring(s2 + 1, s3).toFloat();
    float duration = cmd.substring(s3 + 1, s4).toFloat();
    float xMax     = cmd.substring(s4 + 1, s5).toFloat();
    float yMax     = cmd.substring(s5 + 1, s6).toFloat();
    float dt       = cmd.substring(s6 + 1).toFloat();

    LegIK* leg = nullptr;

    if (legName == "frontLeft")  leg = &frontLeftIK;
    else if (legName == "frontRight") leg = &frontRightIK;
    else if (legName == "backLeft")   leg = &backLeftIK;
    else if (legName == "backRight")  leg = &backRightIK;

    if (!leg) {
      Serial.println("Invalid leg name");
      return;
    }

    walk(*leg, 0, -140,
        freq, duration,
        xMax, yMax,
        0, dt);
  }
}
