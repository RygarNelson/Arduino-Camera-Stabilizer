#include <Wire.h>
#include <math.h> 
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_PWMServoDriver.h>

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define CURRENT_SINGOLARITY_BOUND 0.499
#define SERVOMIN  125
#define SERVOMAX  575
#define SERVO_MAX_ROTATION 1600
#define SERVO_MIN_ROTATION 200
#define SERVO_MIN_MAP 0
#define SERVO_MAX_MAP 1800

// IMU that detect the quaternion
BNO080 myIMU;

// Driver of the PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// the orientation on the z-axis, x-axis and y-axis calculeted by the quaterion detected by the IMU
float zOrientationDetected = 0;
float xOrientationDetected = 0;
float yOrientationDetected = 0;

// the orientation reference to reach from the detected to make the camera stable
float zOrientationReference = 0;
float xOrientationReference = 0;
float yOrientationReference = 0;

// we will use that position to avoid strange behavior during gimbal lock cases
float zPreviousAngle = 0;
float xPreviousAngle = 0;
float yPreviousAngle = 0;

bool gimbalLockReached = false;

int center = 900;

int buttonState = 0;


void setAngles(float w, float x, float y, float z) {
  float sqx = x * x;
  float sqy = y * y;
  float sqz = z * z;
  zOrientationDetected = atan2(2 * y*w - 2 * x*z, 1 - 2 * sqy - 2 * sqz);
  xOrientationDetected = asin(2 * (x*y + z * w));
  yOrientationDetected = atan2(2 * x*w - 2 * y*z, 1 - 2 * sqx - 2 * sqz);
}

float degreeAngle(float radians) {
  return radians * 180.0 / PI;
}

bool gimbalLockCase(float quatReal, float quatI, float quatJ, float quatK) {
  float testGimbalLock = quatJ * quatK + quatI * quatReal;
  if (testGimbalLock < -0.499999 || testGimbalLock > 0.499999) {
    gimbalLockReached = true;
    return true;
  }
  else {
    gimbalLockReached = false;
    return false;
  }
}

void getIMUData() {
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true) {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    if (gimbalLockCase(quatReal, quatI, quatJ, quatK)) {
      pwm.setPWM(0, 0, angleToPulse(zPreviousAngle));
      pwm.setPWM(1, 0, angleToPulse(xPreviousAngle));
      pwm.setPWM(2, 0, angleToPulse(yPreviousAngle));
    }
    else {
      setAngles(quatReal, quatJ, quatK, quatI);
    }
  }
}

int angleToPulse(float ang) {
  int roundedAng = (int)ang;
  int pulse = map(roundedAng, SERVO_MIN_MAP, SERVO_MAX_MAP, SERVOMIN, SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
  return pulse;
}

bool checkMaxMin(int rotation) {
  if (rotation > SERVO_MAX_ROTATION || rotation < SERVO_MIN_ROTATION) {
    return true;
  }
  else {
    return false;
  }
}

int improvedRound(float inNeedOfRounding) {
  int rounded = 0;
  if ((inNeedOfRounding - round(inNeedOfRounding)) < 0.5) {
    rounded = round(inNeedOfRounding);
  }
  else {
    rounded = round(inNeedOfRounding) + 1;
  }
  return rounded;
}

void checkRotationOnZaxis() {
  if (zOrientationDetected == 0 || zOrientationReference == 0) {
    zOrientationReference = zOrientationDetected;
  }
  else {
    if (zOrientationDetected != zOrientationReference) {
      float movement = degreeAngle(zOrientationReference) - degreeAngle(zOrientationDetected);
      movement = movement * 10;
      int movementRounded = improvedRound(movement);

      int angleToReach = center + movementRounded;
      zPreviousAngle = angleToReach;

      if (checkMaxMin(angleToReach)) {
        //tone(8, 2000);
        digitalWrite(9, HIGH);
      }
      else {
        //noTone(8);
        digitalWrite(9, LOW);
        pwm.setPWM(0, 0, angleToPulse(angleToReach));
      }
    }
  }
}

void checkRotationOnXaxis() {
  if (xOrientationDetected == 0 || xOrientationReference == 0) {
    xOrientationReference = xOrientationDetected;
  }
  else {
    if (xOrientationDetected != xOrientationReference) {

      float movement = degreeAngle(xOrientationDetected) - degreeAngle(xOrientationReference);
      movement = movement * 10;
      int movementRounded = improvedRound(movement);

      int angleToReach = center - movementRounded;
      xPreviousAngle = angleToReach;

      if (checkMaxMin(angleToReach)) {
        //tone(8, 2000);
        digitalWrite(9, HIGH);
      }
      else {
        //noTone(8);
        digitalWrite(9, LOW);
        pwm.setPWM(1, 0, angleToPulse(angleToReach));
      }
    }
  }
}

void checkRotationOnYaxis() {
  if (yOrientationDetected == 0 || yOrientationReference == 0) {
    yOrientationReference = yOrientationDetected;
  }
  else {
    if (yOrientationDetected != yOrientationReference) {

      float movement = degreeAngle(yOrientationReference) - degreeAngle(yOrientationDetected);
      movement = movement * 10;
      int movementRounded = improvedRound(movement);

      int angleToReach = center + movementRounded;
      yPreviousAngle = angleToReach;

      if (checkMaxMin(angleToReach)) {
        //tone(8, 2000);
        digitalWrite(9, HIGH);
      }
      else {
        //noTone(8);
        digitalWrite(9, LOW);
        pwm.setPWM(2, 0, angleToPulse(angleToReach));
      }
    }
  }
}


void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.begin();

  myIMU.enableRotationVector(50); //Send data update every 50ms

  pwm.begin();

  pwm.setPWMFreq(60);

  pwm.setPWM(0, 0, angleToPulse(900));

  pwm.setPWM(1, 0, angleToPulse(900));

  pwm.setPWM(2, 0, angleToPulse(900));


}

void loop() {

  getIMUData();

  checkRotationOnZaxis();
  checkRotationOnYaxis();
  checkRotationOnXaxis();

  buttonState = digitalRead(3);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    zOrientationReference = zOrientationDetected;
    yOrientationReference = yOrientationDetected;
    xOrientationReference = xOrientationDetected;
  }

}
