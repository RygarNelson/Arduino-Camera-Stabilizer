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

// The orientation on the z-axis, x-axis and y-axis calculated by the quaternions detected by the IMU
float zOrientationDetected = 0;
float xOrientationDetected = 0;
float yOrientationDetected = 0;

// The orientation reference to reach from the detected to make the camera stable
float zOrientationReference = 0;
float xOrientationReference = 0;
float yOrientationReference = 0;

// We will use that position to avoid strange behavior during gimbal lock cases
float zOrientationReferencePrevious = 0;
float xOrientationReferencePrevious = 0;
float yOrientationReferencePrevious = 0;

// Center of the servos
int center = 900;

// Variable that maintain the state of the button
int buttonState = 0;

/*
Calculates the radiant angles
*/
void setAngles(float w, float x, float y, float z) {
	float sqx = x * x;
	float sqy = y * y;
	float sqz = z * z;
	zOrientationDetected = atan2(2 * y*w - 2 * x*z, 1 - 2 * sqy - 2 * sqz);
	xOrientationDetected = asin(2 * (x*y + z * w));
	yOrientationDetected = atan2(2 * x*w - 2 * y*z, 1 - 2 * sqx - 2 * sqz);
}

/*
Converts radiant angles into grades
*/
float degreeAngle(float radians) {
	return radians * 180.0 / PI;
}

/*
Checks if the system is in a gimbal lock state
*/
bool gimbalLockCase(float quatReal, float quatI, float quatJ, float quatK) {
	float testGimbalLock = quatJ * quatK + quatI * quatReal;
	if (testGimbalLock < -0.499999 || testGimbalLock > 0.499999) {
		return true;
	}
	else {
		return false;
	}
}

/*
Extraxt data from the IMU, performs gimbal lock control and calculates radiant angles
*/
void getIMUData() {
	//Look for reports from the IMU
	if (myIMU.dataAvailable() == true) {
		float quatI = myIMU.getQuatI();
		float quatJ = myIMU.getQuatJ();
		float quatK = myIMU.getQuatK();
		float quatReal = myIMU.getQuatReal();
		if (gimbalLockCase(quatReal, quatI, quatJ, quatK)) {
			zOrientationDetected = zOrientationReferencePrevious;
			xOrientationDetected = xOrientationReferencePrevious;
			yOrientationDetected = yOrientationReferencePrevious;
		}
		else {
			setAngles(quatReal, quatJ, quatK, quatI);
		}
	}
}

/*
Map the wanted servo angle with the actual servo angle
*/
int angleToPulse(float ang) {
	int roundedAng = (int)ang;
	int pulse = map(roundedAng, SERVO_MIN_MAP, SERVO_MAX_MAP, SERVOMIN, SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
	return pulse;
}

/*
Checks if a servo went over the max rotation or under the min rotation
*/
bool checkMaxMin(int rotation) {
	if (rotation > SERVO_MAX_ROTATION || rotation < SERVO_MIN_ROTATION) {
		return true;
	}
	else {
		return false;
	}
}

/*
Rounds a float number into an integer
*/
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

/*
Checks for rotations on the Z axis
*/
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

			if (checkMaxMin(angleToReach)) {
				tone(8, 2000);
			}
			else {
				noTone(8);
				pwm.setPWM(0, 0, angleToPulse(angleToReach));
			}
		}
	}
}

/*
Checks for rotations on the X axis
*/
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

			if (checkMaxMin(angleToReach)) {
				tone(8, 2000);
			}
			else {
				noTone(8);
				pwm.setPWM(1, 0, angleToPulse(angleToReach));
			}
		}
	}
}

/*
Checks for rotations on the Y axis
*/
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

			if (checkMaxMin(angleToReach)) {
				tone(8, 2000);
			}
			else {
				noTone(8);
				pwm.setPWM(2, 0, angleToPulse(angleToReach));
			}
		}
	}
}


void setup() {
	//Initialize the serial port with 9600 baudrate
	Serial.begin(9600);

	//Initialize the comunication for I2C devices
	Wire.begin();
	//Set the clock to 40khz
	Wire.setClock(400000);

	//Initialize the IMU
	myIMU.begin();
	//Set the rate of the rotation vector to 50hz
	myIMU.enableRotationVector(50);

	//Initialize the servo driver
	pwm.begin();
	//Set the frequency for every servo in the driver
	pwm.setPWMFreq(60);
	//Set all the servos to 90 degrees
	pwm.setPWM(0, 0, angleToPulse(900));
	pwm.setPWM(1, 0, angleToPulse(900));
	pwm.setPWM(2, 0, angleToPulse(900));

	//Set the pin number 8 for the button
	pinMode(8, OUTPUT);
}

void loop() {

	//Acquire data
	getIMUData();

	//Check for rotation on each axis
	checkRotationOnZaxis();
	checkRotationOnYaxis();
	checkRotationOnXaxis();

	//Read the state of the button
	buttonState = digitalRead(3);

	//Check if the push button is pressed. If it is, the buttonState is HIGH
	if (buttonState == LOW) {
		//The button is HIGH, so we set the new reference
		zOrientationReference = zOrientationDetected;
		yOrientationReference = yOrientationDetected;
		xOrientationReference = xOrientationDetected;
	}

}
