#include <Wire.h>
#include <math.h> 
#include "SparkFun_BNO080_Arduino_Library.h"

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define CURRENT_SINGOLARITY_BOUND 0.499

BNO080 myIMU;

float heading=0;
float attitude=0;
float bank=0;

void setAngles(float w, float x, float y, float z) {

  float test = x*y + z*w;
  if (test > 0.499) { // singularity at north pole
    heading = 2 * atan2(x,w);
    attitude = PI/2;
    bank = 0;
    return;
  }
  if (test < -0.499) { // singularity at south pole
    heading = -2 * atan2(x,w);
    attitude = - PI/2;
    bank = 0;
    return;
  }
    float sqx = x*x;
    float sqy = y*y;
    float sqz = z*z;
    heading = atan2(2*y*w-2*x*z , 1 - 2*sqy - 2*sqz);
    attitude = asin(2*test);
    bank = atan2(2*x*w-2*y*z , 1 - 2*sqx - 2*sqz);
}


void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.begin();

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));
}


  float degreeAngle(float radians){
    return radians * 180.0 / PI ;
}
    
void loop() {
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    setAngles(quatReal, quatI, quatJ, quatK);
    //setAngles(quatReal, quatI, quatK, quatJ);
    //setAngles(quatReal, quatJ, quatK, quatI);
    
    Serial.print("heading - ");
    Serial.print(degreeAngle(heading));
    Serial.print(" --- ");
    Serial.print("attitude - ");
    Serial.print(degreeAngle(attitude));
    Serial.print(" --- ");
    Serial.print("bank - ");
    Serial.print(degreeAngle(bank));
    Serial.println(" --- ");

  } 

}
