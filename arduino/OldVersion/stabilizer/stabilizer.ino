#include <Wire.h>
#include <math.h> 
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_PWMServoDriver.h>

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define CURRENT_SINGOLARITY_BOUND 0.499
#define SERVOMIN  125
#define SERVOMAX  575
#define SERVO_CORRECTION_VALUE 1
#define SERVO_MAX_ROTATION 165
#define SERVO_MIN_ROTATION 15

BNO080 myIMU;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float heading=0, headingStandard=0;
float attitude=0, attitudeStandard=0;
float bank=0, bankStandard=0;
int center = 90;

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

float degreeAngle(float radians){
    return radians * 180.0 / PI ;
}

void getIMUData(){
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true){
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
    setAngles(quatReal, quatJ, quatK, quatI); 
  }
}

int angleToPulse(float ang){
   int roundedAng = (int) ang;
   int pulse = map(roundedAng,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}

bool checkMaxMin(int rotation){
  if(rotation > SERVO_MAX_ROTATION || rotation < SERVO_MIN_ROTATION){
    return true;
  } else {
    return false;
  }
}

void checkRotationOnZaxis(){
  if(heading == 0 || headingStandard == 0){
    //Questo serve per inizializzare lo stabilizzatore, entrambi i valori vengono inizializzati a 0
    //successivamente setto heading previous come l'heading appena calcolato dall'IMU cosichè
    //leghiamo la posizione 90 del servo motore con l'attuale angolo che è tirato fuori dal quaternione
    headingStandard = heading;
  } else {
    //Appena l'heading attuale "heading" diventa diverso da quello precedente "headingStandard" abbiamo un movimento
    if(heading != headingStandard){
      
      float movement = degreeAngle(headingStandard) - degreeAngle(heading);
      int movementRounded = 0; 
      
      if(  (movement - round(movement)) < 0.5 ){
          movementRounded = round(movement);
        }else{
          movementRounded = round(movement)+1;
        }
      int angleToReach = center + movementRounded;
      if(checkMaxMin(angleToReach)){
        //Non faccio nulla
      } else {
        pwm.setPWM(0, 0, angleToPulse(angleToReach));
      }
    }
  }
}


void checkRotationOnXaxis(){
  if(attitude == 0 || attitudeStandard == 0){
    //Questo serve per inizializzare lo stabilizzatore, entrambi i valori vengono inizializzati a 0
    //successivamente setto heading previous come l'heading appena calcolato dall'IMU cosichè
    //leghiamo la posizione 90 del servo motore con l'attuale angolo che è tirato fuori dal quaternione
    attitudeStandard = attitude;
  } else {
    
    if(attitude != attitudeStandard){
      
      float movement = degreeAngle(attitudeStandard) - degreeAngle(attitude);
      int movementRounded = 0; 
      
      if(  (movement - round(movement)) < 0.5 ){
          movementRounded = round(movement);
        }else{
          movementRounded = round(movement)+1;
        }
      int angleToReach = center - movementRounded;
      if(checkMaxMin(angleToReach)){
        //Non faccio nulla
      } else {
        pwm.setPWM(2, 0, angleToPulse(angleToReach));
      }
    }
  }
}


void checkRotationOnYaxis(){
  if(bank == 0 || bankStandard == 0){
    //Questo serve per inizializzare lo stabilizzatore, entrambi i valori vengono inizializzati a 0
    //successivamente setto heading previous come l'heading appena calcolato dall'IMU cosichè
    //leghiamo la posizione 90 del servo motore con l'attuale angolo che è tirato fuori dal quaternione
    bankStandard = bank;
  } else {
    //Appena l'heading attuale "heading" diventa diverso da quello precedente "headingStandard" abbiamo un movimento
    if(bank != bankStandard){
      
      float movement = degreeAngle(bankStandard) - degreeAngle(bank);
      int movementRounded = 0; 
      
      if(  (movement - round(movement)) < 0.5 ){
          movementRounded = round(movement);
        }else{
          movementRounded = round(movement)+1;
        }
      int angleToReach = center + movementRounded;
      if(checkMaxMin(angleToReach)){
        //Non faccio nulla
      } else {
        pwm.setPWM(1, 0, angleToPulse(angleToReach));
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
}

void loop() {
  getIMUData();
  checkRotationOnZaxis();
  checkRotationOnYaxis();
  checkRotationOnXaxis();
}
