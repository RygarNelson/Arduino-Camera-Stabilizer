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

BNO080 myIMU;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float zOrientationDetected=0, zOrientationReference=0;
float xOrientationDetected=0, xOrientationReference=0;
float yOrientationDetected=0, yOrientationReference=0;

float zOrientationReferenceOLD;
float xOrientationReferenceOLD;
float yOrientationReferenceOLD;

int center = 900;
int buttonState1 = 0;
int buttonState2 = 0;


void setAngles(float w, float x, float y, float z) {
  float test = x*y + z*w;
  //if (test > 0.499) { // singularity at north pole
  //  zOrientationDetected = 2 * atan2(x,w);
  //  xOrientationDetected = PI/2;
  //  yOrientationDetected = 0;
  //  return;
  //}
  //if (test < -0.499) { // singularity at south pole
  //  zOrientationDetected = -2 * atan2(x,w);
  //  xOrientationDetected = - PI/2;
  //  yOrientationDetected = 0;
  //  return;
  //}
    float sqx = x*x;
    float sqy = y*y;
    float sqz = z*z;
    zOrientationDetected = atan2(2*y*w-2*x*z , 1 - 2*sqy - 2*sqz);
    xOrientationDetected = asin(2*test);
    yOrientationDetected = atan2(2*x*w-2*y*z , 1 - 2*sqx - 2*sqz);
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
   int pulse = map(roundedAng,0, 1800, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
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
  if(zOrientationDetected == 0 || zOrientationReference == 0){
    zOrientationReference = zOrientationDetected;
  } else {
   
    if(zOrientationDetected != zOrientationReference){
      
      float movement = degreeAngle(zOrientationReference) - degreeAngle(zOrientationDetected);
      int movementRounded = 0; 
      movement = movement*10;
      
      if(  (movement - round(movement)) < 0.5 ){
          movementRounded = round(movement);
        }else{
          movementRounded = round(movement)+1;
        }
      int angleToReach = center + movementRounded;
      if(checkMaxMin(angleToReach)){
        //Non faccio nulla
        tone(8, 2000);
      } else {
        noTone(8);
        Serial.print("z =  ");
        Serial.print(angleToPulse(angleToReach));
        Serial.print(",");
        pwm.setPWM(0, 0, angleToPulse(angleToReach));
      }
    }
  }
}


void checkRotationOnXaxis(){
  if(xOrientationDetected == 0 || xOrientationReference == 0){
    //Questo serve per inizializzare lo stabilizzatore, entrambi i valori vengono inizializzati a 0
    //successivamente setto zOrientationDetected previous come l'zOrientationDetected appena calcolato dall'IMU cosichè
    //leghiamo la posizione 90 del servo motore con l'attuale angolo che è tirato fuori dal quaternione
    xOrientationReference = xOrientationDetected;
  } else {
    
    if(xOrientationDetected != xOrientationReference){
      
      float movement = degreeAngle(xOrientationReference) - degreeAngle(xOrientationDetected);
      int movementRounded = 0; 
      movement = movement*10;
      if(  (movement - round(movement)) < 0.5 ){
          movementRounded = round(movement);
        }else{
          movementRounded = round(movement)+1;
        }
      int angleToReach = center - movementRounded;
      if(checkMaxMin(angleToReach)){
        //Non faccio nulla
        tone(8, 2000);
      } else {
        noTone(8);
        Serial.print("z =  ");
        Serial.print(angleToPulse(angleToReach));
        Serial.print(",");
        pwm.setPWM(2, 0, angleToPulse(angleToReach));
      }
    }
  }
}


boolean reEnteredClockwise(int angle){
    if(angle>=SERVO_MIN_ROTATION || angle < center ){
        return true;
      }
    else{
        return false;
      }
  }

boolean reEnteredCounterClockwise(int angle){
    if(angle<=SERVO_MAX_ROTATION || angle > center ){
        return true;
      }
    else{
        return false;
      }
  }

void checkRotationOnYaxis(){
  if(yOrientationDetected == 0 || yOrientationReference == 0){
    //Questo serve per inizializzare lo stabilizzatore, entrambi i valori vengono inizializzati a 0
    //successivamente setto zOrientationDetected previous come l'zOrientationDetected appena calcolato dall'IMU cosichè
    //leghiamo la posizione 90 del servo motore con l'attuale angolo che è tirato fuori dal quaternione
    yOrientationReference = yOrientationDetected;
  } else {
    //Appena l'zOrientationDetected attuale "zOrientationDetected" diventa diverso da quello precedente "zOrientationReference" abbiamo un movimento
    if(yOrientationDetected != yOrientationReference){
      
      float movement = degreeAngle(yOrientationReference) - degreeAngle(yOrientationDetected);
      int movementRounded = 0; 
      movement = movement*10;
      if(  (movement - round(movement)) < 0.5 ){
          movementRounded = round(movement);
        }else{
          movementRounded = round(movement)+1;
        }
      int angleToReach = center + movementRounded;
      if(checkMaxMin(angleToReach)){
        //Non faccio nulla
        tone(8, 2000);
      } else {
        noTone(8);
        Serial.print("z =  ");
        Serial.print(angleToPulse(angleToReach));
        Serial.print(",");
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

  pwm.setPWM(0, 0, angleToPulse(900));

  pwm.setPWM(1, 0, angleToPulse(900));

  pwm.setPWM(2, 0, angleToPulse(900));

}

void loop() {

  boolean buttonPressed2 = false;
  if (buttonState2 == HIGH && !buttonPressed2) {
     zOrientationReferenceOLD = zOrientationDetected;
     yOrientationReferenceOLD = yOrientationDetected;
     xOrientationReferenceOLD = xOrientationDetected;
     
     buttonPressed2 = true;
  } 
  if (buttonState2 == LOW) {
     getIMUData();
     buttonPressed2 = false;
  } 
  
  checkRotationOnZaxis();
  checkRotationOnYaxis();
  checkRotationOnXaxis();
  buttonState1 = digitalRead(2);
  buttonState2 = digitalRead(4);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState1 == HIGH) {
    zOrientationReference = zOrientationDetected;
    yOrientationReference = yOrientationDetected;
    xOrientationReference = xOrientationDetected;
  } 


}
