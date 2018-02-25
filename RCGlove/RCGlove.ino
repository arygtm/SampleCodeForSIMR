/* This is code for a glove based R/C radio. I found a standard remote to very unintuitive and difficult to use when I first started flying R/C aircraft so I a remote that allows the
 * user to fly R/C aircraft by simply tilting their hand. The remote uses an IMU to get the angular position of the persons hand and use that to change the motion the the aircraft. 
 * If the person points their hand downward then the quadcopter or plane they are flying will pitch downward but if they point their hand up then the aircraft they are flying will 
 * pitch up. This works in a similar way for roll. To control the throttle the device uses an ultrasonic sensor. As the user raises their hand, the throttle (which is proportional 
 * to the distance from the ground) will increase. Uses the same IMU and ultrasonic sensor as in the quadcopter and some of this value reading code is similar
 * 
 */



#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>3
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <stdarg.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#define DEBUG 1
#ifdef DEBUG
#include "DebugUtils.h"
#endif

#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>
#include <Serial.h>

int Atimer = 0;

float q[4];      //hold q values
float euler[4];  
float hq[4];
float *pq = &q[0];
float *peuler = &euler[0];
float rawValue[9];

double angularVelocityRoll = 0.0;
double angularVelocityPitch = 0.0;
double angularVelocityYaw = 0.0;

double thetaRollRad = 0.0;
double thetaPitchRad = 0.0;
double thetaYawDeg = 0.0;
double OurYaw = 0;
double yawDegWithoutOffset = 0;
double yawOffsetDeg = 0.0;

double lastRoll =0.0;
double lastPitch = 0.0;
double lastYaw = 0.0;

double curRoll = 0.0;
double curPitch = 0.0;
double curYaw = 0.0;

long long LastPrintTime = 0;
long long LastLoopTime = 0;

// Ultrasonic Variables
long lastChirpTimeMs = 0;

// Set the FreeIMU object
FreeSixIMU my3IMU = FreeSixIMU();

//Sets maximum and minimum angles for output of remote
const double vMin = 0.7;//ish
const double vMax = 2.5;//ish
const double yawMinRad = -PI/4;//-45˚
const double yawMaxRad = PI/4;//45˚
const double pitchMinRad = -PI/4;//-45˚
const double pitchMaxRad = PI/4;//45˚
const double rollMinRad = -PI/4;//-45˚
const double rollMaxRad = PI/4;//45˚
const double pwmMin = 1000;
const double pwmMax = 2000;



double AryaMap(double x, double x0, double x1, double y0, double y1)
{
  if(x1 != x0){
  return ((y1-y0)/(x1-x0))*(x-x0)+y0;
  }
  else
  {
    Serial.println("MAJOR ERROR, DIVIDING BY 0 IN ARYA MAxP!!!");
     return 0; 
  }
}

double UpdateUltrasound(){
  long currTimeMs = millis();
  if (currTimeMs > lastChirpTimeMs + 100) {
    sendUltrasonicChirp();
    lastChirpTimeMs = currTimeMs;
  }  
  updateDistance();

 
}

//Ultrasonic methods
int ultrasoundPin = 18;
bool sendingChirp = false;
void sendUltrasonicChirp() {
    sendingChirp = true;
    pinMode(ultrasoundPin, OUTPUT);
    digitalWrite(ultrasoundPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasoundPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(ultrasoundPin,LOW);
    sendingChirp = false;
    pinMode(ultrasoundPin,INPUT);
}

bool setDistance = false;
unsigned long ultrasoundDuration = 0;
unsigned long ultrasoundStartTimeUs = 0;
bool newUltrasoundValue = false;
void onUltrasoundChange() {
  if (sendingChirp) {
    return;
  }
  unsigned long currTimeUs = micros();
  if (digitalRead(ultrasoundPin) == HIGH) {
    ultrasoundStartTimeUs = currTimeUs;
  } else {
    ultrasoundDuration = currTimeUs - ultrasoundStartTimeUs;
    newUltrasoundValue = true;
  }
}

// Max is 400cm
int numSamples = 5;
long distMeasurements[] = {0,0,0,0,0};
int currIndex = 0;
long runningSum = 0;

long inputNextDistance(long dist) {
  runningSum += dist - distMeasurements[currIndex];
  distMeasurements[currIndex] = dist;
  currIndex++;
  if (currIndex == numSamples) {
    currIndex = 0;
  }
}

long getDistanceInCm() {
  return runningSum / numSamples;
}

void updateDistance() {
  if (!newUltrasoundValue) {
    return;
  } else {
    newUltrasoundValue = false;
  }
  long distInCm = (ultrasoundDuration / 29 / 2) * cos(thetaRollRad) * cos(thetaPitchRad);
    // this assumes 
    // 1)we never go above 13 feet (4 meters)
   //  2) that the noisy value does not just pass under the threshold
   // 3) assumes max vertical speed of 7m/s
  if(abs(distInCm - getDistanceInCm()) > 70 || distInCm > 400 || abs(thetaRollRad) * 180 / PI > 20 || abs(thetaPitchRad) * 180 / PI > 20 ) 
  {
   /* if (debugInfoOn && shouldPrint) {
      Serial.print("ULTRASOUND OUTLIER: ");
      Serial.print(distInCm);
      Serial.print(", avgDist: ");
      Serial.println(getDistanceInCm());
      if (distInCm <= 400) {
        Serial.print("residual:  ");
        Serial.println(abs(distInCm - getDistanceInCm()));
      }
    }*/
  } else {
    inputNextDistance(distInCm);
  }
}

const int throttlePin = 3;
const int rollPin = 5;
const int pitchPin = 6;
const int yawPin = 9;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);
  Serial.println("Calibrate");

  //Intializes outputs pins. Because these pins output a PWM signal and not a true analog signal, an electronic low pass filter was used 
  pinMode(throttlePin, OUTPUT); 
  pinMode(rollPin, OUTPUT); 
  pinMode(pitchPin, OUTPUT); 
  pinMode(yawPin, OUTPUT); 
  //YAW OFFSET CODE, PUT THIS LINE AFTER CALIBRATION
 // yawOffsetDeg = -yawDegWithoutOffset;
}

void loop() { 
  long currTimeMs = millis();
  long heightCm = getDistanceInCm();
    double DeltaTimeSec = (currTimeMs - LastLoopTime) / 1000.0;
  double tempYaw; 
  my3IMU.getValues(rawValue);
  rawValue[3] *= -1;
  rawValue[4] *= -1;
  rawValue[5] *= -1;
  quaternionToEuler(pq, peuler, &tempYaw, &thetaPitchRad, &thetaRollRad);
  my3IMU.getQ(q);
  double angularVelocityYawDeg = rawValue[5]; //deg/sec

  //Uses the calculated angle and then maps that to an output voltage. 
  //A large twist of the hand clockwise will result in a high voltage and a large twist of the hand counter-clockwise will result in a small voltage.
  //This voltage is then sent to through a low pass filter and finally to a transmitter
  
  yawDegWithoutOffset += angularVelocityYawDeg * DeltaTimeSec; //deg
  double thetaYawDeg = yawDegWithoutOffset + yawOffsetDeg; 
  double thetaYawRad = thetaYawDeg*PI/180;
  
  double yawVoltage = AryaMap(thetaYawRad,yawMinRad,yawMaxRad,vMin,vMax);
  double yawAnalog = yawVoltage*255/5;// 255 is max analog value and that corresponds to 5 volts
  double yawPwmOutputHigh = yawVoltage/5;//ratio of 5v time to total time is equal to voltage
  double yawRadioPwmOutput = AryaMap(yawVoltage,vMin,vMax,pwmMin,pwmMax); 
  analogWrite(yawPin,yawAnalog);
  //Serial.println(thetaYawDeg);
  
  double pitchVoltage = AryaMap(thetaPitchRad,pitchMinRad,pitchMaxRad,vMin,vMax);
  double pitchAnalog = pitchVoltage*255/5;// 255 is max analog value and that corresponds to 5 volts
  double pitchPwmOutputHigh = pitchVoltage/5;//ratio of 5v time to total time is equal to voltage
  double pitchRadioPwmOutput = AryaMap(pitchVoltage,vMin,vMax,pwmMin,pwmMax); 
  analogWrite(pitchPin,pitchAnalog);
 // Serial.println(thetaPitchRad*180/PI);
  
  double rollVoltage = AryaMap(thetaRollRad,rollMinRad,rollMaxRad,vMin,vMax);
  double rollAnalog = rollVoltage*255/5;// 255 is max analog value and that corresponds to 5 volts
  double rollPwmOutputHigh = rollVoltage/5;//ratio of 5v time to total time is equal to voltage
  double rollRadioPwmOutput = AryaMap(rollVoltage,vMin,vMax,pwmMin,pwmMax); 
  analogWrite(rollPin,rollAnalog);
  Serial.print(thetaYawDeg);
  Serial.print(" ,");
  Serial.print(thetaPitchRad*180/PI);
  Serial.print(" ,");
  Serial.println(thetaRollRad*180/PI);
  
}

//Converts quaternion output of IMU to euler angles
void quaternionToEuler(float *q, float *euler, double *Yaw, double *Pitch, double *Roll) 
{
  if (2 * q[1] * q[2] - 2 * q[0] * q[3] == 0.0f && 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1 == 0.0f) {
    Serial.println("MAJOR ERROR: these should never both be zero in a quat..");
  }
  euler[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  
  float negSinPitch = 2 * q[1] * q[3] + 2 * q[0] * q[2];
  // Take care of numerical issues: pitch should be [-90,90] deg (output is in radians)
  if (negSinPitch < -1.0f) { // -sin(pi/2) = -1
    Serial.println("MAJOR ERROR: PITCH > PI/2");
    euler[1] = 3.1415926535/2; 
  } 
  else if (negSinPitch > 1.0f) { // sin(pi/2) = 1
    Serial.println("MAJOR ERROR: PITCH < -PI/2");
    euler[1] = -3.1415926535/2;
  } else {
    euler[1] = -asin(2 * q[1] * q[3] - 2 * q[0] * q[2]);   // theta
  }
  euler[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
  *Yaw = euler[0];
  *Pitch = euler[1];
  *Roll = euler[2];
} 
