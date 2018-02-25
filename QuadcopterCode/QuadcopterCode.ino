/* This is code that runs an Arduino based quadcopter I built. The program will get values from the IMU onboard the quadcopter and then use a PID algorithm on these angles to figure out the 
 * ideal motor speeds to properly balance the quadcopter. I tuned the PID constants using a test rig that limited the quadcopters motion to only one axis. There is a manual mode and an 
 * autonomous mode. In the manual mode the program will read values from the quadcopter's radio reciever to determine the desired pitch, roll, yaw, and throttle and it adjusts the motor 
 * output based on these as well. In the autonomous mode the quadcopter will read values from the ultrasonic sensor and figure out its height. It will then adjust its motor power to stay 
 * at the same height.
 *
 *  
 *  
 *  
 *  
 *  
 *  
 *  
 */

#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>3
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include "Arduino.h"
#include <Servo.h>

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

long lastChirpTimeMs = 0;

//Creates Servos to activate speed controllers on Quadcopter
Servo Motor_1; 
Servo Motor_2;
Servo Motor_3;
Servo Motor_4;

bool debugInfoOn = true;
bool timeToPrint = false;

//Method to help pause printing so program does not slow down
bool shouldPrint() {
  return debugInfoOn && timeToPrint;
}

//Raw and processed values from IMU including accelerations, and angular velocities
float q[4];      //hold q values
float euler[4];  
float hq[4];
float *pq = &q[0];
float *peuler = &euler[0];
float rawValue[9];

double angularVelocityRoll = 0.0;
double angularVelocityPitch = 0.0;
double angularVelocityYaw = 0.0;

double angularVelocityRollDirect = 0.0;
double angularVelocityPitchDirect = 0.0;

double angularVelocityRollCalculated = 0.0;
double angularVelocityPitchCalculated = 0.0;

double thetaRollRad = 0.0;
double thetaPitchRad = 0.0;
double thetaDesiredRollRad = 0;
double thetaDesiredPitchRad = 0;

double lastRoll =0.0;
double lastPitch = 0.0;
double lastYaw = 0.0;

double curRoll = 0.0;
double curPitch = 0.0;
double curYaw = 0.0;

double mass = 1.0;//normally 1.2
double g = 9.8;
double radius = 0.28;

boolean disarmed = false;

double F1 = 0.0;
double F2 = 0.0;
double F3 = 0.0;
double F4 = 0.0;

double NewPWM0 = 0;
double NewPWM1 = 0;
double NewPWM3 = 0;

double target = 0;
// Max angle quadcopter is commanded when stick is all the way right or up.
const double MAX_ANGLE_RAD = 10.0*PI/180.0;


//PID Gains
double Gain = 0.105;
double PitchdGain = 0.0005*1.25;//1.25 
double RolldGain = 0.0005*1.1;//1.1
double YawGain = 0.2;
double YawDGain = 0.06;
double OurYaw = 0;

double maxthrust = 18.3;//18.3 ounces from  bphobbies datasheet
long long LastPrintTime = 0;
long long LastLoopTime = 0;

#define MOTOR_ARM_VALUE 771
#define MOTOR1_PIN 11

//Initializes speed controllers
void armSpeedControllers() 
{
  
  Motor_1.attach(11);
  Motor_1.write(MOTOR_ARM_VALUE);
    
  Motor_2.attach(9); 
  Motor_2.write(MOTOR_ARM_VALUE);
  
  Motor_3.attach(10);
  Motor_3.write(MOTOR_ARM_VALUE);
  
  Motor_4.attach(6); 
  Motor_4.write(MOTOR_ARM_VALUE);
}

// Set the FreeIMU object
FreeSixIMU my3IMU = FreeSixIMU();
int rcPins[] = {2,3,5,19,7};
volatile long long pwmPulseTime[] = {0,0,0,0,0};
enum Pwm_Channel_Index { THROTTLE = 0, ROLL = 1,YAW =2, PITCH=3 , KILLSWITCH = 4};
int MovingMeanPWM0[] = {0,0,0,0,0};
int MovingMeanPWM1[] = {0,0,0,0,0};
int MovingMeanPWM3[] = {0,0,0,0,0};

//Disables all motors
void disarmMotors() {
  disarmed = true;
  int disarmValue = 800;

  F1 = disarmValue;
  F2 = disarmValue;
  F3 = disarmValue;
  F4 = disarmValue;
              
  Motor_4.write(F4); 
  Motor_3.write(F3); 
  Motor_2.write(F2);
  Motor_1.write(F1);
}


double ComputeHeightError(){
  long heightCm = getDistanceInCm();
  
  const double MAX_ERR = 100.0;
  double heightError = target - heightCm;
  if (heightError < -MAX_ERR) {
    heightError = -MAX_ERR;
  } else if (heightError > MAX_ERR) {
    heightError = MAX_ERR;
  }
  if (shouldPrint()) {
    Serial.print("height, target, height error: ");
    Serial.print(heightCm);
    Serial.print(" ");
    Serial.print(target);
    Serial.print(" ");
    Serial.println(heightError);
  }
  return heightError;
}

double UltrasonicLiftMultiplier(){
  long currTimeMs = millis();
  if (currTimeMs > lastChirpTimeMs + 100) {
    sendUltrasonicChirp();
    lastChirpTimeMs = currTimeMs;
  }  
  updateDistance();
  double heightError = ComputeHeightError();
  const double trimThrottlePerc = 1.0;
  const double throttleChangePerc = 0.01; // Controls how aggressive we try to rise or fall
  return AryaMap(heightError, -100, 100, trimThrottlePerc - throttleChangePerc, trimThrottlePerc + throttleChangePerc); 
}

double ManualLiftMultiplier(){
  return 0.00175 * pwmPulseTime[THROTTLE] - 1.625;
}

boolean manualMode = true;
double ComputeLiftMultiplier(){
  if(manualMode){
    return ManualLiftMultiplier();
  } else {
    return UltrasonicLiftMultiplier();
  }
}

//Map function for doubles, default map function is float only
double AryaMap(double x, double x0, double x1, double y0, double y1)
{
  if(x1 != x0){
    return ((y1-y0)/(x1-x0))*(x-x0)+y0;
  }
  else
  {
    Serial.println("MAJOR ERROR, DIVIDING BY 0 IN ARYA MAxP!");
    return 0; 
  }
}
long long wasShort = -1;

//Handles PWM signal coming from throttle pin of reciever using interrupt pin
void interruptRoutineThrottle()
{
  static long long timerStartUs = 0; 
  long long tUs = micros();
  if(digitalRead(rcPins[THROTTLE]) == HIGH)
  {
    timerStartUs = tUs;
  }
  else
  {
    long long timePassedUs = tUs-timerStartUs;
    if(timePassedUs > 950 && timePassedUs < 2050){
      pwmPulseTime[THROTTLE] = timePassedUs;
    } else {
      wasShort = timePassedUs;
    }
  }
}

//Handles PWM signal coming from roll pin of reciever using interrupt pin
void interruptRoutineRoll()
{
  static long long timerStart = 0; 
  long long t = micros();
  if(digitalRead(rcPins[ROLL]) == HIGH)
  {
    timerStart = t;
  }
  else
  {
    pwmPulseTime[ROLL] = t-timerStart;
  }
}

//Handles PWM signal coming from pitch pin of reciever using interrupt pin
void interruptRoutinePitch()
{
  static long long timerStart = 0; 
  long long t = micros();
  if(digitalRead(rcPins[PITCH]) == HIGH)
  {
    timerStart = t;
  }
  else
  {
      pwmPulseTime[PITCH] = t - timerStart;
  }
}

const float a = 101; // signicand (2 digits), exponent(10 digits).
const double EPS = 1e-8;

void assertDoubleEq(const char* testName, double actual, double expected){
  if(abs(actual - expected)> EPS){
//    if (actual != expected) {
    Serial.print(testName);
    Serial.print(" failed. Actual: ");
    Serial.print(actual);
    Serial.print(" Expected:");
    Serial.print(expected);
  } else {
    Serial.println("success");
  }
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

//Helps get distance and adjusts for the angle of the quadcopter in these calculations
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
    if (debugInfoOn && shouldPrint) {
      Serial.print("ULTRASOUND OUTLIER: ");
      Serial.print(distInCm);
      Serial.print(", avgDist: ");
      Serial.println(getDistanceInCm());
      if (distInCm <= 400) {
        Serial.print("residual:  ");
        Serial.println(abs(distInCm - getDistanceInCm()));
      }
    }
  } else {
    inputNextDistance(distInCm);
  }
}

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  //Sets up remote controller/reciever interrupts
  Serial.println("attach 0");
  attachInterrupt(5, onUltrasoundChange, CHANGE);
  attachInterrupt(0, interruptRoutineThrottle,CHANGE);
  attachInterrupt(1, interruptRoutineRoll,CHANGE);
  attachInterrupt(4, interruptRoutinePitch,CHANGE);
  
  //Stalls program until throttle is pushed above midpoint
   
  while(pwmPulseTime[THROTTLE] < 1200){
    Serial.print("Throttle < 1200 ");
    Serial.println((int)pwmPulseTime[THROTTLE]);
    delay(500);
  }
  
  delay(5);
  my3IMU.init();
  delay(5);
  Serial.println("Calibrate");

  while(pwmPulseTime[THROTTLE] < 1700){
    if (wasShort != -1) {
      wasShort = -1;
      Serial.print("pwm was short: ");
      Serial.println((int) wasShort);
    }
    Serial.print("Throttle < 1700 ");
    Serial.println((int) pwmPulseTime[THROTTLE]);
    delay(500);
  }
  armSpeedControllers();
  
  // Give speed controllers enough time to arm
  Serial.print("Give speed controllers enough time to arm");
  delay(5000);
  disarmMotors();
}

void loop() {
  long currTimeMs = millis();
  if(currTimeMs - LastPrintTime > 300){
    timeToPrint = true;
    LastPrintTime = currTimeMs;
  }
  if (manualMode) {
    manualLoop(currTimeMs);
  } else {
    autoLoop(currTimeMs);
  }
  runController(currTimeMs);
  timeToPrint = false;
}

// Sets desired pitch and roll based upon the remote control joystick positions
void manualLoop(long currTimeMs) {
   // Remote stick is inverted for roll, so we flip it here
 thetaDesiredPitchRad = AryaMap(NewPWM3, 1000, 2000, -MAX_ANGLE_RAD, MAX_ANGLE_RAD);
 thetaDesiredRollRad = AryaMap(NewPWM1, 1000, 2000, MAX_ANGLE_RAD, -MAX_ANGLE_RAD);
 disarmed = pwmPulseTime[THROTTLE] < 1150;
}

enum SMState { DISARMED=0, LIFTOFF, LOITERING, LAUNCHING, STABILIZING, LANDING};
 
SMState state = DISARMED;
long autoTimerStartMs = 0;
void autoLoop(long currTimeMs) {
  thetaDesiredPitchRad = 0;
  thetaDesiredRollRad = 0;
  long timePassedMs = currTimeMs - autoTimerStartMs;
  // inside landing phase disarmed = height < 10cm || pwmPulseTime[THROTTLE] < 1150;
  disarmed = pwmPulseTime[THROTTLE] < 1150;
  long heightCm = getDistanceInCm();
  switch(state) {
    case DISARMED:
      if (pwmPulseTime[THROTTLE] > 1150) {
        target = 100; // Ascend to 2m
        state = LIFTOFF;
        Serial.print("Throttle is ");
        Serial.print((int)pwmPulseTime[THROTTLE]);
        Serial.println(". -> LIFTOFF");
      } else {
        disarmed = true;
      }
      break;
    case LIFTOFF:
      if (heightCm > target) {
        autoTimerStartMs = currTimeMs;
        state = LOITERING;
        Serial.println(". -> LOITERING");
      }
      break;
    case LOITERING:
      if (timePassedMs >= 10000) {
        // TODO: SET ROCKET PIN HIGH
        Serial.println("LAUNCHING ROCKET!");
        autoTimerStartMs = currTimeMs;
        state = LAUNCHING;
        Serial.println(". -> LAUNCHING");
      }
      break;
    case LAUNCHING:
      if (timePassedMs >= 1000) {
        autoTimerStartMs = currTimeMs;
        state = STABILIZING;
        Serial.println("SWITCH THE GAINS");
        // SWITCH THE GAINS
        Serial.println(". -> STABILIZING");
      }
      break;
    case STABILIZING:
      if (timePassedMs >= 5000) {
        autoTimerStartMs = currTimeMs;
        state = LANDING;
        Serial.println("STABILIZED");
        // SWITCH THE GAINS
        Serial.println(". -> LANDING");
      }
      break;
    case LANDING:
      if (heightCm < 10) {
        disarmed = true;
        Serial.println(". -> DISARMED");
        Serial.println(disarmed);
      } else {
        target = -(timePassedMs) / 50 + 200;
        if (timePassedMs > 15000) {
          disarmed = true;
        }
      }
      break;
    default: 
      break;
  }
}

const double cmToMillivolt = 4.9; // Datasheet says this is 4.9

long liftTimerPrevMs = 0;

double liftMultiplier = 0;

void runController(long currTimeMs) { 
  double DeltaTimeSec = (currTimeMs - LastLoopTime) / 1000.0;
  double tempYaw; // We realized their raw yaw estimates were not as good as doing a smoothed integral of the angular yaw rate.
  my3IMU.getValues(rawValue);
  quaternionToEuler(pq, peuler, &tempYaw, &thetaPitchRad, &thetaRollRad);
  my3IMU.getQ(q);
  LastLoopTime = currTimeMs; //Cameron: This used to be millis()
  
  double angVelYaw = rawValue[5]; //deg/sec
  OurYaw = angVelYaw * DeltaTimeSec + OurYaw; //deg

  //Uses a moving mean to account for electrical noise in the reciever PWM signal
  
  //Throttle
  MovingMeanPWM0[4] = MovingMeanPWM0[3];
  MovingMeanPWM0[3] = MovingMeanPWM0[2];
  MovingMeanPWM0[2] = MovingMeanPWM0[1];
  MovingMeanPWM0[1] = MovingMeanPWM0[0];
  MovingMeanPWM0[0] = pwmPulseTime[THROTTLE];
  NewPWM0 = (MovingMeanPWM0[4]+MovingMeanPWM0[3]+MovingMeanPWM0[2]+MovingMeanPWM0[1]+MovingMeanPWM0[0])/5;
  
  //Roll
  MovingMeanPWM1[4] = MovingMeanPWM1[3];
  MovingMeanPWM1[3] = MovingMeanPWM1[2];
  MovingMeanPWM1[2] = MovingMeanPWM1[1];
  MovingMeanPWM1[1] = MovingMeanPWM1[0];
  MovingMeanPWM1[0] = pwmPulseTime[ROLL];
  NewPWM1 = (MovingMeanPWM1[4]+MovingMeanPWM1[3]+MovingMeanPWM1[2]+MovingMeanPWM1[1]+MovingMeanPWM1[0])/5;
  
  //Pitch
  MovingMeanPWM3[4] = MovingMeanPWM3[3];
  MovingMeanPWM3[3] = MovingMeanPWM3[2];
  MovingMeanPWM3[2] = MovingMeanPWM3[1];
  MovingMeanPWM3[1] = MovingMeanPWM3[0];
  MovingMeanPWM3[0] = pwmPulseTime[PITCH];
  NewPWM3 = (MovingMeanPWM3[4]+MovingMeanPWM3[3]+MovingMeanPWM3[2]+MovingMeanPWM3[1]+MovingMeanPWM3[0])/5;
  
  //calculate pitch angular velocity  
  curPitch = thetaPitchRad * 57.29; 
  angularVelocityPitchCalculated = (curPitch - lastPitch) / .01;//0.01 is 10 milliseconds which is the frequency of the loop
  lastPitch = curPitch;
     
  //calculate roll angular velocity 
  curRoll = thetaRollRad * 57.29; 
  angularVelocityRollCalculated = (curRoll - lastRoll) / .01;//0.01 is 10 milliseconds which is the frequency of the loop
  lastRoll = curRoll;
  
    // Calculated Values
    Serial.print("Angular Velocity PITCH Calculated,");
    Serial.print(angularVelocityPitchCalculated);
    Serial.print("Angular Velocity ROLL Calculated,");
    Serial.print(angularVelocityRollCalculated); 
    
    // Direct IMU Values
    angularVelocityRollDirect = rawValue[3];
    angularVelocityPitchDirect = rawValue[4];
    Serial.print("Angular Velocity PITCH Direct,");
    Serial.print(angularVelocityPitchDirect);
    Serial.print("Angular Velocity ROLL Direct,");
    Serial.print(angularVelocityRollDirect); 


 // double desiredLiftMultiplier = ComputeLiftMultiplier();
 double liftMultiplier = ComputeLiftMultiplier();
  
 /* long liftTimerStartMs = millis();
  if(liftTimerStartMs-liftTimerPrevMs > 10){
    if(liftMultiplier < desiredLiftMultiplier){
      liftMultiplier += 0.001;
    //  Serial.println(" +++ ");
    }
    else{
      liftMultiplier -= 0.001;
    //  Serial.println(" --- ");
    }
    
    liftTimerPrevMs = liftTimerStartMs;
    
  }*/
  
  
  //Calculate motor forces based on pitch and pitch angular velocity using PID
  //intermediate variables to shorten equations
  double thetaTrimRollRad = 0.0/180*3.1415;
  double thetaTrimPitchRad = 0.0/180*3.1415;
  double TX = (Gain * (thetaDesiredRollRad + thetaTrimRollRad - thetaRollRad) - RolldGain * angularVelocityRoll);//X is Roll is motors 1&2
  double TY = (Gain * (thetaDesiredPitchRad + thetaTrimPitchRad - thetaPitchRad) - PitchdGain * angularVelocityPitch);//Y is Pitch is motors 3&4
  double TZ = -(YawGain * OurYaw + YawDGain * angVelYaw);
  double denominator = min(cos(thetaRollRad) * cos(thetaPitchRad), cos(PI / 6) * cos(PI / 6));
  double GravityComponent = (mass * g / denominator) * liftMultiplier;
     
  //Calculate motor forces based on roll and roll angular velocity using PID
  F1 = GravityComponent / 4 + TZ / 4 + TX / (2 * radius);
  F2 = GravityComponent / 4 + TZ / 4 - TX / (2 * radius);
  F3 = GravityComponent / 4 - TZ / 4 + TY / (2 * radius);
  F4 = GravityComponent / 4 - TZ / 4 - TY / (2 * radius);
  
  //conversion from newtons to ounces of force.
  F1 = F1 * 3.59;
  F2 = F2 * 3.59;
  F3 = F3 * 3.59;
  F4 = F4 * 3.59;  

  //scaling 18.3 to 1000
  F1 = F1 * 54.644 + 1000;
  F2 = F2 * 54.644 + 1000;
  F3 = F3 * 54.644 + 1000;
  F4 = F4 * 54.644 + 1000;
  
  if (shouldPrint()) {
     //Serial.print("   LiftMultiPlier ");
     //Serial.print(LiftMultiplier);
     /*
     Serial.print("   TZ ");
     Serial.print(TZ);
     Serial.print("   TX ");
     Serial.print(TX);
        Serial.print("   TY ");
     Serial.print(TY);
     Serial.print("   W_YAW ");
     Serial.print(angularVelocityYaw);
     //Serial.print("   Angular Velocity ");
     //Serial.print(angularVelocityPitch);
     //Serial.print("   Gain ");
     //Serial.print(Gain);
  
   //  Serial.print("   Angular Velocity ");
     //Serial.print(angularVelocityRoll);
     //Serial.print("   Gain ");
     //Serial.print(Gain);
     //Serial.print("  Time ");
     //Serial.print(millis()/1000);
     Serial.println("");
     Serial.print("   PR ");
     Serial.print(thetaPitchRad*57.29);
     Serial.print(" ");
     Serial.print(thetaRollRad*57.29);*/
     Serial.print("  liftMultiplier ");
     Serial.print(liftMultiplier);
  
   /*  Serial.print("start,dur,dist,trig ");
     Serial.print(ultrasoundStartTimeUs);
        Serial.print(" ");
     Serial.print(ultrasoundDuration);
        Serial.print(" ");
     Serial.print(distanceInCm());
        Serial.print(" ");
     Serial.println(interruptTriggered);*/
    /* Serial.print("   F1 = ");
     Serial.print(F1);
     Serial.print("   F2 = ");
     Serial.print(F2);
     Serial.print("   F3 = ");
     Serial.print(F3);
     Serial.print("   F4 = ");
     Serial.print(F4);*/
     /*Serial.print("   PRY ");
     Serial.print(thetaPitchRad*57.29);
     Serial.print(" ");
     Serial.print(thetaRollRad*57.29);
     Serial.print(" ");
     Serial.print(OurYaw);
     Serial.print("  pwmPulseTime Throttle  ");
     Serial.print((int)pwmPulseTime[THROTTLE]);
     Serial.print(" ");
     Serial.print("  pwmPulseTime Roll  ");
     Serial.print((int)pwmPulseTime[ROLL]);
     Serial.print(" ");
     Serial.print("  pwmPulseTime Pitch ");
     Serial.print((int)pwmPulseTime[PITCH]);
     Serial.print("  ");
     Serial.print("  thetaDesiredRollDeg ");
     Serial.print("  ");
     Serial.print(thetaDesiredRollRad*57.29);
     Serial.print("  ThetaDesiredPitchDeg ");
     Serial.print("  ");
     Serial.print(thetaDesiredPitchRad*57.29);*/
     Serial.println();
  }
   /*
   Serial.print("   Angular Velocity ");
   Serial.print(angularVelocityPitch);
   Serial.print("  lastPitch  ");
    Serial.print(lastPitch);
     Serial.print("  curPitch  ")
    Serial.print(curPitch);
    Serial.print("   thetaPitchRad ");
    Serial.println(thetaPitchRad*57.29);
  */
  
 //pwmPulseTime[THROTTLE] is the position of the throttle stick and if the throttle stick is below 1150 (roughly 1/4 power) then the motors turn off.
  if(disarmed){
     disarmMotors();
  } else {
    Motor_4.write(F4); 
    Motor_3.write(F3); 
    Motor_2.write(F2);
    Motor_1.write(F1);
  }
  delay(20);
}

//Converts quaternion output of IMU into standard euler angles
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
  *Pitch = -euler[2];
  *Roll = euler[1];
}
