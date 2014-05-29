//Uncomment or comment the following line to select either master or slave mode.
#define ENABLE_MASTER_MODE

/*The master and slave codes can communicate only through the i2c bus,
 handled by the sendDataToMaster and receiveDataFromSlave routines.
 */
#define SLAVE_ADDRESS 44 //an arbitrary slave address
#define ENABLE_LOGGING

// constants
#define TOO_CLOSE 30                /**< distance to obstacle in centimeters */
#define CLOSE 60                    
#define TOO_FAR (200)               
#define TICKS_PER_REVOLUTION 10
#define WHEEL_RADIUS 3.25E-2 //wheel radius (m)
#define WHEEL_BASE  14.7E-2 //wheel base (m)
#define OMEGA_MAX 2.0/WHEEL_BASE //maximum angular velocity (scaled)
//The scaled velocity is in the range -1 <= v <= 1
//The maximum scaled omega = (2/L) * min (1-v, 1+v)
//--------------------------------


//Make pin assignments

#ifdef ENABLE_MASTER_MODE  //{Master block
#define PIN_DIR_A 4
#define PIN_SPEED_A 5
#define PIN_SPEED_B 6
#define PIN_DIR_B 7
#define PIN_BT_RX 13
#define PIN_BT_TX 12

#else                     //Slave block
#define PIN_TRIG 11
#define PIN_ECHO 12
#endif                   //End of Master-Slave block}

//Select Device drivers

#ifdef ENABLE_MASTER_MODE  //{Master block
// Motor controller:
//#define ENABLE_MOTOR_ADAFRUIT
#define ENABLE_MOTOR_DFR
// Remote control:
//#define ENABLE_REMOTE_CONTROL_DRIVER_BLUESTICK
//#define ENABLE_REMOTE_CONTROL_DRIVER_ROCKETBOT

#define ENABLE_BLUETOOTH

#else                     //Slave block
// Distance sensor
//#define ENABLE_DISTANCE_SENSOR_NEWPING
#define ENABLE_DISTANCE_SENSOR_HCSR04
#endif                   //End of Master-Slave block}

//Enable Device drivers

#ifdef ENABLE_BLUETOOTH
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(PIN_BT_RX, PIN_BT_TX);
#endif

#ifdef ENABLE_MOTOR_ADAFRUIT
#include <AFMotor.h>
#include "motor_adafruit.h"
#define LEFT_MOTOR_INIT 1
#define RIGHT_MOTOR_INIT 3
#endif

#ifdef ENABLE_MOTOR_DFR
#include "motor_dfr.h"
#endif

#ifdef ENABLE_DISTANCE_SENSOR_NEWPING
#include <NewPing.h>
#include "distance_sensor_newping.h"
#define DISTANCE_SENSOR_INIT 14,15,TOO_FAR
#endif

#ifdef ENABLE_DISTANCE_SENSOR_HCSR04
#include "Ultrasonic.h"
#include "distance_sensor_HCSR04.h"
#endif

#ifdef ENABLE_REMOTE_CONTROL_BLUESTICK
#include "bluestick_remote_control.h"
#define REMOTE_CONTROL_INIT
#endif

#ifdef ENABLE_REMOTE_CONTROL_ROCKETBOT
#include "rocketbot_remote_control.h"
#define REMOTE_CONTROL_INIT 10,50
#endif



#include "LixRobot.h"
#include <Wire.h>
int dFront, rpmL, rpmR;


//Initilizations
#ifdef ENABLE_MASTER_MODE  //{Master block
boolean initialize = true;
float turnAngle = PI;
boolean turningBack = false;
boolean turningToEscape = false;
boolean mobileStuck = false;
long escapeTime;
int escapeDistance;
Timer slaveTimer(100), mobileStuckTimer(1000);
int stuckCount = 0;

char btCommand ='g';

LixRobot::MotorDFR lm(PIN_SPEED_A, PIN_DIR_A);
LixRobot::MotorDFR rm(PIN_SPEED_B, PIN_DIR_B);
LixRobot::Wheel lw, rw;
LixRobot::Mobile m;


void maneuverRelease(){
  if(stuckCount < 3){
    stuckCount++;
  }
  else{
    stuckCount = 0;
    mobileStuck = true;
    BTSerial.println("Stuck");
    if(turningBack){
      m.stopForcedMove();
      turningBack = false;
      turningToEscape = false;
      BTSerial.println("Stopped turning");
      m.setVelocity(1.0, 0., 1000);
    }
    else{
      m.stopForcedMove();
      m.turn(-0.5, PI/10.);
    }
  }
}

// The master program calls this function to request data from slave. This should exactly match
// the function sendDataToMaster.
void receiveDataFromSlave()
{
  //the following order should be used to write out data in the routine "sendDataToMaster"
  Wire.requestFrom(SLAVE_ADDRESS, 1);    // request 1 byte from slave device #sensorAddress
  while(Wire.available())    // slave may send less than requested
  { 
    dFront = Wire.read(); // receive a byte as integer
  }
  Wire.requestFrom(SLAVE_ADDRESS, 1);    // request 1 byte from slave device #sensorAddress
  while(Wire.available())    // slave may send less than requested
  { 
    rpmL = Wire.read(); // receive a byte as integer
  }
  Wire.requestFrom(SLAVE_ADDRESS, 1);    // request 1 byte from slave device #sensorAddress
  while(Wire.available())    // slave may send less than requested
  { 
    rpmR = Wire.read(); // receive a byte as integer
  }
}


// calibrate the mobile velocity
float calibrateMobile(){ 
  m.setVelocity(1., 0.0);
  delay(1000);
  receiveDataFromSlave();
  delay(1000);
  receiveDataFromSlave();
  float vBoth = 0.5*(rpmR*WHEEL_RADIUS + rpmL*WHEEL_RADIUS) * 2. * PI/60.;
  return vBoth/4.;//4 is a calibration constant
}

#else                     //Slave block
// function that executes whenever data is requested by master. This should exactly match
// the function receiveDataFromSlave. This function is registered as an event, see setup()
int writeOrder = 0;
boolean transmitReady = false;
void sendDataToMaster()
{
  if(transmitReady){
    switch (writeOrder){
    case 0:
      Wire.write(dFront); // respond with message of 1 bytes as expected by master
      break;

    case 1:
      Wire.write(rpmL); // respond with message of 1 bytes as expected by master
      break;

    case 2:
      Wire.write(rpmR); // respond with message of 1 bytes as expected by master
    }
    writeOrder++;
    if(writeOrder > 2){
      writeOrder = 0;
      transmitReady = false; //No more data to transmit to master 
    }
  }
}

//Arduino UNO board has two external interrupts:
// numbers 0 (on digital pin 2) and 1 (on digital pin 3).
unsigned long counter[2] =  {
  0,0};

LixRobot::WheelEncoder lwe(&counter[1], TICKS_PER_REVOLUTION),
rwe(&counter[0], TICKS_PER_REVOLUTION );

LixRobot::DistanceSensorHCSR04 distanceSensor(PIN_TRIG, PIN_ECHO, TOO_FAR);

MovingAverage<long, 3> distanceAverage;


void countIntL()
{
  counter[0]++; //count the wheel encoder interrupts
}

void countIntR()
{
  counter[1]++; //count the wheel encoder interrupts
}

#endif                   //End of Master-Slave block}


void setup() {
  Serial.begin(9600);

#ifdef ENABLE_MASTER_MODE  //{Master block
  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  BTSerial.begin(9600);
  /*   if(BTSerial.isListening()){
   Serial.println("BT Slave is listening!");
   }
   */

  lw.set(&lm, WHEEL_RADIUS);
  rw.set(&rm, WHEEL_RADIUS);
  m.set(lw, rw, WHEEL_BASE);

  Wire.begin();        // join i2c bus (address optional for master)

  float v = calibrateMobile();
  m.calibrateVel(v);
  delay(1000);
  BTSerial.print("Calibrated v = ");
  BTSerial.println(v);


#else                     //Slave block
  //Initialize the moving average with a large value
  long dFrontLong = distanceAverage.add(TOO_FAR);
  dFrontLong = distanceAverage.add(TOO_FAR);
  dFrontLong = distanceAverage.add(TOO_FAR);

  Wire.begin(SLAVE_ADDRESS);                // join i2c bus with my address
  Wire.onRequest(sendDataToMaster); // register event that sends data to master

  attachInterrupt(0, countIntL, CHANGE);    //init the interrupt mode for the digital pin 2    
  attachInterrupt(1, countIntR, CHANGE);    //init the interrupt mode for the digital pin 3    
#endif                   //End of Master-Slave block}
}

void loop()
{
#ifdef ENABLE_MASTER_MODE  //{Master block
  /*  if (BTSerial.overflow()) {
   Serial.println("SoftwareSerial overflow!"); 
   } 
   */
  if (BTSerial.available() > 0) {
    btCommand = BTSerial.read();
    BTSerial.print(btCommand);
    BTSerial.print(" ");
    if(btCommand == 'g'){
      BTSerial.println("=> Go");
    }
    else if(btCommand == 's'){
      BTSerial.println("=> Stop");
    }
    else {
      BTSerial.println("=> Command not known!");
    }
  }
  if(btCommand == 's'){
    m.stop();
  }
  else {
    if(slaveTimer.done()){
      receiveDataFromSlave();

      if(mobileStuck){
        if(m.forcedMoveFinished()) {
          mobileStuck = false;
          BTSerial.println("Unstuck");
        }
      }
      else if(turningBack){
        if(turningToEscape){
          if(m.forcedMoveFinished()){
            turningBack = false;
            turningToEscape = false;
            m.stop();
            delay(1000);
            BTSerial.println("Stopped turning");
          }
        }
        else if(m.forcedMoveFinished()){
          m.stop();
          long t = millis() - escapeTime;
          m.setVelocity(0., -OMEGA_MAX, t);
          turningToEscape = true;
          BTSerial.print("Turning to escape: ");
          BTSerial.println(escapeDistance);
        }
        else{
          if(dFront > escapeDistance){
            if(dFront >=255){
              turningBack = false;
              turningToEscape = false;
              m.stopForcedMove();
              m.stop();
              BTSerial.println("Stopped turning. Escape route found.");
            }
            escapeTime = millis();
            escapeDistance = dFront;
            BTSerial.println(dFront);
          }
        }
      }
      else if(dFront < TOO_CLOSE) {
        turningBack = true;
        BTSerial.println("Turning back");
        escapeTime = millis();
        escapeDistance = dFront;
        m.stopForcedMove();
        //m.turn(0.0, 2.*PI);
        m.setVelocity(0., OMEGA_MAX, 2.*PI*250./OMEGA_MAX);
      }
     /* else if (dFront < CLOSE){
        // check a random number from 0 to 1
        if(random(0, 2) == 0){
          turnAngle = -PI;
        }
        else{
          turnAngle = PI;
        }
        m.turn(0.5, turnAngle/2.);
        //m.stop();
      }*/
      else{
        m.setVelocity(1., 0.);
        //m.stop();
      }
    }

    if(!mobileStuck){
      if(mobileStuckTimer.done()){
        //Release the mobile if it gets stuck
        //check whether the right wheel was supposed to be spinning
        if(abs(m.getVelocity('R')) > 0){
          //if so, is it really spinning?
          if(abs(rpmR) < 5){
            maneuverRelease();
          }
        }
        //check whether the left wheel was supposed to be spinning
        else if(abs(m.getVelocity('L')) > 0 ){
          //if so, is it really spinning?
          if(abs(rpmL) < 5){
            maneuverRelease();
          }
        }
        else{
          stuckCount = 0;
        }
      }
    }
  }

#else                     //Slave block
  if(transmitReady==false){
    long dFrontLong = min (255, distanceAverage.add(distanceSensor.getDistance()));
    dFront = (int) dFrontLong;
    rpmL = (int)lwe.senseRPM();
    rpmR = (int)rwe.senseRPM();
    transmitReady = true; //this flag is set to false after transmission is complete
  }
#endif                   //End of Master-Slave block}

}







































