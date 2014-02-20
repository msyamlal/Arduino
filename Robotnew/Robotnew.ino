/*select either master or slave mode. The master and slave codes can communicate only
 through the i2c bus, handled by the sendDataToMaster and receiveDataFromSlave routines.
 */
//#define ENABLE_MASTER_MODE
#define SLAVE_ADDRESS 44 //an arbitrary slave address

#define ENABLE_LOGGING

// constants
#define TOO_CLOSE 30                /**< distance to obstacle in centimeters */
#define CLOSE 60                    
#define TOO_FAR (200)               
#define TICKS_PER_REVOLUTION 10
#define WHEEL_RADIUS 3.25E-2 //wheel radius (m)
#define WHEEL_BASE  14.7E-2 //wheel base (m)
#define SPEED_FACTOR 5 //about 23 cm/s
//--------------------------------


//Make pin assignments
//{-------mmmmmmmmmmmmmmmmmmmmmmmmmmmm--------
#ifdef ENABLE_MASTER_MODE
#define PIN_DIR_A 4
#define PIN_SPEED_A 5
#define PIN_SPEED_B 6
#define PIN_DIR_B 7
#define PIN_BT_RX 16
#define PIN_BT_TX 17
//--------ssssssssssssssssssssssssssss--------
#else
#define PIN_TRIG 11
#define PIN_ECHO 12
#endif
//--------------------------------}

//Enable Device drivers

//{--------mmmmmmmmmmmmmmmmmmmmmmmmmmmm--------
#ifdef ENABLE_MASTER_MODE
// Motor controller:
//#define ENABLE_MOTOR_ADAFRUIT
#define ENABLE_MOTOR_DFR
// Remote control:
//#define ENABLE_REMOTE_CONTROL_DRIVER_BLUESTICK
//#define ENABLE_REMOTE_CONTROL_DRIVER_ROCKETBOT

//#define ENABLE_BLUETOOTH
//--------ssssssssssssssssssssssssssss--------
#else
// Distance sensor
//#define ENABLE_DISTANCE_SENSOR_NEWPING
#define ENABLE_DISTANCE_SENSOR_HCSR04
#endif
//--------------------------------}


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
//{--------mmmmmmmmmmmmmmmmmmmmmmmmmmmm--------
#ifdef ENABLE_MASTER_MODE
float turnAngle;
boolean turningBack = false;
Timer slaveTimer(100);

LixRobot::MotorDFR lm(PIN_SPEED_A, PIN_DIR_A);
LixRobot::MotorDFR rm(PIN_SPEED_B, PIN_DIR_B);
LixRobot::Wheel lw, rw;
LixRobot::Mobile m;


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

//--------ssssssssssssssssssssssssssss--------
#else
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

MovingAverage<unsigned int, 3> distanceAverage;


void countIntL()
{
  counter[0]++; //count the wheel encoder interrupts
}

void countIntR()
{
  counter[1]++; //count the wheel encoder interrupts
}

#endif
//--------------------------------}


void setup() {
  Serial.begin(9600);

  //{--------mmmmmmmmmmmmmmmmmmmmmmmmmmmm--------
#ifdef ENABLE_MASTER_MODE
  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  lw.set(&lm, WHEEL_RADIUS);
  rw.set(&rm, WHEEL_RADIUS);
  m.set(lw, rw, WHEEL_BASE);

  Wire.begin();        // join i2c bus (address optional for master)

  //--------ssssssssssssssssssssssssssss--------
#else
  //Initialize the moving average with a large value
  dFront = distanceAverage.add(TOO_FAR);
  dFront = distanceAverage.add(TOO_FAR);
  dFront = distanceAverage.add(TOO_FAR);

  Wire.begin(SLAVE_ADDRESS);                // join i2c bus with my address
  Wire.onRequest(sendDataToMaster); // register event that sends data to master
  attachInterrupt(0, countIntL, CHANGE);    //init the interrupt mode for the digital pin 2    
  attachInterrupt(1, countIntR, CHANGE);    //init the interrupt mode for the digital pin 3    
#endif
  //--------------------------------}
}

void loop()
{
  //{--------mmmmmmmmmmmmmmmmmmmmmmmmmmmm--------
#ifdef ENABLE_MASTER_MODE
  if(slaveTimer.done())receiveDataFromSlave();

  if(turningBack){
    if(m.finishTurn()) turningBack = false;
  }
  if(dFront < TOO_CLOSE) {
    if(!turningBack){
      turningBack = true;
      m.stopTurn();
      m.turn(-SPEED_FACTOR, turnAngle);
    }
  }
  else if (dFront < CLOSE){
    // check a random number from 0 to 1
    if(random(0, 2) == 0){
      turnAngle = -PI;
    }
    else{
      turnAngle = PI;
    }
    m.turn(SPEED_FACTOR, turnAngle/2.);
  }
  else{
    m.setVelocity(SPEED_FACTOR, 0.);
  }
  //delay(100);

  //--------ssssssssssssssssssssssssssss--------
#else
  if(transmitReady==false){
    dFront = distanceAverage.add(distanceSensor.getDistance());
    dFront = min(255, dFront);
    rpmL = (int)lwe.senseRPM();
    rpmR = (int)rwe.senseRPM();
    transmitReady = true; //this flag is set to false after transmission is complete
  }
#endif

  //--------------------------------}
}















