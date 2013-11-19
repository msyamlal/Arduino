#include "Mobile.h"
#include <Wire.h>
//Arduino PWM Speed Control:
const int sensorAddress = 44;

const int speedPinA = 5;
const int dirPinA = 4;
const int speedPinB = 6;
const int dirPinB = 7;

int sp = 0;

const int waitTime = 4000;

//Arduino UNO board has two external interrupts:
// numbers 0 (on digital pin 2) and 1 (on digital pin 3).
const int tr = 10; //ticks per revolution
const float r = 3.25E-2; //wheel radius (m)
const float wb = 14.7E-2; //wheel base (m)
unsigned long counter[2] =  {
  0,0};

const float dmax = 100.0;
const float dmin = 10.0;

PIDController turnControl(0., 0., 0.);


unsigned long lastTime = 0;

Wheel lw, rw;
Mobile m;


void setup() {

  float kp = 2.0 /(wb * (dmax - dmin));
  turnControl.set(kp, 0.0, 0.0);
  lw.set(speedPinA, dirPinA, r, &counter[0]);
  rw.set(speedPinB, dirPinB, r, &counter[1]);
  m.set(lw, rw, wb);

  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);
}

void loop()
{
  /* if (Serial.available() > 0){
   sp  = Serial.read();
   sp = sp-48;
   }
   */
  sp = 5;
  Wire.requestFrom(sensorAddress, 1);    // request 1 byte from slave device #sensorAddress

  float d;
  while(Wire.available())    // slave may send less than requested
  { 
    int d_int = Wire.read(); // receive a byte as integer
    Serial.println(d_int);
    d = (float) d_int;
  }
  float e = min(0., d - dmax);
  float om = turnControl.signal(e, 0.0001) * sp;// need a dt for non-zero kI or kD
  //Serial.println("   ");
  //Serial.println(d);
  //Serial.println(e);
  //Serial.println(sp);
  //Serial.println(om);
  m.setVelocity(sp, om);
  //m.stop();
  if(d < dmin) m.stop();
  delay(1000);
}










