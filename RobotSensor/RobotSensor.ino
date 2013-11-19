#include "Mobile.h"
#include "Ultrasonic.h"
#include <Wire.h>

const int sensorAddress = 44;
const int trigPin = 11;
const int echoPin = 12 ;
boolean transmitReady = false;

//Arduino UNO board has two external interrupts:
// numbers 0 (on digital pin 2) and 1 (on digital pin 3).
const int tr = 10; //ticks per revolution
unsigned long counter[2] =  {
  0,0};

Ultrasonic ultrasonic(trigPin, echoPin);
int d_int;


void countIntL()
{
  counter[0]++; //count the wheel encoder interrupts
}

void countIntR()
{
  counter[1]++; //count the wheel encoder interrupts
}

void setup() {

  Wire.begin(sensorAddress);                // join i2c bus with my address
  Wire.onRequest(requestEvent); // register event

    Serial.begin(9600);
  //attachInterrupt(0, countIntL, CHANGE);    //init the interrupt mode for the digital pin 2    
  //attachInterrupt(1, countIntR, CHANGE);    //init the interrupt mode for the digital pin 3    
}

void loop()
{
  /* if (Serial.available() > 0){
   sp  = Serial.read();
   sp = sp-48;
   }
   */
  if(~transmitReady){
    long d = ultrasonic.Ranging(CM);
    d_int = (int) d;
    d_int = min(255, d_int);
    //Serial.print(d);
    //Serial.println(" cm");
    transmitReady = true;
    delay(100);
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  if(transmitReady){
    //Serial.println(d_int);
    Wire.write(d_int); // respond with message of 1 bytes as expected by master
    transmitReady = false;
  }
}








