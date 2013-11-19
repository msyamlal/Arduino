#include "Mobile.h"
//Arduino PWM Speed Control:
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

unsigned long counter[2] =  {
  0,0};

const float r = 3.25E-2; //wheel radius (m)
const float wb = 14.7E-2; //wheel base (m)

Wheel lw, rw;
Mobile m;

void setup() {
  lw.set(E1, M1, r, &counter[0]);
  rw.set(E2, M2, r, &counter[1]);
  m.set(lw, rw, wb);
}

void loop()
{
  //m.setVelocity(5., 30.*PI);//translational and angular velocities
  delay(100);
  m.stop();
  delay(4000);

}








