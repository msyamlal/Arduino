/*
  Mobile.cpp - Library for robot mobility control

  Created by M. Syamlal May 17, 2013.
  References:
    Magnus Egerstedt, Control of Mobile Robots, 2013
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Mobile.h"

//const double PI  =3.141592653589793238462;

Point::Point(float x, float y)
{
    X = x;
    Y = y;
}

float Point::calcDistance(Point p)
{
    return sqrt(pow((X - p.X), 2) + pow((Y - p.Y), 2)) ;
}

PointPolar::PointPolar(float d, float phi)
{
    D = d;
    Phi = phi;
}

/*Pose::Pose(Point p, float phi)
{
    P.X = p.X;
    P.Y = p.Y;
    Phi = phi;
}

float Pose::calcDistance(Point &p)
{
    return P.calcDistance(p);
}*/


Motor::Motor()
{
    stop();
}

Motor::Motor(int s, int d)
{
    speedPin = s;
    directionPin = d;
    pinMode(directionPin, OUTPUT);
    Motor();
    
}

void Motor::stop()
{
    analogWrite(speedPin, 0); //PWM speed control
}

float Motor::setSpeed(float v)
{
    const int maxSpeed = 255;
    float v1 = max(-255, min(255, v));
    if(v < 0.0)
    {
        digitalWrite(directionPin, HIGH);        
    }
    else
    {
        digitalWrite(directionPin, LOW);                
    }
    //Serial.print("Motor: v = ");
    //Serial.println(v1);
    analogWrite(speedPin, v1); //PWM speed control
    return v1;
}

Wheel::Wheel()
{
    stop();
}

void Wheel::set(int ep, int mp, float r, unsigned long* cp)
{
    myMotor = Motor(ep, mp);
    radius = r;
    pCounter = cp;
    countsPerRevolution = 10;
    Wheel();
}

void Wheel::setVelocity(float v)
{
    angularVelocity = myMotor.setSpeed(v);
}


float Wheel::getVelocity(){
    return angularVelocity;
}

float Wheel::senseVelocity(){
    senseRevolutionsandTime();    
    return 2.0 * PI * deltaRevolutions * 1000./deltaTime;
}

float Wheel::senseDistance(){
    senseRevolutionsandTime();    
    return 2.0 * PI * deltaRevolutions * radius;
}

void Wheel::senseRevolutionsandTime(){
    unsigned long currentTime = millis();
    unsigned long cC = *pCounter;
    *pCounter = 0;
        
    deltaRevolutions = cC/countsPerRevolution;
    deltaTime = (currentTime - lastTime);
    
    //Serial.print("Wheel: cC = ");
    //Serial.println(cC);
    
    lastTime = currentTime;
}

float Wheel::getRadius(){
    return radius;
}


void Wheel::stop(){
    myMotor.stop();    
}


Mobile::Mobile()
{
    stop();
}

void Mobile::set(Wheel &lw, Wheel &rw, float wb)
{
    wheelLeft = lw;
    wheelRight = rw;
    wheelBase = wb;
    stop();
}

void Mobile::setVelocity(float v, float om)
{
    velRadial = v;
    velAngular = om;
    float vl = (2.0 * velRadial - velAngular * wheelBase)/ (2.0 * wheelLeft.getRadius());
    float vr = (2.0 * velRadial + velAngular * wheelBase)/ (2.0 * wheelRight.getRadius());
    wheelLeft.setVelocity(vl);
    wheelRight.setVelocity(vr);

    velX = velRadial * cos(direction);
    velY = velAngular * sin(direction);
}

void Mobile::updateState(){
    float dl = wheelLeft.senseDistance();
    float dr = wheelRight.senseDistance();
    float dc = (dl + dr)/2.0;
    positionX += dc * cos(direction);
    positionY += dc * sin(direction);
    direction += (dr - dl)/wheelBase;
}


void Mobile::goToGoal(float d, float phi){
    const float TOLERANCE = 1.0e-5;
    
    float e = 1.0E32;
    float gX = positionX + d * cos(direction + phi);
    float gY = positionY + d * sin(direction + phi);
    while (e < TOLERANCE){
        updateState();
        //e = mobilePose.calcDistance(gX, gY);
        
    }
    stop();
}

void Mobile::stop(){
    wheelLeft.stop();
    wheelRight.stop();    
}


PIDController::PIDController()
{
}

PIDController::PIDController(float ap, float ai, float ad)
{
    kP = ap;
    kI = ai;
    kD = ad;
}

void PIDController::set(float ap, float ai, float ad)
{
    kP = ap;
    kI = ai;
    kD = ad;
}

float PIDController::signal(float e, float dt)
{
    float e_dot = (e - old_e)/dt;
    Sum_e = Sum_e + e*dt;
    old_e = e;
    return kP * e + kI * Sum_e + kD * e_dot;
}
