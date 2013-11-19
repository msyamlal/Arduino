/**
 * @file LixRobot.cpp
 * @brief Robotics software.
 * @author M. Syamlal
  Initiated on May 17, 2013.
  References:
    Magnus Egerstedt, Control of Mobile Robots, 2013
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "LixRobot.h"

//const double PI  =3.141592653589793238462;

namespace LixRobot
{
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

    
Wheel::Wheel()
{
}

void Wheel::set(MotorDriver* mp, float r)
{
    pMotor = mp;
    radius = r;
    Wheel();
}

void Wheel::setVelocity(float v)
{
    pMotor->setSpeed(v);
    angularVelocity = pMotor->getSpeed();
}


float Wheel::getVelocity(){
    return angularVelocity;
}


float Wheel::getRadius(){
    return radius;
}


void Wheel::stop(){
    pMotor->setSpeed(0);
    angularVelocity = 0;
}

    WheelEncoder::WheelEncoder(unsigned long* cp, int cpr){
        pCounter = cp;
        countsPerRevolution = cpr;
    }
    
    float WheelEncoder::senseRPM(){
        unsigned long currentTime = millis();
        unsigned long cC = *pCounter;
        
        float revolutions = cC/countsPerRevolution;
        float deltaTime = (currentTime - lastTime);
        
        //Serial.print("Wheel: cC = ");
        //Serial.println(cC);
        
        *pCounter = 0;
        lastTime = currentTime;
        return revolutions * 60. * 1000./deltaTime;
    }

    Mobile::Mobile()
{
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
    /*float dl = wheelLeft.senseDistance();
    float dr = wheelRight.senseDistance();
    float dc = (dl + dr)/2.0;
    positionX += dc * cos(direction);
    positionY += dc * sin(direction);
    direction += (dr - dl)/wheelBase;*/
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
};
