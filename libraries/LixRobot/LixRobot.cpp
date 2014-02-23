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
    int speed = v/radius;
    pMotor->setSpeed(speed);
    angularVelocity = (float)pMotor->getSpeed();
}


float Wheel::getVelocity(){
    return angularVelocity*radius;
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
        
    float deltaTime = (currentTime - lastTime);
    float rpm = cC * 60. * 1000./((float)countsPerRevolution * deltaTime);
        
    //Serial.print("WheelEncoder::senseRPM: ");
    //Serial.println(rpm);
        
    *pCounter = 0;
    lastTime = currentTime;
    return rpm;
}

Mobile::Mobile()
{
    forcedMove = false;
}

void Mobile::set(Wheel &lw, Wheel &rw, float wb)
{
    wheelLeft = lw;
    wheelRight = rw;
    wheelBase = wb;
    stop();
}

/**
 * @brief set the mobile's translation and and angular velocity
 * @param v translational velocity, m/s
 * @param om angular velocity, rad/s
 * @param t time in miiliseconds to force the move
 */
    void Mobile::setVelocity(float v, float om, float t)
    {
        stopForcedMove();
        setVelocity(v, om);
        forcedMove = true;
        forcedMoveTimer.set(t);
    }
/**
 * @brief set the mobile's translation and and angular velocity
 * @param v translational velocity, m/s
 * @param om angular velocity, rad/s
*/
void Mobile::setVelocity(float v, float om)
{
    if(forcedMove){
        bool y = finishForcedMove();
    }
    else{
        velRadial = v;
        velAngular = om;
        float vl = (2.0 * velRadial - velAngular * wheelBase)/ (2.0);
        float vr = (2.0 * velRadial + velAngular * wheelBase)/ (2.0);
        wheelLeft.setVelocity(vl);
        wheelRight.setVelocity(vr);
        
        velX = velRadial * cos(direction);
        velY = velAngular * sin(direction);
    }
}

/**
 * @brief get the mobile's wheel translation velocity
 * @param wheel r, R, or l, L
*/
float Mobile::getVelocity(char w)
    {
        if(w == 'r' || w == 'R' ){
            return wheelRight.getVelocity();
        }
        else if (w == 'l' || w == 'L' ){
            return wheelLeft.getVelocity();
        }
    }

/**
 * @brief turn the mobile by the specified angle in radians
 * @param v translational velocity, m/s
 * @param theta angle to be turned, rad
*/
void Mobile::turn (float v, float theta)
{
    if(forcedMove){
        bool y = finishForcedMove();
    }
    else{
        if(abs(v) < 1E-5) return;
        forcedMove = true;
        float vl = 0., vr = 0.;
        if(theta > 0){
            vl = 1.5*v;
        }
        else{
            vr = 1.5*v;
        }
        unsigned long turningDeltaTime = 7.5e4 * abs(theta) * (float)wheelBase/abs(v);
        forcedMoveTimer.set(turningDeltaTime);
        wheelLeft.setVelocity(vl);
        wheelRight.setVelocity(vr);
    }
}
    
bool Mobile::finishForcedMove ()
/**
    * @brief if the turn finished stop turning the mobile
*/
{
      if(forcedMoveTimer.done()){
        forcedMove = false;
        wheelLeft.setVelocity(0.);
        wheelRight.setVelocity(0.);
        return true;
    }
    else{
        return false;
    }
}
    
void Mobile::stopForcedMove ()
/**
    * @brief stop turning the mobile; cancels a previous turn command
*/
{
    forcedMove = false;
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
