/*
 Mobile.cpp - Library for robot mobility control
 
 Created by M. Syamlal May 17, 2013.
*/


#ifndef Mobile_h
#define Mobile_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Point
{
public:
    float X;
    float Y;
    Point(float, float);
    float calcDistance(Point);
};

class PointPolar
{
public:
    float D;
    float Phi;
    PointPolar(float, float);
};

/*class Pose
{
public:
    Point P;
    float Phi;
    Pose(Point, float);
    float calcDistance(Point &);
};*/

class Motor
{
public:
    Motor();
    Motor(int, int);
    float setSpeed(float);
    void stop();    
    
private:
    int speedPin;
    int directionPin;
};

class Wheel
{
public:
    Wheel();
    void set(int, int, float, unsigned long*);
    void setVelocity(float);
    float getVelocity(); //wheel angular velocity
    float senseVelocity();
    float senseDistance();
    void senseRevolutionsandTime();
    float getRadius();
    void stop();
    
private:
    Motor myMotor;
    unsigned long* pCounter;
    int countsPerRevolution;
    unsigned long lastTime;
    float deltaRevolutions;
    float deltaTime;
    float angularVelocity;
    float radius;
};


class Mobile
{
  public:
    Mobile();
    void set(Wheel &, Wheel &, float);
    void setVelocity(float, float);//translational and angular velocity
    void updateState();
    void goToGoal(float, float);
    void stop();    

    private:
    Wheel wheelLeft;
    Wheel wheelRight;
    
    float wheelBase;
    
    float velRadial;
    float velAngular;
    
    float velX;
    float velY;
    
    float positionX;
    float positionY;
    float direction;
};

class PIDController
{
public:
    PIDController();
    PIDController(float, float, float);
    void set(float, float, float);
    float signal(float, float);
    
private:
    float kP;
    float kI;
    float kD;
    float Sum_e;
    float old_e;
};
#endif