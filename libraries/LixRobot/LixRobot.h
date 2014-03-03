/**
 * @file LixRobot.h
 * @brief Robotics software header file.
 * @author M. Syamlal
 Initiated on May 17, 2013.
*/


#ifndef LixRobot_h
#define LixRobot_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include "logging.h"
#include "moving_average.h"
#include "timer.h"


namespace LixRobot
{
    
    //----------Device Driver Routines -------------------------
    class MotorDriver
    {
    public:
        /**
         * @brief Change the speed of the motor.
         * @param speed The new speed of the motor.
         *  Valid values are between -255 and 255.
         *  Use positive values to run the motor forward,
         *  negative values to run it backward and zero to stop the motor.
         * @author Miguel Grinberg
         */
        virtual void setSpeed(int speed) = 0;
        
        /**
         * @brief Return the current speed of the motor.
         * @return The current speed of the motor with range -255 to 255.
         */
        virtual int getSpeed() const = 0;
    };

    class DistanceSensorDriver
    {
    public:
        /**
         * @brief Class constructor.
         * @param distance The maximum distance in centimeters that needs to be tracked.
         * @author Miguel Grinberg
         */
        DistanceSensorDriver(long distance) : maxDistance(distance) {}
        
        /**
         * @brief Return the distance to the nearest obstacle in centimeters.
         * @return the distance to the closest object in centimeters
         *   or maxDistance if no object was detected
         */
        virtual long getDistance() = 0;
        
    protected:
        long maxDistance;
    };
        
    class RemoteControlDriver
        {
        public:
            /**
             * @brief abstract representation of a remote command.
             * @author Miguel Grinberg
             */
            struct command_t {
                enum key_t { keyNone, keyF1, keyF2, keyF3, keyF4 };
                int left;   /**< left side speed, between -255 and 255. */
                int right;  /**< right side speed, between -255 and 255. */
                key_t key;  /**< function key. */
                
                command_t() : left(0), right(0), key(keyNone) {}
                void goForward()
                {
                    left = right = 255;
                }
                void goBack()
                {
                    left = right = -255;
                }
                void turnLeft()
                {
                    left = -255;
                    right = 255;
                }
                void turnRight()
                {
                    left = 255;
                    right = -255;
                }
                void stop()
                {
                    left = right = 0;
                }
                void leftAndRightSliders(int l, int r)
                {
                    left = l;
                    right = r;
                }
                void forwardBackAndLeftRightSliders(int fb, int lr)
                {
                    left = fb - lr;
                    right = fb + lr;
                    if (left < -255)
                        left = -255;
                    else if (left > 255)
                        left = 255;
                    if (right < -255)
                        right = -255;
                    else if (right > 255)
                        right = 255;
                }
                void joystick(int x, int y)
                {
                    forwardBackAndLeftRightSliders(y, x);
                }
            };
            
            /**
             * @brief Class constructor.
             */
            RemoteControlDriver() {}
            
            /**
             * @brief Return the next remote command, if available.
             * @param cmd a reference to a command_t struct where the command
             *   information will be stored.
             * @return true if a remote command is available, false if not.
             */
            virtual bool getRemoteCommand(command_t& cmd) = 0;
    };

//----------------------------------------------------------
    
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
    
    class Wheel
    {
    public:
        Wheel();
        void set(MotorDriver*, float);
        void setVelocity(float);
        float getVelocity(); //wheel angular velocity
        float getRadius();
        void stop();
    
    private:
        MotorDriver* pMotor;
        float angularVelocity;
        float radius;
    };

    class WheelEncoder
    {
    public:
        WheelEncoder(unsigned long*, int);
        float senseRPM();
        
    private:
        unsigned long* pCounter;
        int countsPerRevolution;
        unsigned long lastTime;
    };

    class Mobile
    {
    public:
        Mobile();
        void set(Wheel &, Wheel &, float);
        void calibrateVel(float);
        void setVelocity(float, float, long);
        void setVelocity(float, float);
        float getVelocity(char);
        void turn(float, float);
        void stopForcedMove();
        bool forcedMoveFinished();
        void updateState();
        void goToGoal(float, float);
        void stop();

    private:
        Wheel wheelLeft;
        Wheel wheelRight;
    
        float wheelBase;
    
        float velRadial;
        float velAngular;
        float velScale;
        float velMaxActual;
    
        float velX;
        float velY;
    
        float positionX;
        float positionY;
        float direction;

        bool forcedMove;
        Timer forcedMoveTimer;
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
};
#endif