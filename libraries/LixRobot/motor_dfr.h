/**
 * @file motor_dfr.h
 * @brief Motor device driver for the DFR motor shield.
 * @author M. Syamlal
 */

#include "LixRobot.h"

namespace LixRobot
{
    class MotorDFR : public MotorDriver
    {
    public:
        /*        
         * @brief Class constructor.
         * @param speed pin and direction pin numbers.
         */
        MotorDFR(int s, int d)
        : MotorDriver(), currentSpeed(0)
        {
            speedPin = s;
            directionPin = d;
            pinMode(directionPin, OUTPUT);
        }

        void setSpeed(int speed)
        {
            if (speed >= 0) {
                speed = min(255, speed);
                digitalWrite(directionPin, LOW);
                analogWrite(speedPin, speed); //PWM speed control
            }
            else {
                speed = max(-255, speed);
                digitalWrite(directionPin, HIGH);
                analogWrite(speedPin, -speed); //PWM speed control
            }
            currentSpeed = speed;
        }
        
        int getSpeed() const
        {
            return currentSpeed;
        }
        
    private:
        int speedPin;
        int directionPin;
        int currentSpeed;
    };
};
