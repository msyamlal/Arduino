/**
 * @file distance_sensor_HCSR04.h
 * @brief distance sensor driver for distance sensor HC-SR04.
 * @author M. Syamlal
 */

#include "LixRobot.h"

namespace LixRobot
{
    class DistanceSensorHCSR04 : public DistanceSensorDriver
    {
    public:
        DistanceSensorHCSR04(int triggerPin, int echoPin, int maxDistance)
            : DistanceSensorDriver(maxDistance),
              sensor(triggerPin, echoPin)
        {
        }
        
        virtual long getDistance()
        {
            long distance = sensor.Ranging(CM);
            if (distance <= 0)
                return maxDistance;
            return distance;
        }
    private:
        Ultrasonic sensor;
    };
};
