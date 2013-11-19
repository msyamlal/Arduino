/**
 * @file remote_control_bluestick.h
 * @brief remote control driver for the BlueStick Android remote control app.
 * @author Miguel Grinberg
 */

#include "LixRobot.h"

namespace LixRobot
{
    class RemoteControlBluestick : public RemoteControlDriver
    {
    public:
        /**
          * @brief Class constructor.
          */
        RemoteControlBluestick() : RemoteControlDriver(), lastKey(command_t::keyNone) {}

        virtual bool getRemoteCommand(command_t& cmd)
        {
            cmd.stop();
            cmd.key = command_t::keyNone;
            
            if (BTSerial.available() <= 0)
                return false; // no commands available
            char ch = BTSerial.read();
            switch (ch) {
                case '8': // up
                    cmd.goForward();
                    break;
                case '2': // down
                    cmd.goBack();
                    break;
                case '4': // left
                    cmd.turnLeft();
                    break;
                case '6': // right
                    cmd.turnRight();
                    break;
                case 'A': // function key #1
                case 'C':
                    cmd.key = command_t::keyF1;
                    break;
                case 'B': // function key #2
                case 'D':
                    cmd.key = command_t::keyF2;
                    break;
                case 'E': // function key #3
                    cmd.key = command_t::keyF3;
                    break;
                case 'F': // function key #4
                    cmd.key = command_t::keyF4;
                    break;
                default:
                    break;
            }
            if (cmd.key != command_t::keyNone && cmd.key == lastKey) {
                // repeated key, ignore it
                return false; 
            }
            lastKey = cmd.key;
            return true;
        }
    
    private:
        command_t::key_t lastKey;
    };
};
