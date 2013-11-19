//
//  SerialWriter.h
//  
//
//  Created by Madhava Syamlal on 10/3/13.
//
//

#ifndef SerialWriter_h
#define SerialWriter_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class SerialWriter {
public:
    //pass a reference to a Print object
//    void begin(HardwareSerial* hwPrint);
    void begin();
    void test();
    void print(const char*);
    void println(const char*);
    void print(int);
    void println(int);
    void print(float);
    void println(float);
private:
    HardwareSerial* printer;
};
#endif