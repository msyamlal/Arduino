//
//  SerialWriter.cpp
//  
//
//  Created by Madhava Syamlal on 10/3/13.
//
//

#include "SerialWriter.h"
#include "HardwareSerial.h"

#define baudRate 9600

void SerialWriter::begin() {
    printer = &Serial;
    if(printer) {
        printer->begin(baudRate);
    }
}

void SerialWriter::test() {
    if(printer) printer->println("Hello from SerialWriter!");
}

void SerialWriter::print(const char* input) {
    if(printer) printer->print(input);
}

void SerialWriter::println(const char* input) {
    if(printer) printer->println(input);
}

void SerialWriter::print(int input) {
    if(printer) printer->print(input);
}

void SerialWriter::println(int input) {
    if(printer) printer->println(input);
}

void SerialWriter::print(float input) {
    if(printer) printer->print(input);//prints floor(input)
}

void SerialWriter::println(float input) {
    if(printer) printer->println(input);//prints floor(input)
}