#include "Ultrasonic.h"
Ultrasonic ultrasonic(12,13);

void setup() {
  Serial.begin(9600);
}

void loop()
{
  float d = ultrasonic.Ranging(CM);
  Serial.println(d);
  
  //Serial.write(analogRead(sensorPin)/4);
    
  delay(100);
}




