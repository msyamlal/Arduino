#include "SerialWriter.h"

SerialWriter sw;

void setup() {
  sw.begin();
}
void loop() {
  sw.test();
  sw.print("int = ");
  int i = 12;
  sw.println(i);
  sw.print("float = ");
  int f = 1.7;
  sw.println(f); //value gets truncated to 1
  
  delay(1000);
}
