
//Arduino UNO board has two external interrupts:
// numbers 0 (on digital pin 2) and 1 (on digital pin 3).
unsigned long counter[2] =  {0,0};

unsigned long lastTime = 0;

void countIntL()
{
    counter[0]++; //count the wheel encoder interrupts
}

void countIntR()
{
    counter[1]++; //count the wheel encoder interrupts
}

void setup() {

  Serial.begin(9600);
  attachInterrupt(0, countIntL, RISING);    //init the interrupt mode for the digital pin 2    
  attachInterrupt(1, countIntR, RISING);    //init the interrupt mode for the digital pin 3    
}

void loop()
{
  if(millis() > (lastTime + 5000))
  {
     lastTime = millis();
     Serial.print(counter[0]);
     Serial.print(" ");
     Serial.println(counter[1]);
     counter[0] = 0;
     counter[1] = 0;
  }
}


