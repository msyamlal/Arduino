/*
SimpleReceive for ATTiny84
 This sketch displays text strings received using VirtualWire
 Connect the Receiver data pin to Arduino pin 11
 */
#include <VirtualWire.h>
#include <avr/wdt.h>


byte message[10]; // a buffer to store the incoming messages
byte messageLength = 10; // the size of the message

const int receivePin = 7;
const int relay1Pin = 1;
const int ledPin = 0;

unsigned long rebootInterval= 86400000; //reboot Arduino at this interval (milliseconds)

void setup()
{
  //Serial.begin(9600);
  //Serial.println("Device is ready");

  pinMode(relay1Pin, OUTPUT);      // sets the digital pin as output
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output

  digitalWrite(relay1Pin, LOW);        // Prevents relays from starting up engaged

  // Initialize the IO and ISR
  vw_set_rx_pin(receivePin);
  vw_setup(2000); // Bits per sec
  vw_rx_start(); // Start the receiver
}
void loop()
{
  if(millis() > rebootInterval)softwareReboot();
  
  if (vw_get_message(message, &messageLength)) // Non-blocking
  {
    String command;
    for (int i = 0; i < messageLength; i++)
    {
      command = String(command + char(message[i]));
      //Serial.write(message[i]);
    }
    //Serial.println();

    if(command == "Dining Rm1"){
      digitalWrite(ledPin, HIGH);   // sets the LED on
      digitalWrite(relay1Pin, HIGH);        //energize relay
    }
    else if(command == "Dining Rm0"){
      digitalWrite(ledPin, LOW);    // sets the LED off
      digitalWrite(relay1Pin, LOW);        // denergize relay
    }
  }
  
}

//------- software rooboot ------/
//Use watch dog to reset the board
void softwareReboot()
{
  wdt_enable(WDTO_15MS);
  while(1)
  {
  }
}


