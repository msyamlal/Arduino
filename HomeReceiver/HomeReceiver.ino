/*
SimpleReceive
 This sketch displays text strings received using VirtualWire
 Connect the Receiver data pin to Arduino pin 11
 */
#include <VirtualWire.h>

byte message[VW_MAX_MESSAGE_LEN]; // a buffer to store the incoming messages
byte messageLength = VW_MAX_MESSAGE_LEN; // the size of the message

const int receivePin = 11;
const int relay1Pin = 7;
const int relay2Pin = 8;
const int ledPin = 5;

void setup()
{
  Serial.begin(9600);
  Serial.println("Device is ready");

  pinMode(relay1Pin, OUTPUT);      // sets the digital pin as output
  pinMode(relay2Pin, OUTPUT);      // sets the digital pin as output
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output

  digitalWrite(relay1Pin, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relay2Pin, HIGH);        // Prevents relays from starting up engaged

  // Initialize the IO and ISR
  vw_set_rx_pin(receivePin);
  vw_setup(2000); // Bits per sec
  vw_rx_start(); // Start the receiver
}
void loop()
{
  if (vw_get_message(message, &messageLength)) // Non-blocking
  {
    Serial.print("Received: ");
    String command;
    for (int i = 0; i < messageLength; i++)
    {
      command = String(command + char(message[i]));
      Serial.write(message[i]);
    }
    Serial.println();

    if(command == "Turn On"){
      digitalWrite(ledPin, HIGH);   // sets the LED on
      digitalWrite(relay1Pin, LOW);        //energize relay
    }
    else if(command == "Turn Of"){
      digitalWrite(ledPin, LOW);    // sets the LED off
      digitalWrite(relay1Pin, HIGH);        // denergize relay
    }
  }
}





