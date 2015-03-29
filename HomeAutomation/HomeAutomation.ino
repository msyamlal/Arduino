#include "LixRobot.h"
#include <SPI.h>
#include <Ethernet.h>
#include <VirtualWire.h>

#include <avr/wdt.h>

#include <Time.h>
#include <EthernetUdp.h>

#include <SD.h>

//#define LOGGING //turn on print statments

const int timeToTurnOn = 20;
const int timeToTurnOff = 22;

const int led_pin = 6;
const int transmit_pin = 12;
const int receive_pin = 4;
const int transmit_en_pin = 3;

// On the Ethernet Shield, CS is pin 4. It's set as an output by default.
// Note that even if it's not used as the CS pin, the hardware SS pin
// (10 on most Arduino boards, 53 on the Mega) must be left as an output
// or the SD library functions will not work.
const int cs_pin = 4;
const int ss_pin = 10;

boolean timerOn = false;
boolean lightOn = false;
Timer resendTimer(300000);
String s123, s456;

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
}; //physical mac address
byte ip[] = {
  192, 168, 1, 177
}; // ip in lan
byte gateway[] = {
  192, 168, 1, 1
}; // internet access via router
byte subnet[] = {
  255, 255, 255, 0
}; //subnet mask
EthernetServer server(80); //server port

// NTP Servers:
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov

//const int timeZone = 1;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

unsigned long clockSyncInterval = 86400000; //Synchronize the clock at this interval (milliseconds)

EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

String readString;

File myFile;

void setup() {
  //enable serial data print
  Serial.begin(9600);

#ifdef LOGGING
  Serial.println("Start sync clock");
#endif

   syncClock();
   
#ifdef LOGGING
   Serial.println("Done, sync clock");
#endif

  pinMode(led_pin, OUTPUT); //pin selected to control

  //start Ethernet
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();

#ifdef LOGGING
  Serial.println("server LED test 1.0"); // so I can keep track of what is loaded}
#endif

  // Initialize radio frequency communication
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_setup(2000); // Bits per sec


#ifdef LOGGING
  Serial.print("Initializing SD card...");
#endif
  pinMode(ss_pin, OUTPUT);

  if (!SD.begin(cs_pin)) {
#ifdef LOGGING
    Serial.println("initialization failed!");
#endif
    return;
  }
#ifdef LOGGING
  Serial.println("initialization done.");
#endif

}

void loop() {
  //if(millis() > clockSyncInterval-6000)softwareReboot();

  if (timeStatus() != timeNotSet) {
#ifdef LOGGING
    Serial.println(hour());
#endif
    //reset the clock
    if (hour() == 1 && minute() == 1)softwareReboot();

    if (hour() >= timeToTurnOn && hour() < timeToTurnOff ) {
      if (!timerOn) {
        turnOn();
        lightOn = true;
        timerOn = true;
      }
    }
    else {
      if (timerOn) {
        turnOff();
        lightOn = false;
        timerOn = false;
      }
    }
  }
  else {
    if (millis() > clockSyncInterval - 6000)softwareReboot();
  }

  //resend the signal to turn on or off the light, just incase the initial message was missed
  if (resendTimer.done()) {
    if (lightOn) {
      turnOn();
    }
    else {
      turnOff();
    }
  }

  // Create a client connection
  EthernetClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        //read char by char HTTP request
        if (readString.length() < 100) {
          //store characters to string
          readString += c;
#ifdef LOGGING
          Serial.print(c);
#endif
        }

        //if HTTP request has ended
        if (c == '\n') {

          ///////////////////// control arduino pin
          if (readString.indexOf("?lighton") > 0) //checks for on
          {
            turnOn();
            lightOn = true;
          }
          else {
            if (readString.indexOf("?lightoff") > 0) //checks for off
            {
              turnOff();
              lightOn = false;
            }
          }
#ifdef LOGGING
          Serial.println(readString); //print to serial monitor for debuging
#endif
          //clearing string for next read
          readString = "";
          
          // open the microSD file for reading the web page:
          myFile = SD.open("webpage.txt");
          if (myFile) {
#ifdef LOGGING
            Serial.println("webpage.txt");
#endif
            // read from the file until there's nothing else in it:
            while (myFile.available()) {
              char c = myFile.read();
              client.print(c);
            }
            myFile.close();
          } else {

#ifdef LOGGING
            Serial.println("error opening webpage.txt");
#endif
          }
          delay(1);
          //stopping client
          client.stop();
        }
      }
    }
  }

}

void turnOn() {
  digitalWrite(led_pin, HIGH);
  send("Dining Rm1");
#ifdef LOGGING
  Serial.println("Light turned on");
#endif
}
void turnOff() {
  digitalWrite(led_pin, LOW);
  send("Dining Rm0");
#ifdef LOGGING
  Serial.println("Light turned Off");
#endif
}

void send (char *message)
{
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
}

//* Synchrozing Arduino Clock *//
void syncClock() {
  //wait for one minute, just in case there was a brownout and the router needs to comeback up
  delay(60000);
  setSyncInterval(clockSyncInterval);//to a large value
  if (Ethernet.begin(mac) == 0) {
    /* no point in carrying on, so do nothing forevermore:
     while (1) {*/
#ifdef LOGGING
    Serial.println("Failed to configure Ethernet using DHCP");
#endif
    /*delay(10000);
     }*/
#ifdef LOGGING
    Serial.println("Clock syn failed!");
#endif
    return;
  }
  
#ifdef LOGGING
  Serial.print("IP number assigned by DHCP is ");
  Serial.println(Ethernet.localIP());
#endif

  Udp.begin(localPort);
  
#ifdef LOGGING
  Serial.println("waiting for sync");
#endif

  setSyncProvider(getNtpTime);
  //digitalClockDisplay();
  Udp.stop();
}

#ifdef LOGGING

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}


void printDigits(int digits){
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
#endif

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
#ifdef LOGGING
  Serial.println("Transmit NTP Request");
#endif
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
#ifdef LOGGING
      Serial.println("Receive NTP Response");
#endif
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
#ifdef LOGGING
  Serial.println("No NTP Response :-(");
#endif
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//------- software rooboot ------/
//Use watch dog to reset the board
void softwareReboot()
{
  wdt_enable(WDTO_15MS);
  while (1)
  {
  }
}








