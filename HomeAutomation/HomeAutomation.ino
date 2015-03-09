#include "LixRobot.h"
#include <SPI.h>
#include <Ethernet.h>
#include <VirtualWire.h>

#include <avr/wdt.h>

#include <Time.h> 
#include <EthernetUdp.h>

const int led_pin = 6;
const int transmit_pin = 12;
const int receive_pin = 4;
const int transmit_en_pin = 3;

boolean timerOn = false;
boolean lightOn = false;
Timer resendTimer(300000);

byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; //physical mac address
byte ip[] = { 
  192, 168, 1, 177 }; // ip in lan
byte gateway[] = {
  192, 168, 1, 1 }; // internet access via router
byte subnet[] = { 
  255, 255, 255, 0 }; //subnet mask
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

unsigned long clockSyncInterval= 86400000; //Synchronize the clock at this interval (milliseconds)

EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

String readString;


void setup(){
  //enable serial data print
  Serial.begin(9600);

  Serial.println("Start sync clock");
  syncClock();
  Serial.println("Done, sync clock");

  pinMode(led_pin, OUTPUT); //pin selected to control

  //start Ethernet
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();

  Serial.println("server LED test 1.0"); // so I can keep track of what is loaded}

  // Initialize radio frequency communication
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_setup(2000); // Bits per sec

}

void loop(){
  //if(millis() > clockSyncInterval-6000)softwareReboot();

  if (timeStatus() != timeNotSet) {
    //Serial.println(hour());
    //reset the clock
    if(hour() == 1 && minute() ==1)softwareReboot();

    if(hour() >= 20 && hour() < 22 ){
      if(!timerOn){
        turnOn();
        lightOn = true;
        timerOn = true;
      }
    }
    else{
      if(timerOn){
        turnOff();
        lightOn = false;
        timerOn = false;
      }
    }
  }
  else{
    if(millis() > clockSyncInterval-6000)softwareReboot();
  }

  //resend the signal to turn on or off the light, just incase the initial message was missed
  if(resendTimer.done()){
    if(lightOn){
      turnOn();
    }
    else{
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
          //Serial.print(c);
        }

        //if HTTP request has ended
        if (c == '\n') {

          Serial.println(readString); //print to serial monitor for debuging
          client.println("HTTP/1.1 200 OK"); //send new page
          client.println("Content-Type: text/html");
          client.println();
          client.println("<HTML>");
          client.println("<HEAD>");
          client.println("<meta name='apple-mobile-web-app-capable' content='yes' />");
          client.println("<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent' />");
          client.println("<TITLE>Home Automation</TITLE>");
          client.println("</HEAD>");
          client.println("<BODY>");
          client.println("<H1>Home Automation</H1>");
          client.println("<hr />");
          client.println("<br />");

          client.println("Dining Rm   ");

          //if(lightOn){
          client.println("<button style=\"font-size: 150%;cursor: pointer\" type=\"button\" value=\"Submit\" onclick=\"funTurnOn()\">TurnOn</button>"); 
          client.println("<button style=\"font-size: 150%;cursor: pointer\" type=\"button\" value=\"Submit\" onclick=\"funTurnOff()\">TurnOff</button>"); 
          //}
          /*else {
          client.println("<button style=\"color:#900;font-weight: bold;font-size: 150%\;cursor: pointer\" type=\"button\" value=\"Submit\" onclick=\"funTurnOn()\">TurnOn</button>"); 
          client.println("<button style=\"font-size: 150%\;cursor: pointer\" type=\"button\" value=\"Submit\" onclick=\"funTurnOff()\">TurnOff</button>"); 
          }*/

          client.println("<script>");
          client.println("function funTurnOn() {window.location.href=\"/?lighton\";}");
          client.println("function funTurnOff() {window.location.href=\"/?lightoff\";}");
          client.println("</script>");

          client.println("</BODY>");
          client.println("</HTML>");
          delay(1);
          //stopping client
          client.stop();
          ///////////////////// control arduino pin
          if(readString.indexOf("?lighton") >0)//checks for on
          {
            turnOn();
            lightOn = true;
          }
          else{
            if(readString.indexOf("?lightoff") >0)//checks for off
            {
              turnOff();
              lightOn = false;
            }
          }
          //clearing string for next read
          readString="";
        }
      }
    }
  }

}

void turnOn(){
  digitalWrite(led_pin, HIGH);    
  send("Dining Rm1");
  Serial.println("Light turned on");
}
void turnOff(){
  digitalWrite(led_pin, LOW);    
  send("Dining Rm0");
  Serial.println("Light turned Off");
}

void send (char *message)
{
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
}

//* Synchrozing Arduino Clock *//
void syncClock(){
  //wait for one minute, just in case there was a brownout and the router needs to comeback up
  //delay(60000);
  setSyncInterval(clockSyncInterval);//to a large value
  if (Ethernet.begin(mac) == 0) {
    /* no point in carrying on, so do nothing forevermore:
     while (1) {*/
    Serial.println("Failed to configure Ethernet using DHCP");
    /*delay(10000);
     }*/
    Serial.println("Clock syn failed!");
    return;
  }
  Serial.print("IP number assigned by DHCP is ");
  Serial.println(Ethernet.localIP());
  Udp.begin(localPort);
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);  
  digitalClockDisplay();
  Udp.stop();
}


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
/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
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
  Serial.println("No NTP Response :-(");
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
  while(1)
  {
  }
}






