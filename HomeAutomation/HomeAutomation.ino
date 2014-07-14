#include <SPI.h>
#include <Ethernet.h>
#include <VirtualWire.h>

const int led_pin = 6;
const int transmit_pin = 12;
const int receive_pin = 4;
const int transmit_en_pin = 3;

byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; //physical mac address
byte ip[] = { 
  192, 168, 1, 177 }; // ip in lan
byte gateway[] = {
  192, 168, 1, 1 }; // internet access via router
byte subnet[] = { 
  255, 255, 255, 0 }; //subnet mask
EthernetServer server(80); //server port

String readString;


void setup(){
  pinMode(led_pin, OUTPUT); //pin selected to control
  //start Ethernet
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();
  //enable serial data print
  Serial.begin(9600);
  Serial.println("server LED test 1.0"); // so I can keep track of what is loaded}

  // Initialize radio frequency communication
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_setup(2000); // Bits per sec

}
void loop(){
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
          client.println("<link rel='stylesheet' type='text/css' href='http://homeautocss.net84.net/a.css' />");
          client.println("<TITLE>Home Automation</TITLE>");
          client.println("</HEAD>");
          client.println("<BODY>");
          client.println("<H1>Home Automation</H1>");
          client.println("<hr />");
          client.println("<br />");

          client.println("<a href=\"/?lighton\"\">Turn On Light</a>");
          client.println("<a href=\"/?lightoff\"\">Turn Off Light</a><br />");        

          client.println("</BODY>");
          client.println("</HTML>");
          delay(1);
          //stopping client
          client.stop();
          ///////////////////// control arduino pin
          if(readString.indexOf("?lighton") >0)//checks for on
          {
            digitalWrite(led_pin, HIGH);    // set pin 4 high
            send("Turn On");
            Serial.println("Led On");
          }
          else{
            if(readString.indexOf("?lightoff") >0)//checks for off
            {
              digitalWrite(led_pin, LOW);    // set pin 4 low
              send("Turn Off");
              Serial.println("Led Off");
            }
          }
          //clearing string for next read
          readString="";
        }
      }
    }
  }

}

void send (char *message)
{
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
}





