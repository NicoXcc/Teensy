/* Based on:
 * ====== ESP8266 Demo ======
 * (Updated Dec 14, 2014)
 * Ray Wang @ Rayshobby LLC
 * http://rayshobby.net/?p=9734
 * ==========================
 *
 * Modified by R. Wozniak
 * Compiled with Arduino 1.60 and Teensyduino 1.21b6
 * ESP8266 Firmware: AT21SDK95-2015-01-24.bin
 *
 * Change SSID and PASS to match your WiFi settings.
 * The IP address is displayed serial upon successful connection.
 */

#define BUFFER_SIZE 1024

#define SSID  "SMD"      // change this to match your WiFi SSID
#define PASS  "12345678"  // change this to match your WiFi password
#define PORT  "8080"           // using port 8080 by default

const int ledPIN = 13;  //Heart led pin
char buffer[BUFFER_SIZE];

String productId = "414390";
String productQty = "2";

int activateControls = false;
int isButtonAlreadyPressed = false;
int buttonPressedCount = 0;
int lastButtonPressedCount = 0;

/*
   Methos will create the Heart Beat pulses in LED
   Assuming 72 beats every 60 seconds
   So in one second number of beats: 72/60 = 1.2  beats per second.
   1000 = 1 second ; 1.2 second = 1200 = (500 + 250+ 250 + 200)
*/
void createHeartBeat () {

  digitalWrite(ledPIN, HIGH);   // set the LED on
  delay(500);                  // wait for 1/2 second

  digitalWrite(ledPIN, LOW);    // set the LED off
  delay(250);                  // wait for 1/4 second
  Serial.println( "..Heart Beat. Beep.." );
  digitalWrite(ledPIN, HIGH);   // set the LED on
  delay(250);                  // wait for 1/4 second

  digitalWrite(ledPIN, LOW);    // set the LED off
  delay(200);                  // wait for 1/5 second
  Serial.println( "..Beep Beep.." );
  Serial.println( "" );
}

// By default we are looking for OK\r\n
char OKrn[] = "OK\r\n";
byte wait_for_esp_response(int timeout, char* term=OKrn) {
  unsigned long t=millis();
  bool found=false;
  int i=0;
  int len=strlen(term);
  // wait for at most timeout milliseconds
  // or if OK\r\n is found
  while(millis()<t+timeout) {
    if(Serial1.available()) {
      buffer[i++]=Serial1.read();
      if(i>=len) {
        if(strncmp(buffer+i-len, term, len)==0) {
          found=true;
          break;
        }
      }
    }
  }
  buffer[i]=0;
  Serial.print(buffer);
  return found;
}

void setup() {

  //createHeartBeat();
  //Sense on Pin 7 in Teensy if active post data
  pinMode(7, INPUT);
  
  // assume esp8266 operates at 115200 baud rate
  // change if necessary to match your modules' baud rate
  Serial1.begin(115200);  // Teensy Hardware Serial port 1   (pins 0 and 1)
  Serial.begin(115200);   // Teensy USB Serial Port
  
  delay(5000);
  Serial.println("begin.");  
  setupWiFi();

  // print device IP address
  Serial.print("device ip addr: ");
  Serial1.println("AT+CIFSR");
  wait_for_esp_response(1000);
}

bool read_till_eol() {
  static int i=0;
  if(Serial1.available()) {
    buffer[i++]=Serial1.read();
    if(i==BUFFER_SIZE)  i=0;
    if(i>1 && buffer[i-2]==13 && buffer[i-1]==10) {
      buffer[i]=0;
      i=0;
      Serial.println("RX received [");
      Serial.print(buffer);
      Serial.println(" ]");
      return true;
    }
  }
  return false;
}

void loop() {
  if (activateControls == false) 
    return;
    
  int ch_id, packet_len;
  char *pb;  
  if(read_till_eol()) {
    if(strncmp(buffer, "+IPD,", 5)==0) {
      // request: +IPD,ch,len:data
      sscanf(buffer+5, "%d,%d", &ch_id, &packet_len);
      if (packet_len > 0) {
        // read serial until packet_len character received
        // start from :
        pb = buffer+5;
        while(*pb!=':') pb++;
        pb++;
        if (strncmp(pb, "GET /", 5) == 0) {
          wait_for_esp_response(1000);
          Serial.println("-> serve homepage");
          serve_homepage(ch_id);
        }
      }
    }
  }

  //If we get Pin 7 low post data
  if (digitalRead(7) == HIGH) {
    //Serial.println("Button is not pressed...");
  } else {
    
    //Post data now
    if (isButtonAlreadyPressed == false ) {
      Serial.println("Button pressed!!!");
      isButtonAlreadyPressed = true;
      buttonPressedCount += 1;
    }
  }


}

void serve_homepage(int ch_id) {
  
  String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\nRefresh: 300\r\n";
  String content="";
    content += "<!DOCTYPE html>";
    content += "<html>";
    content += "<head><title>Teensy 3.6 with ESP8266 Demo</title></head>";
    content += "<body  style='background-color:black;' >";
    content += " <h1 style='color:white;' > This is a webpage via TEENSY 3.6 Server & ESP8266EX 2.4 GHz! </h1><br/>";
    //Display dash button
    if (buttonPressedCount != lastButtonPressedCount ) {  
      content += " <br/> <h1 style='color:red;' > Dash Button Pressed </h1> <br/>  "; 
      content += " <dash_button><product_id>"+productId+"</product_id><product_qty>"+productQty+"</product_qty></dash_button>";
    } 
    content += " <p style='color:white;' > Teensy server uptime ";
    Serial.print("***************************************** UPTIME = ");
    Serial.println(millis());
    content += "<font style='color:red;'>";
    content += String(millis());
    content += " milliseconds </font> </p>";
    content += "</body>";
    content += "</html>";
    content += "<br />\n";       
    content += "\r\n";       

  header += "Content-Length:";
  header += (int)(content.length());
  header += "\r\n\r\n";
  Serial1.print("AT+CIPSEND=");
  Serial1.print(ch_id);
  Serial1.print(",");
  Serial1.println(header.length()+content.length());
  if(wait_for_esp_response(2000)) {
   //delay(100);
   Serial1.print(header);
   Serial1.print(content);
  } 
  else {
  Serial1.print("AT+CIPCLOSE=");
  Serial1.println(ch_id);
 }
 lastButtonPressedCount = buttonPressedCount = 0;
 isButtonAlreadyPressed = false;
}

void setupWiFi() {

  // turn on echo
  Serial1.println("ATE1");
  wait_for_esp_response(1000);
  
  // try empty AT command
  //Serial1.println("AT");
  //wait_for_esp_response(1000);

  // set mode 1 (client)
  Serial1.println("AT+CWMODE=3");
  wait_for_esp_response(1000); 
 
  // reset WiFi module
  Serial1.print("AT+RST\r\n");
  wait_for_esp_response(1500);

   //join AP
  Serial1.print("AT+CWJAP=\"");
  Serial1.print(SSID);
  Serial1.print("\",\"");
  Serial1.print(PASS);
  Serial1.println("\"");
  wait_for_esp_response(5000);

  // start server
  Serial1.println("AT+CIPMUX=1");
   wait_for_esp_response(1000);
  
  //Create TCP Server in 
  Serial1.print("AT+CIPSERVER=1,"); // turn on TCP service
  Serial1.println(PORT);
   wait_for_esp_response(1000);
  
  Serial1.println("AT+CIPSTO=30");  
  wait_for_esp_response(1000);

  Serial1.println("AT+GMR");
  wait_for_esp_response(1000);
  
  Serial1.println("AT+CWJAP?");
  wait_for_esp_response(1000);
  
  Serial1.println("AT+CIPSTA?");
  wait_for_esp_response(1000);
  
  Serial1.println("AT+CWMODE?");
  wait_for_esp_response(1000);
  
  Serial1.println("AT+CIFSR");
  wait_for_esp_response(5000);
  
  Serial1.println("AT+CWLAP");
  wait_for_esp_response(5000);

  Serial.println("************************** Teensy web server started *****************************");
  activateControls = true;
  
}
