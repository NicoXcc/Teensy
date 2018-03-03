
#include <ESP8266Teensy.h>

/*
 * Access Point details, once configured will be remembered.
*/
#define ssid         "SMD"
#define pass         "12345678"

/*
 * Server IP and port to connect as a client
*/
#define serverIP      "172.20.10.4" 
#define serverPort    8181
#define clientChannel 4

/*
 * Connect this PIN_RESET to CH_PD pin on ESP8266 (the violet wire)
 * PIN_SEND that connected to button to send any interrupt
*/
#define PIN_RESET    6
#define PIN_SEND     7

ESP8266Teensy eSP;
bool isTCPStarted = false;
bool isButtonEnabled = true;

/*
   Create a heart beat to see if Teensy 3.6 is alive
   We will use the default LED in Teensy to produce the blink
   Teensy 3.x / Teensy LC have the LED on pin 13
*/
const int ledPIN = 13;



/*################# Server Only ###############*/
// By default we are looking for OK\r\n
char OKrn[] = "OK\r\n";
#define BUFFER_SIZE 1024
char buffer[BUFFER_SIZE];

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

bool read_till_eol() {
  static int i=0;
  if(Serial1.available()) {
    buffer[i++]=Serial1.read();
    if(i==BUFFER_SIZE)  i=0;
    if(i>1 && buffer[i-2]==13 && buffer[i-1]==10) {
      buffer[i]=0;
      i=0;
      Serial.print(buffer);
      return true;
    }
  }
  return false;
}


void serve_homepage(int ch_id) {
  
  String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\nRefresh: 300\r\n";
  String content="";
  content += "<!DOCTYPE html>";
  content += "<html>";
  content += "<body>";
  content += " <h1> ESP Says Hello SHOMU  </h1> <br/>  ";  
  content += "</body>";
  content += "</html>";
  content += "<br />\n";       
  content += "\r\n";       

  header += "Content-Length:";
  header += (int)(content.length());
  header += "\r\n\r\n";
  
 String response = header + content;
 if (eSP.send( ch_id, response ) ) {
    createHeartBeat();

    //Close server
    eSP.closeTCPConnection( ch_id );
 }
  
}
/*################# Server Only ###############*/


//Setup ESP
void setup() {
  pinMode(ledPIN, OUTPUT);  // Set up heartbeat LED
  pinMode(PIN_SEND, INPUT); // Set pin as in input mode
  
  delay(3000);
  // We use Serial1 to interface with ESP8266 
  //and use Serial to debugging
  Serial.begin(115200);
  Serial1.begin(115200);

  eSP.start(&Serial1, &Serial, PIN_RESET, true);
  Serial.println("********* Setting up ***********");

  //Check that ESP8266 is available
  eSP.check();
  if( eSP.isCheckingSuccess() ) 
  {
    String espIPAddress = connectToAccessPoint( &eSP ); //Connect to AP and get ESP designated IP address
    Serial.println("ESP is Connected on IP: "+ espIPAddress );

    delay(50);
    eSP.setMux( WIFI_MUX_MULTI ); //We need to say esp that we need a server else we get Link type ERROR in TCP Connection

    /*################# Server Only ###############*/
    eSP.setMode(ESP_DUAL);
    if ( eSP.startTCPServer(80, 30) )  { //Start a TCP server
        createHeartBeat();      //Indication AP Connected
    }
    /*################# Server Only ###############*/
    
    //createHeartBeat();      //Indication AP Connected
  } 
  else 
  {
  // Open TCP Client on port 8080 and 30 seconds for timeout
    Serial.println("Check module connection and restart to try again..."); 
    while(true);
  }
  
}

void loop() {

  //If we get Pin 7 low send message 
  if( !digitalRead( PIN_SEND ) && (isButtonEnabled == true) ) {
  
    Serial.println("Button pressed!!!");
    isButtonEnabled = false;

    //Start TCP Connection
    isTCPStarted = eSP.startTCPConnection( clientChannel, serverIP, serverPort );
    isButtonEnabled = isTCPStarted; //If TCP server is not started the button should not be active

    if ( isTCPStarted == true ) {
      
//       String content = "data={\"productId\":1234,\"qty\":145}"; 
//       String postUri = "/sample/research/general_webservice.php?";
//      // prepare the data to be posted
//      String postRequest =
//          "POST " + postUri + " HTTP/1.1\r\n" +
//          "Host: " + serverIP + ":" + serverPort + "\r\n" +
//          "Accept: *" + "/" + "*\r\n" +
//          "Content-Length: " + content.length() + "\r\n" +
//          "Content-Type: application/x-www-form-urlencoded\r\n" +
//          "\r\n" +
//          content;
          
       String content = "buttonid=LowesConnectButton1124"; 
       String postUri = "/tapOnButton?";
       
      // prepare the data to be posted
      String postRequest =
          "POST " + postUri + " HTTP/1.1\r\n" +
          "Host: " + serverIP + ":" + serverPort + "\r\n" +
          "Accept: *" + "/" + "*\r\n" +
          "Content-Length: " + content.length() + "\r\n" +
          "Content-Type: application/json\r\n" +
          "\r\n" +
          content;
        
       if (eSP.send( clientChannel, postRequest ) ) {
          createHeartBeat();

          //Close server
          eSP.closeTCPConnection( clientChannel );
       } 
       
    } else {
      Serial.println("ESP did not start TCP server");
    }
    
    delay( 500 );
    isButtonEnabled = true;
      
  }

/*################# Server Only ###############*/
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
/*################# Server Only ###############*/
  
}

// Access Point Connection Function that you can loop connect to Access Point until successful
String connectToAccessPoint( ESP8266Teensy *aWifi ) 
{
  String ip = "0.0.0.0";
  while(ip.equals("0.0.0.0")) {
    //ip = aWifi->connectToAccessPoint(ssid, pass, ESP_CLIENT); //working code
    ip = aWifi->connectToAccessPoint(ssid, pass, ESP_DUAL);
    if(!ip.equals("0.0.0.0")) {
      break;
    } 
  }
  return ip;
}

/*
   Method will create the Heart Beat pulses in LED
   Assuming 72 beats every 60 seconds
   So in one second number of beats: 72/60 = 1.2  beats per second.
   1000 = 1 second ; 1.2 second = 1200 = (500 + 250+ 250 + 200)
*/
void createHeartBeat () {
  digitalWrite(ledPIN, HIGH);   // set the LED on
  delay(500);                  // wait for 1/2 second
  digitalWrite(ledPIN, LOW);    // set the LED off
  delay(250);                  // wait for 1/4 second
  digitalWrite(ledPIN, HIGH);   // set the LED on
  delay(250);                  // wait for 1/4 second
  digitalWrite(ledPIN, LOW);    // set the LED off
  delay(200);                  // wait for 1/5 second
  Serial.println("Bip Bip ..");
}
