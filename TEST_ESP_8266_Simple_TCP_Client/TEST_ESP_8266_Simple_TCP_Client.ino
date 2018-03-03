/*
 *
 *	Client side for Simple Connection (Always Connected) 
 *  https://github.com/INEXTH/Arduino-ESP8266_libs/blob/master/examples/Simple_TCP_Client/Simple_TCP_Client.ino
 */
#include <ESP8266_TCP.h>

//ESP8266 Class
ESP8266_TCP microServer;
ESP8266_TCP microClient;
// Target Access Point
#define ssid         "SMD"
#define pass         "12345678"

// TCP Server IP and port
#define serverIP    "http://bb7882a2.ngrok.io" //"127.0.0.1"
#define serverPort  2000 //2000

// Connect this pin to CH_PD pin on ESP8266 (Our violet wire)
#define PIN_RESET    6

// Pin that connected to button to send any message
#define PIN_SEND     7

void setup()
{
  delay(3000);
  
  // Set pin for send command to input mode
  pinMode(PIN_SEND, INPUT);
  
  // We use Serial1 to interface with ESP8266 
  // and use Serial to debugging
  Serial.begin(9600); // 9600
  Serial1.begin(115200);
  //microServer.begin(&Serial1, &Serial, PIN_RESET);
  
  microClient.begin(&Serial1, &Serial, PIN_RESET);
  Serial.println("Setting up"); 
  //
  
  /* If your board has only 1 serial port
   * or you didn't need to debugging, try this.
   *
   * Serial.begin(115200);
   * wifi.begin(&Serial, PIN_RESET);
   *
   */

  /*
  // Check that ESP8266 client can access network
  if(microServer.test()) 
  {
	  //Connect to target Access Point
    String ip = connectToAccessPoint( &microServer );
    // Open TCP Server on port and 30 seconds for connection timeout (Max 2880)
    microServer.openTCPServer(serverPort, 30); 
    Serial.println("Connecting to IP: " + ip + ", Response: " + microServer.getMessage());    
  } 
  else 
  {
	  //TCP Server failed
    Serial.println("Client is unable to connect to AP Check module connection and restart to try again..."); 
    while(true);
  }
  */

  // Check that ESP8266 is available
  if(microClient.test()) 
  {
  // Connect to target Access Point
    String ip = connectToAccessPoint( &microClient );
    Serial.println("Client connected IP "+ ip); 
    microClient.connectTCP(serverIP, serverPort);
    delay(2000);
  } 
  else 
  {
  // Open TCP Client on port 8080 and 30 seconds for timeout
    Serial.println("Check module connection and restart to try again..."); 
    while(true);
  }
}

void loop()
{
// Check for any data has coming to ESP8266
//  int dataState = microServer.isNewDataComing(WIFI_SERVER);
//  
//  if(dataState != WIFI_NEW_NONE) {
//    if(dataState == WIFI_NEW_CONNECTED) {
//    // Connected with TCP Client Side
//      Serial.println("Status : Connected");
//    } else if(dataState == WIFI_NEW_DISCONNECTED) {
//    // Disconnected from TCP Client Side
//      Serial.println("Status : Disconnected");
//    } else if(dataState == WIFI_NEW_MESSAGE) {
//    // Got a message from TCP Client Side
//      Serial.println("ID : " + String(microServer.getId()));
//      Serial.println("Message : " + microServer.getMessage());
//      microServer.closeTCPConnection(0);
//    } else if(dataState == WIFI_NEW_SEND_OK) {
//    // Message transfer has successful
//      Serial.println("SENT!!!!");
//    } 
//  }

  //******************
   int dataState2 = microClient.isNewDataComing(WIFI_CLIENT);
  if(dataState2 != WIFI_NEW_NONE) {
    if(dataState2 == WIFI_NEW_CONNECTED) {
    // Connected with TCP Server Side
      Serial.println("===========> Client Connected");
    } else if(dataState2 == WIFI_NEW_DISCONNECTED) {
    // Disconnected from TCP Server Side
      Serial.println("===========> Client Disconnected");
    } else if(dataState2 == WIFI_NEW_MESSAGE) {
    // Got a message from TCP Server Side
      Serial.println("===========> Client Message : " + microClient.getMessage());
    } else if(dataState2 == WIFI_NEW_SEND_OK) {
    // Message transfer has successful
      Serial.println("===========> Client SENT!!!!");
    } 
  }
  //*******************
  
  //If we get Pin 7 low send message 
  if(!digitalRead(PIN_SEND)) {

    Serial.println("Button pressed!!!");
    delay(500);
    microClient.connectTCP(serverIP, serverPort);
    delay(500);
    Serial.println("Connect Message : " + microServer.getMessage());
    
    microClient.send( 0, "Hello Soumya");
    delay(500); 
    Serial.println("Send Message : " + microServer.getMessage());
    
//        if ( microServer.getMessage() == true ) {
//          Serial.println("Send success!!!");
//        } else {
//          Serial.println("Send failed!!! " + microClient.getMessage() );
//        }
    delay(1000);
    //microClient.closeTCPConnection();
  }

  
  // Auto connect to TCP Server Side when connection timeout
  if(microClient.getRunningState() == WIFI_STATE_UNAVAILABLE) {
    delay(500);
    microClient.connectTCP(serverIP, serverPort);
    delay(500);
  }
  
  delay(50);
  
}

// Access Point Connection Function that you can loop connect to Access Point until successful
String connectToAccessPoint( ESP8266_TCP *aWifi ) 
{
  String ip = "0.0.0.0";
  while(ip.equals("0.0.0.0")) 
  {
    ip = aWifi->connectAccessPoint(ssid, pass);
    if(!ip.equals("0.0.0.0")) 
    {
      break;
    } 
  }
  return ip;
}
