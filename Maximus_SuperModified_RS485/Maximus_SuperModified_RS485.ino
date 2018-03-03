#include <ZerooneSupermodified_T3X_Wire.h>

/*
   Create a heart beat to see if Teensy 3.6 is alive
   We will use the default LED in Teensy to produce the blink
   Teensy 3.x / Teensy LC have the LED on pin 13
*/
const int ledPIN = 13;

/*
   For Super Modified Servo or simply SMS read pin is attached to Teensy's pin 14
*/
const int readPIN = 14;

/*
   SMS motor ID
   ZO_HW_SERIAL is Serial for Teensy 3.6 not Serial1 or other
*/
const int motorID = 5;

ZerooneSupermodified motor(ZO_HW_SERIAL, readPIN);


/*
   Setup
   setup() method runs once, when the sketch starts
*/
void setup() {

  // initialize the digital pin as an output.
  pinMode(ledPIN, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(57600);

  Serial.println();
  Serial.print("Set up.");

  motor.setPIDgainP(motorID, 180);
  motor.setPIDgainI(motorID, 20);
  motor.setPIDgainD(motorID, 140);
    
  motor.resetErrors(motorID);//send a command
}

/*
   Infinite Loop
   Method loop() runs over and over again, as long as the board is powered on
*/
void loop() {

  int c;

  if ( Serial.available()) {
    c = Serial.read();

    if ( c == '1' )
      motor.start(motorID);

    if ( c == '2' ) /*Communication Warning :20*/
      motor.moveWithVelocity(motorID, 1000);

    if ( c == '3' ) /*Communication Warning :20*/
      motor.moveWithVelocity(motorID, -1000);

    if ( c == '4' )
      motor.resetErrors(motorID);

    if ( c == '5' )
      motor.broadcastStart();

    if ( c == '6' )
      motor.broadcastStop();

    if ( c == '7' )
      motor.stop(motorID);

    if ( !motor.getCommunicationSuccess() ) {
      Serial.println();
      Serial.print("Communication Warning :");
      Serial.print(motor.getWarning());
    }
    else {
      Serial.println();
      Serial.print("Communication Success.");
    }
  }

  //Create a heart beat
  createHeartBeat();

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

  Serial.println();
  Serial.print("Bip Bip ..");
}



/*
   Methos will set actuator with its id and start position and end position
   We will call this method on actuator oject to set all actuated joints
   Params: id: int, startPostion: int, endPosition: int
*/
void setActuatorSpan (int, int, int) {


  Serial.println();
  Serial.print( "Prepared " );

}
