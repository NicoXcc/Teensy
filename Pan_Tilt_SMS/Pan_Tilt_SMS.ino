#include <ZerooneSupermodified_T3X_Wire.h>

/*
  * Roll - rotate about X-axis
  * Pitch - rotate about Y-axis
  * Yaw - rotate about Z-axis
*/
#define MOVE                           0
#define DEBUG                         -50
#define SET_HOME                      -100
#define HOME                          -200
#define STOP                          -800
#define RESET_ERROR                   -999

const int panActuatorId = 4;  //Pan Actuator ID, X
const int tiltActuatorId = 5; //Tilt Actuator ID, Y
int start_ID = panActuatorId ;
int end_ID = tiltActuatorId ;
const int Max_Absolute_Position         = 16384; //16384 (i4 bit contactless magnetic encoder)
const float actuatorSingleTickInDegree =  0.02197265625; //->  360/16384 (i4 bit contactless magnetic encoder)

const int ledPIN = 13;  //Heart led pin
const int readPin = 14;
ZerooneSupermodified motor( ZO_HW_SERIAL, readPin );

double  ticks2deg(signed int ticks){
    return (double)360*ticks/(double)Max_Absolute_Position;
}

signed int  deg2ticks(double deg){
    return (signed int)deg*Max_Absolute_Position/360;
}

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

/*
   Methos will set actuator with default gain
   01Mechatronics default value P = 180, I = 20, D = 140
*/
void setDefaultPIDGain ( int actuatorId ) {

  motor.setPIDgainP(actuatorId, 180);
  motor.setPIDgainI(actuatorId, 20);
  motor.setPIDgainD(actuatorId, 140);
}

/*
   Methos will set actuator with a spacial PID gain for soft stiffness 
   01Mechatronics default value P = 4, I = 26, D = 10
   TODO: Need to test with actuall graph out put .. need help of Ioannis
*/
void setSoftStiffnessPIDGain ( int actuatorId ) {
  
   motor.setPIDgainP(actuatorId, 4);  //Best value Kp = 180 >> 4
   motor.setPIDgainI(actuatorId, 26);  //Best value Ki = 20  >> 26
   motor.setPIDgainD(actuatorId, 10);  //Best value Kd = 140 >>10
}

void setup() {
  // put your setup code here, to run once:
  Serial.println("************Entering Setup ******************");
  // initialize the digital pin as an output.
  pinMode(ledPIN, OUTPUT);

  createHeartBeat(); //slowing down speed just to test done setup

  Serial.begin(57600);
  Serial1.begin(57600);
  Serial.println("************Setup Done ******************");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  performCommand();
  delay(500);
  
}

/*
 * SMS control methods 
 * 
*/

void checkAllCommunications() {
  
  int mId; 
  unsigned int entry_time = 0xffff & millis();
  for ( mId= start_ID ; mId <= end_ID ; mId++ ) {
    
    boolean read_bit = motor.getDigitalIOConfiguration( mId, 1 ) ;
    boolean error_flag = motor.getCommunicationSuccess();
    unsigned int warning ;
    if (!error_flag) 
        {warning = motor.getWarning();}
    if (!error_flag) 
        {Serial.print(warning); Serial.print(" ");}
    else 
        {
          Serial.println("---------------");
          int wakeUpAbsPosition = motor.getAbsolutePosition( mId );  //Absolute position of zero point
          Serial.print("Actuator Id [ ");
          Serial.print( mId ); 
          Serial.print(" ], Bit Value = ") ;
          Serial.print(read_bit); 
          Serial.print(", Horn Absolute position = ");
          Serial.print( wakeUpAbsPosition, DEC );
          Serial.println("  ---------------");
     }
    delay(3);
     
  }

  Serial.println( "***********************************************************************" );
  unsigned int execution_time = (0xffff + millis() - entry_time)%0xffff;
  Serial.print("Check communications time taken "); Serial.print(execution_time); Serial.print(" milliseconds ");
  Serial.println( "***********************************************************************" );
}


void resetAllActuatorsError() {
  
  int mId ; 
  unsigned int entry_time = 0xffff & millis();
  for ( mId= start_ID ; mId <= end_ID ; mId++ ) {
    motor.resetErrors( mId );
    boolean error_flag = motor.getCommunicationSuccess();
    unsigned int warning ;
    if (!error_flag) 
      {warning = motor.getWarning();}
    if (!error_flag) 
      {Serial.print(warning); Serial.print(" ");}
    else 
      {Serial.print( mId ); Serial.print(" ");}
    Serial.print("***** ID : "); Serial.print(mId); Serial.print(" *****");
    Serial.println();
    delay(3);
  }

  Serial.println( "***********************************************************************" );
  unsigned int execution_time = (0xffff + millis() - entry_time)%0xffff;
  Serial.print("Reset All Errors time taken "); Serial.print(execution_time); Serial.print(" milliseconds ");
  Serial.println( "***********************************************************************" );
}

void startAllAutuators() {
  unsigned int entry_time = 0xffff & millis();
  motor.broadcastStart();
  delay(15);
  Serial.println( "***********************************************************************" );
  unsigned int execution_time = (0xffff + millis() - entry_time)%0xffff;
  Serial.print("Global Start time taken "); Serial.print(execution_time); Serial.print(" milliseconds ");
  Serial.println( "***********************************************************************" );
}

void moveAllAutuators() {
  unsigned int entry_time = 0xffff & millis();
  motor.broadCastDoMove();
  delay(15);
  Serial.println( "***********************************************************************" );
  unsigned int execution_time = (0xffff + millis() - entry_time)%0xffff;
  Serial.print("Global Move time taken "); Serial.print(execution_time); Serial.print(" milliseconds ");
  Serial.println( "***********************************************************************" );
  delay(15);
}

void haltAllAutuators() {
  unsigned int entry_time = 0xffff & millis();
  motor.broadcastHalt();
  delay(15);
  Serial.println( "***********************************************************************" );
  unsigned int execution_time = (0xffff + millis() - entry_time)%0xffff;
  Serial.print("Global Halt time taken "); Serial.print(execution_time); Serial.print(" milliseconds ");
  Serial.println( "***********************************************************************" );
}

void stopAllAutuators() {
  unsigned int entry_time = 0xffff & millis();
  motor.broadcastStop();
  delay(15);
  Serial.println( "***********************************************************************" );
  unsigned int execution_time = (0xffff + millis() - entry_time)%0xffff;
  Serial.print("Global Stop time taken "); Serial.print(execution_time); Serial.print(" milliseconds ");
  Serial.println( "***********************************************************************" );
}

void performCommand() {

  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    unsigned int entry_time = 0xffff & millis();

    //Get Command ID
    int commandID = Serial.parseInt();
    
    //Pan set
    int panMotorId = Serial.parseInt();
    float panGoalPositionValue = Serial.parseFloat();
    
    //Tilt set
    int tiltMotorId = Serial.parseInt();
    float tiltGoalPositionValue = Serial.parseFloat();

    switch ( commandID ) {
      case DEBUG : {
        checkAllCommunications();
        
      }
      break;
      case SET_HOME : { //Set home and start so no stop command after words
        startAllAutuators();

        motor.resetIncrementalPosition( panMotorId );
        delay(4);
        motor.resetIncrementalPosition( tiltMotorId );
        delay(4);
        motor.setProfiledAbsolutePositionSetpoint( panMotorId, 0 );
        delay(4);
        motor.setProfiledAbsolutePositionSetpoint( tiltMotorId, 0 );
        delay(4);
      }
      break;
      case HOME : {        
        motor.setAbsolutePositionSetpoint( panMotorId, 0 );
        delay(4);
        motor.setAbsolutePositionSetpoint( tiltMotorId, 0 );
        delay(4);
        moveAllAutuators();
      }
      break;
      case RESET_ERROR : {
        resetAllActuatorsError();
      }
      break;
      case STOP : {
        stopAllAutuators();
      } 
      break;
      case MOVE: {
            startAllAutuators();
            motor.setAbsolutePositionSetpoint( panMotorId, deg2ticks(panGoalPositionValue) );
            delay(4);
            motor.setAbsolutePositionSetpoint( tiltMotorId, deg2ticks(tiltGoalPositionValue) ); 
            delay(4);
            moveAllAutuators();
      }
      break;
      default : {}
    }

    boolean error_flag = motor.getCommunicationSuccess();
    unsigned int warning ;
    if ( !error_flag ) {
        Serial.println();
        Serial.print("Uff, Communication Warning :");
        warning = motor.getWarning();
        Serial.print( warning );
    }
    else {
        Serial.println();
        Serial.print("Wow, Communication Success.");
    }
  
    Serial.println( "***********************************************************************" );
    unsigned int execution_time = (0xffff + millis() - entry_time)%0xffff;
    Serial.print("#Pan : "); Serial.print(panMotorId); Serial.print(" Goal Position : ");Serial.print(panGoalPositionValue);
    Serial.println();
    Serial.print("#Tilt : "); Serial.print(tiltMotorId); Serial.print(" Goal Position : ");Serial.print(tiltGoalPositionValue);
    Serial.println();
    Serial.print("Command : "); Serial.print(commandID) ;Serial.print(" performed, time taken "); Serial.print(execution_time); Serial.print(" milliseconds ");
    Serial.println( "***********************************************************************" );
  }
  
}

//void simple_homing (int curActuatorId) {
//
//  //Id [ 4 ], Horn Absolute position = 8942  ---------------
//  //Id [ 5 ], Horn Absolute position = 6777  ---------------
//
//  //command_complete_flag = false ;
//  motor.broadcastStart();
//  delay(15);
//  int absolute_positions_homing = (curActuatorId == 4) ?8942 :6777 ;
//  int absolute_positions_entry_positions = 0;
//  int absolute_positions_ticks_positive = 0;
//  int loop_count ;
//  unsigned int entry_time = 0xffff & millis();
//  
//  // We save the entry positions in the array "absolute_positions_entry_positions"
//  absolute_positions_entry_positions = motor.getAbsolutePosition(curActuatorId);
//  delay(3);
//  
//  int ticks_to_go_positive = 0;
//  ticks_to_go_positive = (16384 + absolute_positions_homing - absolute_positions_entry_positions) % 16384 ;
//  absolute_positions_ticks_positive = ticks_to_go_positive;
//  
//  motor.setProfiledAbsolutePositionSetpoint( curActuatorId , absolute_positions_ticks_positive );
//  delay(4);
//  motor.broadCastDoMove();
//  delay(3510); //wait for the servo to home
//  delay(10);
//  motor.broadcastStart();
//  
//  unsigned long homing_time = (0xffff + millis() - entry_time)%0xffff;
//  Serial.print("homing done in ");
//  Serial.print((int)homing_time);
//  Serial.println(" milliseconds");
//  delay(10);
//  //command_complete_flag = true ;
//}
