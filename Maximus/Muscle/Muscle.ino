/*

 ****************************************************************************
 ***************************                       **************************
 *                                                                          *
   [!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!]
   [!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!]
   [---      ----            !!!!!!!!!!!!!!!!!!             -----      ---]
   [---      ------             !!!!!!!!!!!!!!            -------      ---]
   [---      --------             !!!!!!!!!             ---------      ---]
   [---      --------!!             !!!!!             !!---------      ---]
   [---      ---------!!!            !!!            !!!----------      ---]
   [---      ----------!!!!           !           !!!!-----------      ---]
   [---      ---------------!                   !----------------      ---]
   [---      ------------------               -------------------      ---]
   [---      --------------------           ---------------------      ---]
   [---      -----------------------      -----------------------      ---]
   [---      -------------------------  -------------------------      ---]
   [---      ------------------------- --------------------------      ---]
   [---      ----------------------------------------------------      ---]
   [!!!!!!!!!!!!!!]                                        [!!!!!!!!!!!!!!]
   [!!!!!!!!!!!!!!]                                        [!!!!!!!!!!!!!!]
 *                                                                          *
 **************** MAXIMUS By NicoX [arch.smaitra@gmail.com] *****************

   Version: 2.0
 * ******************************************************************************************************************** *
 * ******************************************************************************************************************** *
*/

#include <ZerooneSupermodified_T3X_Wire.h>
#include <EEPROM.h>

#define Max_Absolute_Position 16384
unsigned int eeAddress = 0;


#define MOVE                           0
#define SET_HOME                      -100
#define HOME                          -200
#define STOP                          -800
#define RESET_ERROR                   -999



/*
     https://www.arduino.cc/en/Hacking/LibraryTutorial
     Create a heart beat to see if Teensy 3.6 is alive
     We will use the default LED in Teensy to produce the blink
     Teensy 3.x / Teensy LC have the LED on pin 13
*/

const int ledPIN = 13;  //Heart led pin
const int panActuatorId = 4;  //Pan Actuator ID, X
const int tiltActuatorId = 5; //Tilt Actuator ID, Y
 

const float actuatorSingleTickInDegree =  0.02197265625; //->  360/16384 (i4 bit contactless magnetic encoder)
const int readPin = 14;
ZerooneSupermodified motor( ZO_HW_SERIAL, readPin );

/*
   Setup
   setup() method runs once, when the sketch starts
*/
void setup() {

  // initialize the digital pin as an output.
  pinMode(ledPIN, OUTPUT);

  Serial.begin(9600); //57600
  Serial1.begin(57600);

motor.start(panActuatorId);
motor.resetErrors(panActuatorId);
motor.stop(panActuatorId);

motor.start(tiltActuatorId);
motor.resetErrors(tiltActuatorId);
motor.stop(tiltActuatorId);


  Serial.println();
  Serial.println(" ********* Set up done *********" );
  createHeartBeat(); //slowing down speed just to test done setup

}

/*
   Infinite Loop
   Method loop() runs over and over again, as long as the board is powered on
*/
void loop() {

  //Create a heart beat
  //createHeartBeat(); //slowing down speed

  //Perform command on servos
  performComand( motor );
    
}

//void loop() {
//
//  //Create a heart beat
//  //createHeartBeat(); //slowing down speed
//
//    //testActuatorSpan( motor, tiltActuatorId,  40, -40 );
//    //delay(400);
//    //testActuatorSpan( motor, panActuatorId,  30, -30 ); 
//
//    if ( Serial.available()) {
//        char aCommand = Serial.read();
//
//         if ( aCommand == '0' ) {
//            
//            /*
//             * When Start is received incremental position is reset [resetIncrementalPosition(actuatorId) ],
//             * memory is initialized and the controller enters position control mode
//             * with position setpoint = 0.
//             */
//            motor.start( tiltActuatorId );
//            motor.start( panActuatorId );
//            motor.resetIncrementalPosition( tiltActuatorId );
//            motor.resetIncrementalPosition( panActuatorId );
//            motor.setProfiledAbsolutePositionSetpoint( tiltActuatorId, 0 );
//            motor.setProfiledAbsolutePositionSetpoint( panActuatorId, 0 );
//            motor.broadCastDoMove();
//        }
//
//        if ( aCommand == '2' ){
//            motor.setAbsolutePositionSetpoint( tiltActuatorId, (0) );
//            motor.setAbsolutePositionSetpoint( panActuatorId, (0) ); 
//            motor.broadCastDoMove();
//        }
//        
//        if ( aCommand == '4' ){
//            //motor.setRelativePositionSetpoint( tiltActuatorId, (40/ actuatorSingleTickInDegree) );
//            //motor.setRelativePositionSetpoint( panActuatorId, (30/ actuatorSingleTickInDegree) );
//            motor.setAbsolutePositionSetpoint( tiltActuatorId, (40/ actuatorSingleTickInDegree) );
//            motor.setAbsolutePositionSetpoint( panActuatorId, (30/ actuatorSingleTickInDegree) ); 
//            motor.broadCastDoMove();
//        }
//        
//        if ( aCommand == '5' ){
//            //motor.setRelativePositionSetpoint( tiltActuatorId, (-40/ actuatorSingleTickInDegree) );
//            //motor.setRelativePositionSetpoint( panActuatorId, (-30/ actuatorSingleTickInDegree) );
//            motor.setAbsolutePositionSetpoint( tiltActuatorId, (-40/ actuatorSingleTickInDegree) );
//            motor.setAbsolutePositionSetpoint( panActuatorId, (-30/ actuatorSingleTickInDegree) ); 
//            motor.broadCastDoMove();
//            
//        }
//
//        if ( aCommand == '8' ){
//            motor.resetErrors( tiltActuatorId );
//            motor.resetErrors( panActuatorId );
//        }
//
//        if ( aCommand == '9' ){
//            motor.stop( tiltActuatorId );
//            motor.stop( panActuatorId );
//        }
//    }
//    
//}

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


////****************************************/////
double  ticks2deg(signed int ticks){
    return (double)360*ticks/(double)Max_Absolute_Position;
}

signed int  deg2ticks(double deg){
    return (signed int)deg*Max_Absolute_Position/360;
}


/*
 Methos will set actuator with default gain
 01Mechatronics default value P = 180, I = 20, D = 140
 */
void  setDefaultPIDGain ( ZerooneSupermodified motor, int actuatorId ) {
    
    motor.setPIDgainP(actuatorId, 180);
    motor.setPIDgainI(actuatorId, 20);
    motor.setPIDgainD(actuatorId, 140);
}

/*
 Methos will set actuator with a spacial PID gain for soft stiffness
 01Mechatronics default value P = 4, I = 26, D = 10
 TODO: Need to test with actuall graph out put .. need help of Ioannis
 */
void  setSoftStiffnessPIDGain ( ZerooneSupermodified motor, int actuatorId ) {
    
    motor.setPIDgainP(actuatorId, 4);  //Best value Kp = 180 >> 4
    motor.setPIDgainI(actuatorId, 26);  //Best value Ki = 20  >> 26
    motor.setPIDgainD(actuatorId, 10);  //Best value Kd = 140 >>10
}


void  setHomeAndStart( ZerooneSupermodified motor, int actuatorId ){
    motor.start( actuatorId );
    //int homeTickPosition = motor.getAbsolutePosition( actuatorId );      //Absolute position of zero point
    motor.resetIncrementalPosition( actuatorId );
    motor.setProfiledAbsolutePositionSetpoint(actuatorId, 0);
    motor.broadCastDoMove();
    motor.stop( actuatorId );
}

void  setStart( ZerooneSupermodified motor, int actuatorId ){
    motor.start(actuatorId);
}

void  setStop( ZerooneSupermodified motor, int actuatorId  ){
    motor.stop(actuatorId);
}



/*
 Methos will set actuator with its id and start position and end position
 We will call this method on actuator oject to set all actuated joints
 Params: id: int, startPostion: int, endPosition: int
 01Mechatronics default value P = 180, I = 20, D = 140
 */
void  testActuatorSpan ( ZerooneSupermodified motor,  int actuatorId, int spanCWInDegree, int spanCCWInDegree ) {
    
    int Kp = motor.getPIDgainP( actuatorId );                         //Kp
    int Ki = motor.getPIDgainI( actuatorId );                         //Ki
    int Kd = motor.getPIDgainD( actuatorId );                         //Kd
    
    int wakeUpAbsPosition = motor.getAbsolutePosition( actuatorId );  //Absolute position of zero point
    int curPosition = motor.getPosition( actuatorId );                //Curent position of zero point
    
    int curVelocity = motor.getVelocity( actuatorId );                //Unit ticks/sec
    int curCurrent = motor.getCurrent( actuatorId );                  //Unit mA
                           
    if ( Serial.available()) {
        char aCommand = Serial.read();
        uint16_t homeAbsolutePosition = EEPROMReadlong( eeAddress );        //Last known position of zero point
        
        Serial.print("\t Command = ");  Serial.println( aCommand );
        //Set Zero Position for the servo, can be anywhere but it considers as zero position
        if ( aCommand == '0' ) {
            
            /*
             * When Start is received incremental position is reset [resetIncrementalPosition(actuatorId) ],
             * memory is initialized and the controller enters position control mode
             * with position setpoint = 0.
             */
            motor.start(actuatorId);
            int homeTickPosition = motor.getAbsolutePosition( actuatorId );      //Absolute position of zero point
            homeAbsolutePosition = ceil( ( homeTickPosition + wakeUpAbsPosition ) / 2 );
            EEPROMWritelong(eeAddress, homeAbsolutePosition);                    //Writing first long.
            
            motor.resetIncrementalPosition( actuatorId );
            motor.setProfiledAbsolutePositionSetpoint(actuatorId, 0);
            motor.broadCastDoMove(); //?????? may this be called after all SMS set to its position
            
            
            Serial.println();
            Serial.print("\t Wake up homeTickPosition = ");
            Serial.println(homeAbsolutePosition);
            
            
            //Reading first long.
            uint16_t value = EEPROMReadlong(eeAddress);
            homeAbsolutePosition = value;
            Serial.print("*** EPROM Address = ");
            Serial.print(eeAddress);
            Serial.print("\t Last Known Abs Position = ");
            Serial.println(value);
            Serial.print(" ***");
            
            motor.stop(actuatorId);
        }
        
        //Start servo
        if ( aCommand == '1' ) {
            motor.start(actuatorId);
        }
        
        if ( aCommand == '2' ) {
            motor.moveToAbsolutePosition(actuatorId, 0 );
        }
        
        if ( aCommand == '3' ) {
            //motor.moveWithVelocity(actuatorId, 10000);
            
            /*motor.setPIDgainP(actuatorId, 2);  //Best value Kp = 180 >> 2
             motor.setPIDgainD(actuatorId, 200);  //Best value Kd = 140 >>200
             motor.setPIDgainI(actuatorId, 26);  //Best value Ki = 20  >> 26*/
        }
        
        if ( aCommand == '4' ){
            //motor.moveToAbsolutePosition(actuatorId, ( spanCWInDegree / actuatorSingleTickInDegree) );    //Clock Wise

            motor.setRelativePositionSetpoint( tiltActuatorId, (40/ actuatorSingleTickInDegree) );
            motor.setRelativePositionSetpoint( panActuatorId, (30/ actuatorSingleTickInDegree) );
            motor.broadCastDoMove();
        }
        
        if ( aCommand == '5' ){
            
            /*for (int i = 0; i< 50; i++ ) {
                motor.moveToAbsolutePosition(actuatorId, ( spanCCWInDegree / actuatorSingleTickInDegree ) );  //Couter Clock Wise
                delay(400);
                motor.moveToAbsolutePosition(actuatorId, ( spanCWInDegree / actuatorSingleTickInDegree) );    //Clock Wise
                delay(400);
            }
            motor.moveToAbsolutePosition(actuatorId, 0 );*/
            //motor.moveToAbsolutePosition(actuatorId, ( spanCCWInDegree / actuatorSingleTickInDegree) );    //Clock Wise

            motor.setRelativePositionSetpoint( tiltActuatorId, (-40/ actuatorSingleTickInDegree) );
            motor.setRelativePositionSetpoint( panActuatorId, (-30/ actuatorSingleTickInDegree) );
            motor.broadCastDoMove();
            
        }
        
        if ( aCommand == '6' ) {
            setSoftStiffnessPIDGain( motor, actuatorId );
        }
        
        if ( aCommand == '7' ) {
            setDefaultPIDGain( motor, actuatorId );
        }
        
        if ( aCommand == '8' ){
            motor.resetErrors(actuatorId);//send a command
        }
        
        if ( aCommand == '9' ){
            motor.halt(actuatorId);
        }
        if ( aCommand == 's' ){
            motor.stop(actuatorId);
        }
        
        Serial.println();
        Serial.print( " #[ " ); Serial.print( actuatorId, DEC );
        Serial.print( " ] Kp= " ); Serial.print( Kp, DEC );
        Serial.print( " Ki= " ); Serial.print( Ki, DEC );
        Serial.print( " Kd= " ); Serial.print( Kd, DEC );
        Serial.print( " Abs Position = " ); Serial.print( wakeUpAbsPosition, DEC );
        Serial.print( " Home Position = " ); Serial.print( homeAbsolutePosition, DEC );
        Serial.print( " <Ticks moved?>  = " ); Serial.print( curPosition, DEC );
        Serial.print( " Velocity = " ); Serial.print( curVelocity, DEC );
        Serial.print( " Current = " ); Serial.print( curCurrent, DEC );
        Serial.println();
    
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
    
    
    // motor.halt( actuatorId );
    // motor.profiledMoveToRelativePosition( actuatorId, 50 );
    // motor.profiledMoveToRelativePosition( actuatorId, -50 );
    
}


void  performComand ( ZerooneSupermodified motor ) {
    
    // if there's any serial available, read it:
    while (Serial.available() > 0) {
        
        //Look for the next valid integer in the incoming serial stream:
        //Get Command ID
        int commandID = Serial.parseInt();
        
        //Pan set
        int panMotorId = Serial.parseInt();
        int panGoalPositionValue = Serial.parseInt();

        //Tilt set
        int tiltMotorId = Serial.parseInt();
        int tiltGoalPositionValue = Serial.parseInt();
        
        Serial.println("********************START**********************************< ");
        Serial.print(" #Pan = "); Serial.println(panMotorId);
        Serial.print(" Goal = "); Serial.println(panGoalPositionValue);
        Serial.print(", #Tilt = "); Serial.println(tiltMotorId);
        Serial.print(" Goal = "); Serial.println(tiltGoalPositionValue);
        Serial.println(" >**********************END************************************");
        
        if ( commandID == SET_HOME ) {
            
            motor.start( panMotorId );
            motor.start( tiltMotorId );
            motor.resetIncrementalPosition( panMotorId );
            motor.resetIncrementalPosition( tiltMotorId );
            motor.setProfiledAbsolutePositionSetpoint( panMotorId, 0 );
            motor.setProfiledAbsolutePositionSetpoint( tiltMotorId, 0 );
            motor.broadCastDoMove();
            Serial.print("******* Home set *******");
        }
        else if ( commandID == HOME ){ 
            motor.setAbsolutePositionSetpoint( panMotorId, 0 );
            motor.setAbsolutePositionSetpoint( tiltMotorId, 0 );
            motor.broadCastDoMove();
            Serial.print("******* Back Home ********");
        }
        else if ( commandID == RESET_ERROR ){ 
            motor.resetErrors( panMotorId );
            motor.resetErrors( tiltMotorId );
            Serial.print("******* Reset Error ********");
        }
        else if ( commandID == STOP ) {
            motor.stop( panMotorId );
            motor.stop( tiltMotorId );
            Serial.print("******* Stopped ********");
        }
        else {
            //motor.profiledMoveToAbsolutePosition(motorId, deg2ticks(goalPositionValue)  );  //fed degree converted to tick
            motor.setProfiledAbsolutePositionSetpoint( panMotorId, deg2ticks(panGoalPositionValue) );
            delay(2);
            motor.setProfiledAbsolutePositionSetpoint( tiltMotorId, deg2ticks(tiltGoalPositionValue) ); 
            delay(2);
            motor.broadCastDoMove();
        }
        
        if ( !motor.getCommunicationSuccess() ) {
            Serial.println();
            Serial.print("Uff, Communication Warning :");
            Serial.print(motor.getWarning());
        }
        else {
            Serial.println();
            Serial.print("Wow, Communication Success.");
        }
    }

}

void simple_homing (int curActuatorId) {
  
  //command_complete_flag = false ;
  motor.broadcastStart();
  delay(15);
  int absolute_positions_homing = 16161;
  int absolute_positions_entry_positions = 0;
  int absolute_positions_ticks_positive = 0;
  int loop_count ;
  unsigned int entry_time = 0xffff & millis();
  
  // We save the entry positions in the array "absolute_positions_entry_positions"
  absolute_positions_entry_positions = motor.getAbsolutePosition(curActuatorId);
  delay(3);
  
  int ticks_to_go_positive = 0;
  ticks_to_go_positive = (16384 + absolute_positions_homing - absolute_positions_entry_positions) % 16384 ;
  absolute_positions_ticks_positive = ticks_to_go_positive;
  
  motor.setProfiledAbsolutePositionSetpoint( curActuatorId , absolute_positions_ticks_positive );
  delay(4);
  motor.broadCastDoMove();
  delay(3510); //wait for the servo to home
  delay(10);
  motor.broadcastStart();
  
  unsigned long homing_time = (0xffff + millis() - entry_time)%0xffff;
  Serial.print("homing done in ");
  Serial.print((int)homing_time);
  Serial.println(" milliseconds");
  delay(10);
  //command_complete_flag = true ;
}

// move CCW
void menu1_6() {
  
  motor.broadcastStart();
  delay(2);
  int q;
  for (q= start_ID ; q <= end_ID ; q++) {
    motor.setProfiledRelativePositionSetpoint(q, 65536) ;
  } // for all motors ends here
  delay(2);
  motor.broadCastDoMove();
  delay(100);

}

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void  EEPROMWritelong(int address, long value) {
    //Decomposition from a long to 4 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);
    
    //Write the 4 bytes into the eeprom memory.
    EEPROM.write(address, four);
    EEPROM.write(address + 1, three);
    EEPROM.write(address + 2, two);
    EEPROM.write(address + 3, one);
}


//This function will return a 4 byte (32bit) long from the eeprom
//at the specified address to address + 3.
long  EEPROMReadlong(long address) {
    //Read the 4 bytes from the eeprom memory.
    long four = EEPROM.read(address);
    long three = EEPROM.read(address + 1);
    long two = EEPROM.read(address + 2);
    long one = EEPROM.read(address + 3);
    
    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
////****************************************/////

