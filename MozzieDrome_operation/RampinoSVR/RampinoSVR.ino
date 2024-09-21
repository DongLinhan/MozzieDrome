#define REV "221216.1" 
/* 
 STILL IN DEVELOPMENT DC Motor section no tested yet
 REV 221216.1 adds special pinMode to set PWM frequency
 REV 221208.1 adds I2C support for TCA9548A MUX, STC31 Co2, SHTC3 Humidity, and Sfm3000 sensors, 
 REV 1.2 adds SPI for Adafruit DotStar LED strg Library
 REV 1.1 adds support for stepper limit switch polarity
 
 
 Rampino Server for MATLAB, Rick Hormigo, ZMBBI AIC Columbia university 2021                         
 state machine derived from 2012 arduinoio server by deGiampiero Campa for Matlab    
 This actual code supports current 2021 Arduino Mega board with Ramps 1.6 Plus boards 
 The communication arduinoio protocol has been modifified to minimize overhead
 acelerated to 250000 bauds, and extended to cover the modern stepper and DC motor drivers supported by Ramps  
 
 This server is meant to be used with the MATLAB rampino class. 
 The class is instantiated with a serial communitations port,
 what allows multiple instances as far each uses a separate port and Mega/Ramps combination
 Also it is possible to use a Mega board alone, if no extra electronics are requiered.
 This server can also be used along with other serial terminal softare that follows the protocol

Serial Protocol:
There is a state machine that process several states. The state s is calculated as: s=10*(val-48)
 Where val is what arrives to the serial port in decimal (or ASCII equivalent, a ex. a 88='X'):
 2 to 4 Bytes together are processed, First the mode (or command) where val is decimal 48 to 89,
 such as pinMode or AnalogRead, second the pin we are addresing where val is 99 to 166 depending on the command, 
 and third (if requiered) value for the pin that could be any decimal 0 to 255
 Then some commands can respond with a single byte or two for value.
 
At reset the state machine is s=-1 (idle) state while waits for a valid byte to arrive in the serial port. 
first byte that arrives is used to calculate next state s as s=10*(val-48) 
These are the possible command states:       
s=0  (val=48 or '0')is pinMode
s=10 (val=49 or '1')is digitaRead
s=20 (val=50 or '2')is digitalWrite;  
s=30 (val=51 or '3')is analogRead
s=40 (val=52 or '4')is analogWrite 
s=50 (val=53 or '5')is servoStatus
s=60 (val=54 or '6')is servoAttach and servoDetach 
s=70 (val=55 or '7')is servoRead
s=80 (val=56 or '8')is servoWrite
s=90 (val=57 or '9')is server type. This is Rampino=82 (Ascii 'R')
s=100(val=58 or ':')is tone  
s=110(val=59 or ';')is DCMotorAttach and DCMotorDetach
s=120(val=60 or '<')is DCMotorStatus
s=130(val=61 or '=')is DCMotorVelocity
s=170(val=65 or 'A')is stepperAttach and stepperDetach
s=180(val=66 or 'B')is stepperStatus
s=190(val=67 or 'C')is stepperSpeed
s=200(val=68 or 'D')is stepperStep
s=210(val=69 or 'E')is encoderAttach
s=220(val=70 or 'F')is encoderDetach
s=230(val=71 or 'G')is encoderRead
s=240(val=72 or 'H')is encoderReset
s=250(val=73 or 'I')is encoderDebounce
s=270(val=75 or 'K')is SPI serialLEDs (DotStar)
s=290(val=77 or 'M')is I2C Mux and Multiple sensors 
s=340(val=82 or 'R')is analogReference
s=400(val=88 or 'X')is roundTrip

Next (for most commands) the pin value is read as val=99 to 166 or ascii 'c' to 'ª'
to what is subtacted 97 to calculate the pin=2 to 69 (Mega pins)

Depending on the command there can be other additional states involving byte values need for execution
(see below individual commands)

A simple example to set pin 13 in output mode... 48-110-49 (in decimal sending) or "0n1" no quotations nor CR LF values used (ASCII string).

Mega harware resources used by rampino:
Serial0: Protocol interface
Timer0: Timing: delay(), millis, and micros(). AnalogWrite at pins 4 and 23 (Mega)
Timer1: 16bit used by Servo lib pins 2 to 11. AnalogWrite at pins 11 and 12 (Mega)
Timer2: DC Motor PWM at Any Pin library (DCM_PWM_AP). AnalogWrite at pins 9 and 10 (Mega)
Timer3: Tones (3rd instance) AnalogWrite at pins 2, 3, and 5 (Mega)
Timer4: Tones (2nd instance) AnalogWrite at pins 6, 7, and 8 (Mega) 
Timer5: Tones (1st instance) AnalogWrite at pins 44, 45, and 46 (Mega) 
*/

//Custom Libs
#include "rampino.h"
#include "BasicStepperDriver.h"
#include "DCM_PWM_AP.h"
#include "Tone.h"
#include "Servo.h"
//SPI
#include <SPI.h>    
#include <Adafruit_DotStar.h>
//I2C   
#include <Wire.h>
#include <SparkFun_STC3x_Arduino_Library.h> 
#include <SparkFun_SHTC3.h> 
#include <PWFusion_TCA9548A.h>
#include <SensirionI2CSfm3000.h>



//Dot Start Support (SPI only )
#define DATAPIN    51   
#define CLOCKPIN   52
#define NUMPIXELS 64 // Default Number of LEDs in strip
//Dot Start Commands (operations)
#define ENPIXS  49 //ascii '1', Free SPI pins
#define SHOW 50 //ascii '2',  Transmit set info to light LEDs
#define CLEAR 51 //ascii '3', Set all LEDs off
#define SETNUMPIXELS 52 //ascii '4', Set new Number of LEDs
#define SETBRIGHT 53 //ascii '5', Set LEDs bright 
#define SETPIX 60 //ascii '<', Set Pixel color
#define SETPIXS 61 //ascii '=', Set all LEDs off
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);


//I2C Sensors
//Operations
#define TCA9548_EN  1//Switch Enable with I2C address x70 is default
#define TCA9548_SEL 2//Swich Port
#define STC3X_EN    10//CO2 Sensor Enable (uses SHTC3 for compensation)  
#define STC3X_CAL   11//CO2 Sensor set actual Gas Concentration in porcentage as reference for Calibration
#define STC3X_STMP  12//CO2 Sensor set current temperature for compensation
#define STC3X_SHUM  13//CO2 Sensor set current humidity for compensation
#define STC3X_SPRE  14//CO2 Sensor set current or known enviroment Pressure for compensation
#define STC3X_RDT   15//CO2 Sensor Reads CO2 Temperature
#define STC3X_RDG   16//CO2 Sensor Reads CO2 Gas Concentration in porcentage 
#define SHTC3_EN    20//Humidity Sensor Enable 
#define SHTC3_RDH   21//Humidity Sensor Reads Humidity Percentage
#define SHTC3_RDT   22//Humidity Sensor Reads Temperature
#define SFM3000_EN  30//GAS FLOW Sensor Enable 
#define SFM3000_RD  31//GAS FLOW Sensor Reads Flow in lpm
TCA9548A tca9548aMux;
STC3x stc3xSensor;
SHTC3 shtc3Sensor;
SensirionI2CSfm3000 sfm3000Sensor;


// define internal analog reference for Arduino Mega
#define INTERNAL INTERNAL1V1

// Encoder structure                                  
typedef struct { int pinA; int pinB; int pos; int del;} Encoder;    
volatile Encoder Enc[3] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}};

// Servos                                       
Servo servo[12]; //Mega 48 max (12, 10 used in rampino) # of servos array instantiate

//DC Motors
DCM_PWM_AP dcMotor[5] = {{X_STEP, X_DIR, X_ENABLE},   //DC Motor class array instantiate. 
                        {Y_STEP, Y_DIR, Y_ENABLE}, 
                        {Z_STEP, Z_DIR, Z_ENABLE}, 
                        {E0_STEP, E0_DIR, E0_ENABLE}, 
                        {E1_STEP, E1_DIR, E1_ENABLE}}; 
byte dcAttached[]={0,0,0,0,0};    //Flags for DC motors atttached status: 0-dettached, 1-Attached, 2-Attached with direction flip 

//Stepper Motors
BasicStepperDriver sMotor[5]={{200, X_DIR, X_STEP, X_ENABLE},   //Stepper class array instantiate. 
                              {200, Y_DIR, Y_STEP, Y_ENABLE},   //Pins are activated outs on begin (attach)
                              {200, Z_DIR, Z_STEP, Z_ENABLE},   //de-activated as inputs on (detach)
                              {200, E0_DIR, E0_STEP, E0_ENABLE},//so could be overrided for diferent use.
                              {200, E1_DIR, E1_STEP, E1_ENABLE}};
uint8_t smAttached[]={0,0,0,0,0};    //Flags for stepper motors atttached status: 0-dettached, 1-Attached Software, 2-Attached Hardware 
uint8_t smAutoD[]={0,0,0,0,0};       //Auto disable (release motor) time in ms, 0 is no autodisable 
bool smAutoDSet[]={0,0,0,0,0};       //Auto disable (release motor) time is set 
uint8_t smLimitOn[]={0,0,0,0,0};   //used for fast homing limit function, 2-69 pin pending for homing (active low), 0 for homing off/done
//bool limitDetect[]={0,0,0,0,0};  //Limit switch asserted but still pending delayed disable.          
uint8_t microstepMode[]={1,2,4,8,16,32,32,16};  //A4988 -> {1,2,4,8,-,-,-,16}  DRV8825 {1,2,4,8,16,32,32,-}   
#define RELEASE_DELAY  50 //mS to release
bool limActive=LOW; //Assert Polarity of limit switch for motors 
// Microstepping pins
byte MODE_MS3[]={A9,40,42,A11,A12};
#define MODE_MS1      MOSI
#define MODE_MS2      SCK
#define N_RESET_SM    MISO 
//Stepper Operations(no pins)
#define DISABLE       48 //ascii '0'
#define STOP          49 //ascii '1'
#define BRAKING       50 //ascii '2'
#define ENABLE        56 //ascii '8' 
#define CHANGE_RUN    57 //ascii '9'
#define ENABLE_AD     58 //ascii ':'
#define CHANGE_RUN_AD 59 //ascii ';'




// Steppers Notes on microsepping modes, using Library  BasicStepperDriver by Laurentiu Badea: 
//Mod0 at 240->237rpm (optic measured) gives 790.6Hz*60 47436/mt /200= 237rpm --400 marginal max that measures 391 rpm max
//Mod1 at 240->235rpm gives 1550Hz *60 = 93000/mt /400 steps=232rpm
//Mod2 at 240->230rpm. gives 3068Hz*60= 184080/mt  /800 = 230.1 rpm
//Mod3 at 240->220rpm  gives 5860Hz *60= 351600/mt /1600 =220 rpm --400 marginal max that measures 340 rpm max
//Mod4 at 240->199.5rpm  gives 10620Hz *60= 637200/mt /3200 =199.1 rpm --500 marginal max that measures 358 rpm max
//Mod5 at 240->180 rpm  gives 19100Hz *60= 1146000/mt /6400 =179 rpm --500 marginal max that measures 184 rpm max
//NOTE: This libray doesn't block when moving, yet the main loop is blocked by the length of a step,
// and a step could be as short as 20uS (at 255RPM and X32 microstepping), but as long as 300mS for full steps at 1 RPM  

//TONES-----Max 4 posible tones simultaneously, implemente  3 pins only for DC motor compatibility.  --MEGA only. 
Tone tonePin[3]; // Simultaneus tones may easily conflict with other hardware use. This implimentation limits to timers 3, 4, and 5
uint8_t tonePinMap[]={0,0,0}; //Pins asigned to TonePin instances
//NOTE: The Tone.h lib currently can generate up to 65535Hz, but over 323767Hz the duration parameter must be 0, then control with stop
//NOTE: Frequencies can generate an undesired random mix if a frequency is time overlapped with another frequency on the same pin.
//Only 3 instances (one by timer) of the class are allowed in this Rampino implementation, the 4th will replace the first automatically.  
//Limitation: if used 4 pins, the 4th created instance (that corresponds to timer 2) can't handle frequency under 32Hz
 
void setup() {
  // initialize serial 
 // Serial2.begin(115200); //R&D Debug
 // Serial2.println("---START DEBUG SESSION---");                                   
  Serial.begin(115200); //up to 250000 Bauds that is multiple of 16M and available at Arduino IDE Monitor
  Serial.print(REV);  //Initial Charecter to indicate port has opened (This is important with slow drivers that take long to open the serial)
                       //I found that in slow computers the serial print/write may happen before the serial is efectivelly open, leading to serial block  
#if 0  //R&D DCM testing
  for (uint8_t i = 0; i < 5; i++)
  {
    dcMotor[i].begin();  //attach pins
  }
  dcMotor[4].setSpeed(1);    
  while(1){ 
    for(int8_t dc=100;dc>=0;dc--){
      for (uint8_t i=0; i < 4; i++){
          dcMotor[i].setVelocity(dc);
      }
      delay(20);
    }
  }
#endif

}

/*******************************************************************************************************************/
void loop() {
  
  static int  s   = -1;    // state     
  static uint8_t mt; //motor sequence in main loop                     
  static int  pin = 13;    // generic pin number             
  static int  enc = 0;     // generic encoder number         
  static int  motor = 0;  // generic motor number         
  static long sdgv =  0;  // Long serial digital value
  static uint8_t sdgv8;   // 8 bit serial digital value
  static uint16_t sdgv16;   // 16 bit serial digital value
  static unsigned int frequency = 0;  //Tone frequency in Hz, 0 to stop tone 
  static unsigned int duration = 0;   //Tone Duration in mS, 0 means keep playing until stop by frequency 0.
  static uint8_t toneInstCnt = 2; //Tone Pin Instance counter, 0 to 2, if more is requested, get overlapped to 0
  static int8_t toneInstIdx = -1;//Tone Pin Instance counter Index lo locate if pin is attached
  static uint8_t red, green, blue; // 8 bit serial bytes 

  union {
    float f; // float serial digital value
    byte a[4]; // float serial digital value array access
    uint32_t i; //32 bit integer representation (needed to create special single NaN) 
  }static sTmp;
  union {
    float f; // float serial digital value
    byte a[4]; //// float serial digital value array access
  }static scalingFactor;
  union {
    float f; // float serial digital value
    byte a[4]; //// float serial digital value array access
  }static offset;

  static bool tca9548a_EN=false;

  static BasicStepperDriver::Profile profile;  //Profile parameters structure to pass to the class
  
  bool err = false;        //generic bool answer
  int  val =  0;           // generic value read from serial 
  int  agv =  0;           // generic analog value           
  long dgv =  0;           // generic digital value          

          
  //Wait in external loop until somenthing arrives via serial
  if (Serial.available() >0) {
    val = Serial.read();
    // This is the state machine that process the value.
    // Itnitially is Idle (s=-1) where it checks for new arriving values 
    // to move to the next state that is the command to run  
    switch (s) {
      case -2:  //We should not be here! Protocol validation error, wait for extra data and clear. 
        delay(10);
        while (Serial.read() != -1); //clear trouble at buffer (-1 means empty) and go idle
        s=-1;
        break;
      case -1:  //idle so check comming value 1st Byte as a command to determine next state (see command table above)
        if (val>47 && val<90) {
          s=10*(val-48); //calculate next state based in first recived byte
          //Serial2.print("Sate -1 Next "); Serial2.print(s); Serial2.print(" "); Serial2.println(val); 
        }
        //validate further results. If out of range, reset state to idle
        if ((s>130 && s<170) || (s>290 && s!=340 && s!=400))
          s=-2;
        break; 
        
      case 0: //process pinMode() the second received byte indicates the pin 
              //byte value from ascii 'c'=99, pin 2, to ascii 'ª'=166, pin 69  
        if (val>98 && val<167) {
          pin=val-97; //calculate pin       
          s=1;        // next we will need to know status for this pin in, out, or pull */
        } 
        else {
          s=-2; //validation failed so reset
        }
        break;
      case 1: //process pinMode() third byte to find pin status to be  input, output, or pullUp
              //values are defined as: ascii '0'=48 for input, ascii '1'=49 for output, and ascii '2'=50 for pullUp
              //51 to 55 for PWM Frequencies  
        if (val>47 && val<56)  {
          // set pin mode
          switch(val){
            case 50:
              pinMode(pin,INPUT_PULLUP);
              break;
            case 49:
              pinMode(pin,OUTPUT);
              break;
            case 51 ... 55:  //Special mode for analog write with alternative frequencies (see setPwmFrequencyMEGA below)
              setPwmFrequencyMEGA(pin,val);
              break;             
            default:
              pinMode(pin,INPUT);         
          }   
          s=-1;  //pinMode() is complete, so reset state
        }
        else
          s=-2; //Validation trouble
        break;

      case 10: //digitalRead() the second received byte indicates the pin 
              //byte value from ascii 'c'=99, pin 2, to ascii 'ª'=166, pin 69  
        if (val>98 && val<167) {
          pin=val-97;                //calculate pin    
          dgv=digitalRead(pin);      // read and return value via serial 
          Serial.write(dgv+48);   //This returns 48 ascii '0' or 49 ascii '1' 
          s=-1;  //digitalRead() is complete, so reset state     
        }
        else
          s=-2; //Validation trouble
        break; 

      case 20: //digitalWrite() the second received byte indicates the pin 
               //byte value from ascii 'c'=99, pin 2, to ascii 'ª'=166, pin 69  
        if (val>98 && val<167) {
          pin=val-97;   //calculate pin  
          s=21;         //next we will need to know from serial if set 0 or 1 
        } 
        else {
          s=-2; //if validation fails, reset
        }
        break;
      case 21:  //digitalWrite() the third received byte indicates the value to set:
                // ascii '0'=48 for 0, and  ascii '1'=49 for 1 
        if (val>47 && val<50) {
          dgv=val-48;                /* calculate value        */
          digitalWrite(pin,dgv);     /* perform Digital Output */
          s=-1;  //digitalWrite() complete
        }
        else
          s=-2; //Validation trouble
        break; 

      case 30://analogRead() the second received byte indicates the pin 
              //byte value from ascii 'a'=97, pin 0, to ascii 'p'=112, pin 15  
        if (val>96 && val<113) {
          pin=val-97;           //calculate pin    
          agv=analogRead(pin);  //read and return via serial
          Serial.write(agv & 0xFF); //send binary LSB  
          Serial.write(agv>>8);     //send binary MSB
          s=-1; //analogRead() complete
        }
        else
          s=-2; //Validation trouble
        
        break;

      case 40://analogWrite() the second received byte indicates the pin 
              //byte value from ascii 'c'=99, pin 2, to ascii 'ª'=166, pin 69  
        if (val>98 && val<167) {
          pin=val-97;   // calculate pin       
          s=41;         // next find byte value from serial to be written
        }
        else {
          s=-2; //if validation fails, reset
        }
        break; 
      case 41: //analogWrite() the third received byte indicates the value to write
        analogWrite(pin,val);
        s=-1;  //analogWrite() complete
        break;                 

      case 50: //servoStatus() the second received byte indicates the pin
               //byte value from ascii 'c'=99, pin 2, to ascii 'l'= 108, pin 11
        if (val>98 && val<109) {
          pin=val-97;                 // calculate pin
          dgv=servo[pin].attached();  //read if attached and send answer as 0 or 1
          Serial.write(dgv);
          s=-1;  //Competed, so reset    
        }
        else
          s=-2; //Validation trouble
        break; 
         
      case 60://servoAttach()/servoDetach() the second received byte indicates the pin
              //byte value from ascii 'c'=99, pin 2, to ascii 'l'= 108, pin 11 
        if (val>98 && val<109) {
          pin=val-97;    //calculate pin and set next state to set if Attach or Detach    
          s=61;   
        } 
        else {
          s=-2; //validation failed
        }
        break;
      case 61://servoAttach()/servoDetach() the third received byte indicates the value to set:
              // ascii '0'=48 is Detach, and  ascii '1'=49 is Detach 
        if (val>47 && val<50) {
          dgv=val-48;   //convert ascii '0' or '1' to number 0 or  1, false or true.
          if (dgv)
            servo[pin].attach(pin);    
          else
            servo[pin].detach();     
          s=-1;  // Completed, so reset
        }
        else
          s=-2; //Validation trouble
        break;

      case 70://servoRead() the second received byte indicates the pin
              //byte value from ascii 'c'=99, pin 2, to ascii 'l'= 108, pin 11
        if (val>98 && val<109) {
          pin=val-97;     //calculate pin   
          agv=servo[pin].read();// read servo and send position back to client
  	      Serial.write(agv);
          s=-1;  // Completed, so reset  
        }
        else
          s=-2; //Validation trouble
        break; 
        
      case 80://servoWrite() the second received byte indicates the pin
              //byte value from ascii 'c'=99, pin 2, to ascii 'l'= 108, pin 11
        if (val>98 && val<109) {
          pin=val-97;   //calculate pin and set next state to set the position
          s=81; 
        }
        else {
          s=-2; //validation failed
        }
        break;  
      case 81://servoWrite() the third received byte indicates the value to write 
        servo[pin].write(val);                
        s=-1;  // Completed, so reset
        break;        

      case 90: //rampino() constructor calling for validation for the this server type and been active.
               //client sends 57 or ascii '9' twice, this server answers 82 or ascii 'R'for rampino type
        //Serial2.print("Sate 90 "); Serial2.println(val); 
        if (val==57) { 
          Serial.write('R');  //value 82
          s=-1;  // Completed, so reset
        }
        else
          s=-2; //Validation trouble
        break;     

      case 100://tone() the second received byte indicates the pin
              //byte value from ascii 'c'=99, pin 2, to ascii 'ª'=166, pin 69  
        if (val>98 && val<167) {
          pin=val-97;   //calculate pin 
          //Keep map of pins used to tone instances (using 3 tones on pin simultaneously)
          toneInstIdx=isPinAttached(tonePinMap,sizeof(tonePinMap),pin);
          if (toneInstIdx==-1){ //pin no attached, attach (tonePinMap) and init object (tonePin)
            if (++toneInstCnt>2) toneInstCnt=0;   //Roll over 0, 1, 2, 0...      
            toneInstIdx=toneInstCnt; 
            tonePinMap[toneInstIdx]=pin;  
            tonePin[toneInstIdx].begin(pin);
          }

          s=101; //set next states to get freq
        }
        else {
          s=-2; //validation failed
        }
        break;  
      case 101: // tone(), the second received parameter is int (1 of 2 bytes), spread in 2 states to allow tasks run in back
                // It indicates frequency from 1 to 65536 Hz, 0 is to stop the tone  
          frequency=(uint16_t )val; //LSB
          s=102; // Go for Next byte
        break;
       case 102: // tone(), the second received parameter is int (2 of 2 bytes), spread in 2 states to allow tasks run in back
                // It indicates frequency from 1 to 65536 Hz, 0 is to stop the tone  
          frequency|=(uint16_t )val<<8;  //MSB
          s=103; // Go for Next Parameter, duration.
        break; 
      case 103: // tone(), the third received parameter is int (1 of 2 bytes), spread in 2 states to allow tasks run in back
                // It indicates duration from 1 to 65536 mS, 0 is to play the tone continuosly  
          duration=(uint16_t )val; //LSB
          s=104; // Go for Next byte
        break;
       case 104: // tone(), the third received parameter is int (2 of 2 bytes), spread in 2 states to allow tasks run in back
                // It indicates duration from 1 to 65536 mS, 0 is to play the tone continuosly 
          duration|=(uint16_t )val<<8;  //MSB
          if (frequency){
            tonePin[toneInstIdx].play(frequency, duration);
          }
          else{
            tonePin[toneInstIdx].stop();
          }
          s=-1; // Done
        break; 

     case 110: // DCMotorAttach and DCMotorDetach, the second received byte indicates the DC motor number:
                // ascii '1'=49 for 1 to ascii '5'=53 for 5
        if (val>48 && val<54) {
          motor=val-49;       //calculate motor 0-4 and advance state to get attach/detach request
          s=111;
        } 
        else 
          s=-2; //Validation failed
        break;
      case 111: // DCMotorAttach and DCMotorAttach, the third byte received indicates if we attach or detach:
                // ascii '0'=48 is Detach,  ascii '1'=49 is Attached, ascii '2'=50 is Attached with Direction Flip   
        switch (val){
          case '0':  //Dettach
            dcMotor[motor].disable(); //disable pins, so they are available for other uses 
            dcAttached[motor]=0;    //Flag this motor detached
            s=-1;  // Completed 
            break;
          case '2':
            dcMotor[motor].flip(true);      //flip polarity of direction for this pin 
            dcAttached[motor]=2;    //Flag this motor soft attached and disable pins, so they are available for other uses 
            dcMotor[motor].begin();
            dcMotor[motor].enable();
            s=-1;  // Completed
            break;       
          case '1':  //Attach 
            dcMotor[motor].flip(false);      //unflip polarity of direction for this pin   
            dcAttached[motor]=1;    //Flag this motor soft attached and disable pins, so they are available for other uses 
            dcMotor[motor].begin();
            dcMotor[motor].enable();
            s=-1;  // Completed
            break;
          default: // Wrong value
            s=-2;           
        }
        break;

      case 120: // DCMotorStatus, the second received byte indicates the DC motor number:
                // ascii '1'=49 for 1 to ascii '5'=53 for 5
        if (val>48 && val<54) {
          motor=val-49;       //calculate motor 
          dgv=dcAttached[motor] ;
          Serial.write(dgv);  //Return attached status 0, 1, or 2
          s=-1; // Completed, so reset
        }
        else 
          s=-2; //Validation failed
        break;

      case 130: // DCMotorVelocity, the second received byte indicates the DC motor number:
                // ascii '1'=49 for 1 to ascii '5'=53 for 5
        if (val>48 && val<54) {
          motor=val-49;       //calculate motor and advance state to get velocity
          s=131;
        } 
        else
          s=-2; //Validation failed
        break;
      case 131: // DCMotorVelocity, the third received byte indicates the DC motor DCMotorVelocity in %
                // value is a vector, includes direction (sign)  -100 to 100, 0 is stop (brake mode) 
        dcMotor[motor].setVelocity(val);  //update object velocity
        s=-1;  // Completed, so reset
        break;

      case 170: // stepperAttach and stepperDetach, the second received byte indicates the stepper motor number:
                // ascii '1'=49 for 1 to ascii '5'=53 for 5
        if (val>48 && val<54) {
          motor=val-49;       //calculate motor 0-4 and advance state to get attach/detach request
          s=171;
        } 
        else
          s=-2; //Validation failed
        break;
      case 171: // stepperAttach and stepperDetach, the third byte received indicates if we attach or detach:
                // ascii '0'=48 is Detach, ascii '1'=49 is Attach SoftwareMode, or ascii '2'=50 is Attach HardwareMode 
        switch (val){
          case '0':  //Dettach
            sMotor[motor].disable(); 
            digitalWrite(MODE_MS3[0],0);
            digitalWrite(MODE_MS2,0);
            digitalWrite(MODE_MS1,0);
            pinMode(MODE_MS3[motor],INPUT);
            if (!isAttached(smAttached,sizeof(smAttached))){  //If I was the last motor, free the other SPI pins used by all motors
              pinMode(MODE_MS2,INPUT);
              pinMode(MODE_MS1,INPUT); 
            } 
            smAttached[motor]=0;    //Flag this motor detached and disable pins, so they are available for other uses 
            s=-1;  // Completed
            break;
        
          case '1':  //Attach SoftwareMode 
            smAttached[motor]=1;    //Flag this motor software attached, and reserve pins
            pinMode(MODE_MS3[motor],OUTPUT);
            pinMode(MODE_MS2,OUTPUT);
            pinMode(MODE_MS1,OUTPUT); 
            digitalWrite(MODE_MS3[0],0);
            digitalWrite(MODE_MS2,0);
            digitalWrite(MODE_MS1,0);
            sMotor[motor].begin();
            sMotor[motor].setEnableActiveState(LOW);
            s=-1;  // Completed
            break;          
          
          case '2':  //Attach in HardwareMode
            smAttached[motor]=2;    //Flag this motor hardware attached and disable pins, so they are available for other uses 
            pinMode(MODE_MS3[motor],INPUT); 
            if (!isAttached(smAttached,sizeof(smAttached))){  //If I was the last motor, free the other SPI pins used by all motors
              pinMode(MODE_MS2,INPUT);
              pinMode(MODE_MS1,INPUT);
            }
            sMotor[motor].begin();
            sMotor[motor].setEnableActiveState(LOW);
            s=-1;  // Completed
            break; 
          default:
            s=-2;  // Wrong Value
        }
        break;
 
      case 180: // stepperStatus, the second received byte indicates the stepper motor number:
                // ascii '1'=49 for 1 to ascii '5'=53 for 5 to return steps status
                // ascii 'E'=69 for 1 to ascii 'I'=73 for 5 to return attachment mode (0 to 2)(0 to 2) status
        if (val>48 && val<54) {
          motor=val-49;       //calculate motor 
          dgv=sMotor[motor].getStepsRemaining();
          dgv=smAttached[motor]? labs(dgv) : -1;  //send steps (absolute) remaining if attached or -1  otherwise 
          Serial.write(dgv & 0xFF); //send binary LSB first
          Serial.write(dgv>>8 & 0xFF);
          Serial.write(dgv>>16 & 0xFF);  
          Serial.write(dgv>>24);     //send binary MSB last
          s=-1; // Completed, so reset
        }
        else if(val>68 && val<74){
          motor=val-69;       //calculate motor
          Serial.write(smAttached[motor]+48); //send attachment status as ascii '0' to '2'
          s=-1; // Completed, so reset
        }
        else
          s=-2; //Validation failed 
        break;
   
      case 190: // stepperSpeed, the second received byte indicates the stepper motor number:
                // ascii '1'=49 for 1 to ascii '5'=53 for 5
        if (val>48 && val<54) {
          motor=val-49;       //calculate motor and advance state to get speed
          s=191;
        } 
        else
          s=-2; //Validation failed
        break;
      case 191: // stepperSpeed, the third received byte indicates the stepper motor speed in rpm:
                // value is in range 0 to 255 
        sMotor[motor].setRPM(val);  //update object RPMs
        s=192;  // Go for profile mode
        break;
      case 192: // stepperSpeed, the fourth received byte indicates the stepper motor profile mode:
                // value is in range 0 to 1  {CONSTANT_SPEED, LINEAR_SPEED}
        profile.mode= (BasicStepperDriver::Mode)val;  
        s=193;  // Go for profile acceleration LSB
        break;
      case 193: // stepperSpeed, the fifth received byte indicates the stepper motor acceleration profile:
                // value is the LSB in range 0 to 65536
        profile.accel=val;
        s=194;  // Go for profile acceleration MSB
        break;
      case 194: // stepperSpeed, the sixth received byte indicates the stepper motor  acceleration profile:
                // value is the MSB in range 0 to 65536
        profile.accel|=val<<8;
        s=195;  // Go for profile deceleration LSB
        break;
      case 195: // stepperSpeed, the seventh received byte indicates the stepper motor deceleration profile:
                // value is the LSB in range 0 to 65536 
        profile.decel=val;
        s=196;  //// Go for profile deceleration MSB
        break;
      case 196: // stepperSpeed, the eighth received byte indicates the stepper motor deceleration profile:
                // value is the MSB in range 0 to 65536
        profile.decel|=val<<8;
        if((profile.mode==BasicStepperDriver::CONSTANT_SPEED || profile.mode== BasicStepperDriver::LINEAR_SPEED) &&
            profile.accel>0 && profile.accel<16384 &&  profile.decel>0 && profile.decel<16384) //Range validation
            sMotor[motor].setSpeedProfile(profile);
        s=-1;  // Completed or invalid parameters, so reset
        break;

      case 200: // stepperStep, the second received byte indicates the stepper motor number:
                // ascii '1'=49 for 1 to ascii '5'=53 for 5
        if (val>48 && val<54) {
          motor=val-49;       //calculate motor (0 to 4 in local objects) and advance state to get speed
          s=201;
        } 
        else
          s=-2; //Validation failed
        break;
      case 201: // stepperStep, the third received byte indicates operation to be made 
                // val is by  #define DISABLE, STOP, BRAKING, or ENABLE
        switch (val){
          case DISABLE:            
            smAutoDSet[motor]=false;
            smAutoD[motor]=0;
   //         limitDetect[mt]=false;
            smLimitOn[motor]=0; 
            sMotor[motor].disable();//stop inmediatly and free motor
            s=-1;  //Done here  
            break;
          case STOP:           
            smAutoDSet[motor]=false;
            smAutoD[motor]=0;
    //        limitDetect[mt]=false;
            smLimitOn[motor]=0; 
            sMotor[motor].stop();  //stop inmediatly and keep engaged
            s=-1;  //Done here  
            break;
          case BRAKING:
            sMotor[motor].startBrake(); //stop with decceleration (Linear only)  and keep engaged
            s=-1;  //Done here    
            break;
          case ENABLE:        //Normal step run, enabled at the end
            sdgv8= ENABLE;    //Normal run with profile if set linear
            s=202;  //Next take mode 
            break;
          case CHANGE_RUN:    //Change (add) steps run, enabled at the end
            sdgv8= CHANGE_RUN;  // Flag change (add) steps (and ignore profile)
            s=202;
            break;
          case ENABLE_AD:     //Normal step run, auto disabled after small delay at the end
            sdgv8= ENABLE_AD;    //Normal run with profile if set linear
            s=202;  //Next take mode 
            break;         
          case CHANGE_RUN_AD: //Change (add) steps run, auto disabled after small delay at the end
            sdgv8= CHANGE_RUN_AD;    //Normal run with profile if set linear
            s=202;  //Next take mode 
            break;
          default:
            s=-2;  //bad operation failed validation.
            break;
        }
        break;
      case 202: // stepperStep, the fourth received byte indicates microstepping mode 
                // ascii '0'=48 for 1 to ascii '7'=55 for 7
        if (val>47 && val<56) {
          if(smAttached[motor]==1){ //Atemp to write Microstepper mode only attached in software mode 
            digitalWrite(MODE_MS3[motor],(val-48)>>2);
            digitalWrite(MODE_MS2,((val-48)&2)>>1);
            digitalWrite(MODE_MS1,(val-48)&1);
          }
//    sMotor[motor].enable(); //Ready to go
          sMotor[motor].setMicrostep(microstepMode[val-48]);   //calculate and map mode, val is 0 to 7, function needs 1X to 32X stepping
          s=203; //Go next for pin number parameter
        }
        else
          s=-2; //Validation failed
        break;
      case 203: // stepperStep, the fifth received parameter is the pin (if used) for fast homing operations 
                //byte value from ascii 'a'=97, pin 0, to ascii 'ª'=166, pin 69 (Pin 0 is no limit, 1 to 69 Active Low limit) (pin 0 or 1 are not physically used)
                //byte value from ascii 'º'=167, pin 2, to ascii 'Û'=234, pin 69 (Pin 2 to 69 Active High limit)
        if (val>166) {  //If this is an active high limit
          val-=68;  //correct pin to 96 to 167 range and define limActive as active high 
          limActive = HIGH;
        }
        else{ 
          limActive = LOW;
        }
        if (val>96 && val<167) {
          pin=val-97;                //calculate pin   
          if (pin>1){ //Requesting limit at pin
            pinMode(pin,INPUT_PULLUP); //Prepare limit switch pin
            delay(1); //Let the weak pull up fight any capacitance for a mS
            if (digitalRead(pin)==limActive){ //  If it is already at limit cancel whole step moving
              char dummy[4];
              smLimitOn[motor]=0; //Make sure flag is reset 
              Serial.readBytes(dummy, 4);//clear the remaining bytes to dummy 
              // Serial2.println("OOOPS ALREADY LOW ");
              s=-1; //oops never mind, we are done!
              break;
            }
          }
          smLimitOn[motor]=pin;   //Flag limit using the pin or 0 if no limit, so steeper is stop at limit on the main loop (no blocking)
          // Serial2.print("Limit pin ");Serial2.println(smLimitOn[motor]);
          sMotor[motor].enable(); //get Ready to go ...
          s=204; // Go for next steps parameter
          break;   
        }
        else
          s=-2; //Validation trouble
        break;
      case 204: // stepperStep, the sixth received parameter is a long (1 of 4 bytes), spread in 4 states to allow tasks run in back
                // It indicates stteper steps to run. Positive value is right motion, negative left 
        sdgv=(long)val; //LSB
        s=205; // Go for Next byte
        break;
      case 205: // stepperStep, the sixth received parameter is a long (2 of 4 bytes), spread in 4 states to allow tasks run in back
                // It indicates stteper steps to run. Positive value is right motion, negative left 
        sdgv|=(long)val<<8;
        s=206; // Go for Next byte
        break;
      case 206: // stepperStep, the sixth received parameter is a long (3 of 4 bytes), spread in 4 states to allow tasks run in back
                // It indicates stteper steps to run. Positive value is right motion, negative left 
        sdgv|=(long)val<<16; 
        s=207; // Go for Next byte
        break;
      case 207: // stepperStep, the sixth received parameter is a long (4 of 4 bytes), spread in 4 states to allow tasks run in back
                // It indicates stteper steps to run. Positive value is right motion, negative left 
        sdgv|=(long)val<<24; //MSB 
        s=-1; // flag Done OK, pending operation
        switch (sdgv8){
          case ENABLE_AD:
            sMotor[motor].startMove(sdgv);   //And go that number of steps!!  
            smAutoD[motor]=1; //Once steps are in, set flag to use autodelay
            break;
          case CHANGE_RUN_AD:
            sMotor[motor].alterMove(sdgv); // Add steps to this run without the linear profile
            smAutoD[motor]=1; ////Once steps are in, set flag to use autodelay
            break;
          case ENABLE:
            sMotor[motor].startMove(sdgv);   //And go that number of steps!!  
            break;
          case CHANGE_RUN: 
            sMotor[motor].alterMove(sdgv); // Add steps to this run without the linear profile
            break;
          default:
            s=-2; //Bad operation cancel
            break;
        }      
        // Serial2.print(motor);Serial2.print(" ");Serial2.print(sdgv);Serial2.print(" ");Serial2.println(sdgv8);

        break;

      case 210: // encoderAttach, the second received byte indicates the encoder number:
                // ascii '1'=49 for 1, ascii '1'=50 for 2, and  ascii '3'=51 for 3
        if (val>48 && val<52) {
          enc=val-49;       //calculate encoder and advance state to get pin A
          s=211;
        } 
        else
          s=-2; //Validation failed
        break;
  
      case 211: // encoderAttach, the third received byte indicates the requested pin for A
                // Encoders need interrupt pins in a range from ascii 'c'=99, pin 2, to ascii 'v'=118, pin 21  
        if (val>98 && val<119) {
          pin=val-97;           //Calculate pin A and attach
          Enc[enc].pinA=pin;       
          s=212;  //advance state to get pin B
        } 
        else
          s=-2; //Validation failed
        break;

      case 212:// encoderAttach, the fourth received byte indicates the requested pin for B
                // Encoders need interrupt pins in a range from ascii 'c'=99, pin 2, to ascii 'v'=118, pin 21 
        if (val>98 && val<119) {
          pin=val-97;           //Calculate pin B and attach
          Enc[enc].pinB=pin;       
          //set encoder pins as inputs  with pullups
          pinMode(Enc[enc].pinA, INPUT_PULLUP); 
          pinMode(Enc[enc].pinB, INPUT_PULLUP);       
          /* attach interrupts                                 */
          switch(enc) {
            case 0:
              attachInterrupt(digitalPinToInterrupt(Enc[0].pinA), isrPinAEn0, CHANGE);
              attachInterrupt(digitalPinToInterrupt(Enc[0].pinB), isrPinBEn0, CHANGE);
              break;  
            case 1:
              attachInterrupt(digitalPinToInterrupt(Enc[1].pinA), isrPinAEn1, CHANGE);
              attachInterrupt(digitalPinToInterrupt(Enc[1].pinB), isrPinBEn1, CHANGE);
              break;  
            case 2:
              attachInterrupt(digitalPinToInterrupt(Enc[2].pinA), isrPinAEn2, CHANGE);
              attachInterrupt(digitalPinToInterrupt(Enc[2].pinB), isrPinBEn2, CHANGE);
              break;  
          }
          s=-1; //Done 
        }
        else
          s=-2; //Validation failed 
        break;

      case 220:// encoderDetach, the second received byte indicates the encoder number:
                // ascii '1'=49 for 1, ascii '1'=50 for 2, and  ascii '3'=51 for 3
        if (val>48 && val<52) {
          enc=val-49;        //calculate encoder number and detach
          detachInterrupt(digitalPinToInterrupt(Enc[enc].pinA));
          detachInterrupt(digitalPinToInterrupt(Enc[enc].pinB));
          s=-1; // Done with encoderDetach, so reset
        }
        else
          s=-2; //Validation failed
        break;

      case 230:// encoderPosition, the second received byte indicates the encoder number:
                // ascii '1'=49 for 1, ascii '1'=50 for 2, and  ascii '3'=51 for 3
        if (val>48 && val<52) {
          enc=val-49;     //Calculate encoder and send back position value
          Serial.write(Enc[enc].pos & 0xFF);  //send binary LSB  
          Serial.write(Enc[enc].pos >>8);     //send binary MSB
          s=-1;  // Done with encoderDetach, so reset
        }
        else
          s=-2; //Validation failed
        break;

      case 240://encoderReset, the second received byte indicates the encoder number:
               // ascii '1'=49 for 1, ascii '1'=50 for 2, and  ascii '3'=51 for 3
        if (val>48 && val<52) {
          enc=val-49;     //Calculate encoder and reset position
          Enc[enc].pos=0;
          s=-1;  // Done, so reset
        }
        else
          s=-2; //Validation failed
        break; 

      case 250:// encoderDebounce, the second received byte indicates the encoder number:
               // ascii '1'=49 for 1, ascii '1'=50 for 2, and  ascii '3'=51 for 3
        if (val>48 && val<52) {
          enc=val-49;   //Calculate encoder and move to next state to find the delay requested
          s=251; 
        } 
        else
          s=-2; //Failed validation
        break;
        
      case 251://encoderDebounce, the third received byte indicates the delay value 
               //byte value from ascii 'a'=97, 0 units , to ascii 'ª'=166, 69 units (in tenths of mS, 0.1mS)  
        if (val>96 && val<167) {
          Enc[enc].del=val-97;   //Calculate and set debounce delay
          s=-1;  // Done, so reset 
        }
        else
          s=-2; //Validation failed
        break; 
      
      case 270: //spiLEDs() SPI LEDs string. The second byte received byte indicates the Operation:
                //ascii '1'=49 for op 1 to ascii '?'=63 for op 15, the operations defined above in the #defines 
        switch (val){    
            case ENPIXS:  
              strip.begin();
              //Serial2.println("ENABLE BEGIN()");
              s=-1;  //Completed, so reset.
              break;              
            case SETNUMPIXELS:  
              s=271;   //64 is by default, move to next two states to get new 16bit len (updateLength)
              break;
            case SHOW:
              strip.show();
              //Serial2.println("SHOW!");
              s=-1;  //Completed, so reset.
              break;
            case CLEAR:
              strip.clear();
              //Serial2.println("CLEAR!");
              s=-1;  //Completed, so reset.
              break;
            case SETPIX:
              s=273;  //Move to next five states to get 3x8bit RGB color, and then 16bit position (setPixelColor)  
              break;            
            case SETPIXS:
              s=278;  //Move to next seven states to get 3x8bit RGB color, and then 2x16bit start and end fill positions (fill)  
              break;            
            case SETBRIGHT:
              s=285;  //Move to next state to get 8bit global brightnes 
              break;
            default: //Validation for operation failed
              s=-2;
              break;        
        }
        break;
      case 271://spiLEDs() updateLength SPI LEDs string. The third byte received  indicates number of LEDs in chain (LSB) for 16 bit
        sdgv8=val; //Store LSB Len value
        s=272;  // Go for MSB
        break;
      case 272: //Take 4h Byte for MSB
        strip.updateLength((val<<8)+sdgv8); //Compose and set 16 bit number of LEDs
        //Serial2.print("UPDATE LENGTH "); Serial2.println((val<<8)+sdgv8);
        s=-1;  // And done, reset state machine
        break;
      case 273://spiLEDs() setPixelColor. The third byte received indicates Red Color Level 8Bit
        red=val; //Store Red value
        s=274;  // next
        break;
      case 274: //spiLEDs() setPixelColor. The forth byte received indicates Green Color Level 8Bit
        green=val; //Store Green value
        s=275;  // Next
        break;
      case 275: //spiLEDs() setPixelColor. The fith byte received indicates Blue Color Level 8Bit
        blue=val; //Store Blue value
        s=276;  // Next
        break;   
      case 276: //spiLEDs() setPixelColor. The sixth byte received indicates Position from 0 (LSB)
        sdgv8=val; //Store LSB
        s=277;  // Next
        break;
      case 277: //spiLEDs() setPixelColor. The seventh byte received indicates Position from 0 (MSB)
        strip.setPixelColor((val<<8)+sdgv8, red, green, blue); //Compose and set 16 bit position and 3X8bit RGB 
        //Serial2.print((val<<8)+sdgv8); Serial2.print(" ");
        //Serial2.print(red); Serial2.print(" ");   Serial2.print(green); Serial2.print(" "); Serial2.println(blue);
        s=-1;  // And done, reset state machine
        break;    
      case 278://spiLEDs() fill. The third byte received indicates Red Color Level 8Bit
        red=val; //Store Red value
        s=279;  // next
        break;
      case 279: //spiLEDs() fill. The forth byte received indicates Green Color Level 8Bit
        green=val; //Store Green value
        s=280;  // Next
        break;
      case 280: //spiLEDs() fill. The fith byte received indicates Blue Color Level 8Bit
        blue=val; //Store Blue value
        s=281;  // Next
        break;   
      case 281: //spiLEDs() fill. The sixth byte received indicates Position from 0 (LSB)
        sdgv16=val; //Store LSB
        s=282;  // Next
        break;
      case 282: //spiLEDs() fill. The seventh byte received indicates Position from 0 (MSB)
        sdgv16+=val<<8; //Store Blue value
        s=283;  // Next
        break;   
      case 283: //spiLEDs() fill. The eigth byte received indicates Number of LEDs (length) to fill (LSB)
        sdgv8=val; //Store LSB
        s=284;  // Next
        break;
      case 284: //spiLEDs() fill. The nineth byte received indicates Number of LEDs (length) to fill (MSB)
        strip.fill((uint32_t)(((uint32_t)red<<16)+((uint16_t)green<<8)+blue),sdgv16,(uint16_t)((val<<8) + sdgv8)); //Compose and set 16 bit position
       // Serial2.print((uint32_t)(((uint32_t)red<<16)+((uint16_t)green<<8)+blue)); Serial2.print(" ");  Serial2.print(sdgv16); Serial2.print(" "); Serial2.println((uint16_t)(((uint16_t)val<<8) + sdgv8)); 
        s=-1;  // And done, reset state machine
        break;    
      case 285: //spiLEDs() setBrightness. The third byte received indicates the global brightness
        strip.setBrightness(val);
        //Serial2.print("SET BRIGHTNESS "); Serial2.println(val); 
        s=-1;  // And done, reset state machine
        break; 

      case 290: //I2C Modules. The second byte received byte indicates the Operation:
                //ascii '1'=49 for op 1 ...., the operations defined above in the #defines 
        switch (val){    
            case TCA9548_EN:  //Switch Enable with I2C address x70 is default
              delay(50);
              sdgv8=Serial.read();
              Wire.begin();
              tca9548aMux.begin(sdgv8 & 0x7); //set Mux Switch to 0x71 (1) from 0x70 as the SHTC3 also is 0x70
              //Validation sequence
              tca9548aMux.setChannel(8);
              err=!(tca9548aMux.getChannel()==8);
              tca9548aMux.setChannel(0);
              err|=!(tca9548aMux.getChannel()==0);
              Serial.write(err); //Bool response. 0 means no error
              if(!err) tca9548a_EN=true;
              s=-1; //Completed, so reset.
              break;              
            case TCA9548_SEL:  //Swich Port
              delay(50);
              sdgv8=Serial.read(); //get selected port 2^n where n is the channel. It can be summed to select multiple channels in a multibus
              if (tca9548a_EN){
                tca9548aMux.setChannel(sdgv8); //1 byte with value 0 is none, then 1, 2, 4,...128, for channels 0 to 7, a 255 will select all channels
                Serial.write(!(tca9548aMux.getChannel() == sdgv8)); //Validate call with bool response 
              }
              else{
                 Serial.write(1); //I2C MUST be enable before attemping any communication, or the internal driver can crash
              }
              s=-1; //Completed, so reset.
              break;   
            case STC3X_EN://CO2 Sensor Enable (uses SHTC3 for compensation)
              delay(50); 
              sdgv8=Serial.read();
              val=Serial.read();
              //Serial2.println("STC3EN");
              err=!stc3xSensor.begin(sdgv8); //Enable it. The default address  to 0x29 (chipset can handle 0x29 to 0x2c)
              //Serial2.println(err);
              err|=!stc3xSensor.setBinaryGas((STC3X_binary_gas_type_e)val);//Set Gas type and Range, 0 to 3 (per datasheeet) 3 is for 25% range CO2 with Air
              //Serial2.println(err);
              Serial.write(err); //Bool response, true if error in any of the two commands
              s=-1; //Completed, so reset.
              break;       
            case STC3X_CAL://CO2 Sensor set for actual Gas Concentration in porcentage as calibration reference
              delay(50); //let the buffer fill, it shoudld by a float (4 bytes)
              Serial.readBytes(sTmp.a,4);
              err=!stc3xSensor.forcedRecalibration(sTmp.f);
              //Serial2.println(sTmp.f); //Debug float 
              Serial.write(err);  //Feedback to caller if function run was ok.
              s=-1;  //Completed, so reset.
              break;
            case STC3X_STMP://CO2 Sensor set sensed temperature for compensation
              delay(50); //let the buffer fill, it shoudld by a float (4 bytes)
              Serial.readBytes(sTmp.a,4);
              err=!stc3xSensor.setTemperature(sTmp.f);
              //Serial2.println(sTmp.f); //Debug float 
              Serial.write(err);  //Feedback to caller if function run was ok.
              s=-1;  //Completed, so reset.
              break;
            case STC3X_SHUM://CO2 Sensor set sensed humidity for compensation
              delay(50); //let the buffer fill, it shoudld by a float (4 bytes)
              Serial.readBytes(sTmp.a,4);
              err=!stc3xSensor.setRelativeHumidity(sTmp.f);
              //Serial2.println(sTmp.f); //Debug float 
              Serial.write(err);  //Feedback to caller if function run was ok.
              s=-1;  //Completed, so reset.
              break;
            case STC3X_SPRE://CO2 Sensor set sensed or known env. pressure for compensation
              delay(50); //let the buffer fill, it shoudld by a float (4 bytes)
              Serial.readBytes(sTmp.a,4);
              err=!stc3xSensor.setPressure(sTmp.f);
              //Serial2.println(sTmp.f); //Debug float 
              Serial.write(err);  //Feedback to caller if function run was ok.
              s=-1;  //Completed, so reset.
              break;
            case STC3X_RDT://CO2 Sensor Reads Temperature
              if(stc3xSensor.measureGasConcentration())
                sTmp.f=stc3xSensor.getTemperature();
              else
                sTmp.i=0xFFFFFFFF;  //Feedback to caller there was an error (using NaN)
              Serial.write(sTmp.a,4); //send back float value
              //Serial2.println(sTmp.f);
              s=-1;  //Completed, so reset.
              break;
            case STC3X_RDG://CO2 Sensor Reads CO2 Gas Concentration in porcentage
              if(stc3xSensor.measureGasConcentration())
                sTmp.f=stc3xSensor.getCO2();
              else
                sTmp.i=0xFFFFFFFF;  //Feedback to caller there was an error (using NaN)
              Serial.write(sTmp.a,4); //send back float value
              //Serial2.println(sTmp.f);
              s=-1;  //Completed, so reset.
              break;
            case SHTC3_EN: //Humidity Sensor Enable (fix at I2C x70)
              err=shtc3Sensor.begin();
              Serial.write(err);  //Feedback to caller if function run was ok.
              s=-1; //Completed, so reset.
              break;            
            case SHTC3_RDH: //Humidity Sensor Reads Humidity Percentage
              err=shtc3Sensor.update(); //wait 20mS and read
              if(err)
                sTmp.i=0xFFFFFFFF;  //Feedback to caller there was an error (using NaN)
              else
                sTmp.f=shtc3Sensor.toPercent();
              Serial.write(sTmp.a,4); //send back float value
              s=-1;  //Completed, so reset.
              break;
            case SHTC3_RDT: //Humidity Sensor Reads Temperaure
              err=shtc3Sensor.update(); 
              if(err)
                sTmp.i=0xFFFFFFFF;  //Feedback to caller there was an error (using NaN)
              else
                sTmp.f=shtc3Sensor.toDegC();
              Serial.write(sTmp.a,4); //send back float value
              s=-1;  //Completed, so reset.
              break;          
            case SFM3000_EN: //GAS FLOW Sensor Enable 
              delay(100); //let the buffer fill, it shoudld by a float (4 bytes)
              Serial.readBytes(scalingFactor.a,4);
              Serial.readBytes(offset.a,4);
              sdgv8=Serial.read(); //This should be the I2C address, often 0x40
              Wire.begin();
              sfm3000Sensor.begin(Wire, sdgv8);
              sdgv16=sfm3000Sensor.startContinuousMeasurement();
              //Feedback to caller if function run was ok.
              //Serial2.println(scalingFactor.f);
              //Serial2.println(offset.f);
              //Serial2.println(sdgv16);
              Serial.write(sdgv16 & 0xFF);   //send binary LSB  
              Serial.write(sdgv16>>8);     //send binary MSB  
              s=-1;  //Completed, so reset
              break;
            case SFM3000_RD: //GAS FLOW Sensor Reads Flow in lpm
              sdgv16=sfm3000Sensor.readMeasurement(sTmp.f,scalingFactor.f,offset.f);
              //Serial2.println(sdgv16);
              if(sdgv16) sTmp.i=0xFFFFFFFF;  //Feedback to caller there was an error (using NaN)               
              Serial.write(sTmp.a,4); //send back float value or NaN error
              s=-1;  //Completed, so reset
              break;
            default: //Validation for operation failed
              s=-2;
              break;        
        }
        break;

      case 340://process analogReference() second byte that is the value or reference to be used
               //ascii '0'=48 for DEFAULT, ascii '1'=49 for INTERNAL, or ascii '2'=50 for EXTERNAL
        switch (val){ 
          case 48:
            analogReference(DEFAULT);
            s=-1;  //Completed, so reset.
            break;        
          case 49:
            analogReference(INTERNAL);
            s=-1;  //Completed, so reset.
            break;              
          case 50:
            analogReference(EXTERNAL);
            s=-1;  //Completed, so reset.
            break;
          default:
            s=-2;        
        }
        break;
        
      case 400://process roundTrip() second byte that is any byte value to echo
        Serial.write(val);
        s=-1; //Done, so reset
        break;                   
        
      default: //Invalid state, so reset.
        s=-2;
    } //State Machine Close
  }  //Serial Polling Close
 
  //Run stepper motors in the background
  if (smLimitOn[mt]>1){//if a pin is assigned (>1), pending limit, poll for Low
    if (digitalRead(smLimitOn[mt])==limActive ){                            //&& !limitDetect[mt]) { 
      //Serial2.print(mt);Serial2.print(" STOP SWITCH ENABLED BS ");Serial2.println((uint16_t)micros());
      sMotor[mt].stop(); //Stop inmediatly with enable (engaged motor)
      //Serial2.print(mt);Serial2.print(" STOP SWITCH ENABLED AS ");Serial2.println((uint16_t)micros());
      pinMode(smLimitOn[mt],INPUT); //restore input to default
      smLimitOn[mt]=0; //Limit operation done, so reset pin and detect flags
    //  limitDetect[mt]=true;
    }
  } 
  if (smAutoD[mt]){   //Autodisable
    if (!sMotor[mt].getStepsRemaining() && !smAutoDSet[mt]){  //No more steps, stopping,  and delay not set yet, so time stamp
      smAutoD[mt] = (uint8_t)millis(); //Set timestamp to detect later delayed release time 
      smAutoDSet[mt]= true;
      //Serial2.print(mt);Serial2.print(" STOP done, Steps 0 ");Serial2.println((uint16_t)micros());
    }  
    if(smAutoDSet[mt] && (uint8_t)((uint8_t)millis() - smAutoD[mt])>= RELEASE_DELAY ){  //Detect release time with rollover (max 255ms) 
      //Serial2.print(mt);Serial2.print(" Auto DISABLE ");Serial2.println((uint16_t)micros());
      sMotor[mt].disable();
      //Serial2.print(mt);Serial2.print(" AFTER DISABLE ");Serial2.println((uint16_t)micros());
      smAutoD[mt]=0;
      smAutoDSet[mt]= false;
    }
  }
  
 //long tmp= sMotor[mt].nextAction(); //us for next action
 // if (tmp>1000){
 //   Serial2.print(mt);Serial2.print(" "); Serial2.println(tmp);
 // }
  //New loop call
  static long actionTime[]={0,0,0,0,0};
  static long lastTime[]={0,0,0,0,0};
  if (sMotor[mt].getStepsRemaining() > 0){
    if (micros()-lastTime[mt] >= actionTime[mt] ){
      actionTime[mt]= sMotor[mt].nextActionIn(); // Call next action In actionTime uS
      lastTime[mt]=micros();
    }
  }  
  else{
    lastTime[mt]=0;
    actionTime[mt]=0;
  }
  
  if (++mt >4) mt=0;   //sequence motors for  switches polling, 1 each loop

}//Loop 
/*****************************************************************************************************************************/

//Check if any device is still attached (if any element is >0) 
bool isAttached(byte *items, uint8_t len){
  for (byte n=0; n<len;n++){
    if (items[n]) return true;
  }
  return false;
}
//Check if a pin is attached, if it is return position on map array, tht should match object instance 
int8_t isPinAttached(uint8_t *pinMap,uint8_t len, uint8_t pin){
  for (uint8_t n=0; n<len;n++){
    if (pinMap[n]==pin) return n;
  }
  return -1;
}


//Debouncing delay function      
void debounce(int del) {
  volatile int k;
  for (int d=0;d<del*64;d++) k=k*d;
}

//ISR when there is a change on pin A for Encoder 0
void isrPinAEn0(){
  //Decode AB positions
  bool B=digitalRead(Enc[0].pinB);
  if(Enc[0].del) debounce(Enc[0].del);  //Call delay for debounce only if del>0 
  ((digitalRead(Enc[0].pinA)) ^ B) ?  Enc[0].pos++ : Enc[0].pos--; // if A!=B clockwise, A==B counterclockwise
}

//ISR when there is a change on pin B for Encoder 0
void isrPinBEn0(){
  //Decode AB positions
  bool A=digitalRead(Enc[0].pinA);
  if(Enc[0].del) debounce(Enc[0].del); //Call delay for debounce only if del>0 
  ((digitalRead(Enc[0].pinB)) ^ A) ?  Enc[0].pos-- : Enc[0].pos++; // if A!=B counterclockwise, A==B clockwise 
}

//ISR when there is a change on pin A for Encoder 1
void isrPinAEn1(){
  //Decode AB positions
  bool B=digitalRead(Enc[1].pinB);
  if(Enc[1].del) debounce(Enc[1].del);  //Call delay for debounce only if del>0 
  ((digitalRead(Enc[1].pinA)) ^ B) ?  Enc[1].pos++ : Enc[1].pos--; // if A!=B clockwise, A==B counterclockwise
}

//ISR when there is a change on pin B for Encoder 1
void isrPinBEn1(){
  //Decode AB positions
  bool A=digitalRead(Enc[1].pinA);
  if(Enc[1].del) debounce(Enc[1].del);  //Call delay for debounce only if del>0 
  ((digitalRead(Enc[1].pinB)) ^ A) ?  Enc[1].pos-- : Enc[1].pos++; // if A!=B counterclockwise, A==B clockwise 
}

//ISR when there is a change on pin A for Encoder 2
void isrPinAEn2(){
  //Decode AB positions
  bool B=digitalRead(Enc[2].pinB);
  if(Enc[2].del) debounce(Enc[2].del);  //Call delay for debounce only if del>0 
  ((digitalRead(Enc[2].pinA)) ^ B) ?  Enc[2].pos++ : Enc[2].pos--; // if A!=B clockwise, A==B counterclockwise
}

//ISR when there is a change on pin B for Encoder 2
void isrPinBEn2(){
  //Decode AB positions
  bool A=digitalRead(Enc[2].pinA);
  if(Enc[2].del) debounce(Enc[2].del);  //Call delay for debounce only if del>0 
  ((digitalRead(Enc[2].pinB)) ^ A) ?  Enc[2].pos-- : Enc[2].pos++; // if A!=B counterclockwise, A==B clockwise 
}

//Change PWM timing in Arduino MEGA
//Warning use with caution as this can change other pin frequency functions as servo and tone control
//And may not be compatible with other pin uses simultaneously, check Arduino timers info to learn more.

void setPwmFrequencyMEGA(int pin, int divisor) { 
  byte mode;
      switch(divisor) {  //Cases for pins 3, and 5 to 12 OR pins 4 and 13 
      case 51: mode = 0x01; break; // 31372.55 Hz / 62500 Hz
      case 52: mode = 0x02; break; // 3921.16 Hz / 7812.5 Hz
      case 53: mode = 0x03; break; // 490.20 Hz / 976.56 Hz  (Default)
      case 54: mode = 0x04; break; // 122.55 Hz / 244.14 Hz
      case 55: mode = 0x05; break; // 30.61 Hz / 61.04 Hz 
      default: return;
      }
      
        switch(pin) {	  
      case 2:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 3:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 4:  TCCR0B = TCCR0B  & 0b11111000 | mode; break; //Warning used in other Arduino timing
      case 5:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 6:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 7:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 8:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 9:  TCCR2B = TCCR0B  & 0b11111000 | mode; break;
      case 10: TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 11: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 12: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 13: TCCR0B = TCCR0B  & 0b11111000 | mode; break; //Warning used in other Arduino timing
      default: return;
    }

}


