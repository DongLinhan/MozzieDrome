/* Arduino Mega with RAMPS v1.6 Plus for AIC ZMBBI
 * Columbia University 2021 */

//
// Servos
//
#define SERVO1                        11
#define SERVO2                         6
#define SERVO3                         5
#define SERVO4                         4

//
// Limit Switches
//
#define X_MIN                          3
#define X_MAX                          2
#define Y_MIN                         14
#define Y_MAX                         15
#define Z_MIN                         18
#define Z_MAX                         19

//
// Steppers
//
#define X_STEP                        54
#define X_DIR                         55
#define X_ENABLE                      38

#define Y_STEP                        60
#define Y_DIR                         61
#define Y_ENABLE                      56

#define Z_STEP                        46
#define Z_DIR                         48
#define Z_ENABLE                      62

#define E0_STEP                       26
#define E0_DIR                        28
#define E0_ENABLE                     24

#define E1_STEP                       36
#define E1_DIR                        34
#define E1_ENABLE                     30

//
// Analog Sensors
//
#define TEMP_0                        A13 
#define TEMP_1                        A14  
#define TEMP_2                        A15  

//
// Solenoids
//
#define RAMPS_D8                        8   //BIG J28
#define RAMPS_D9                        9
#define RAMPS_D10                      10

//
// Misc. Functions
//

#define LED_D5                         13
#define PS_ON                          12

//AUX-1
#define AUX1_S3                       TX0   //1
#define AUX1_S4                       RX0   //0
#define AUX1_A3                       A5
#define AUX1_A4                       A5

//AUX-2
#define AUX2_3                        A5
#define AUX2_4                        A9
#define AUX2_5                        A10
#define AUX2_6                        40
#define AUX2_7                        44
#define AUX2_8                        42
#define AUX2_9                        A12
#define AUX2_10                       A11

//AUX-3 / SPI
#define AUX3_2                        49
#define AUX3_3                        MISO  //50
#define AUX3_4                        MOSI  //51
#define AUX3_5                        SCK   //52
#define AUX3_6                        SS    //53

//AUX-4
#define AUX4_3                        32
#define AUX4_4                        47
#define AUX4_5                        45
#define AUX4_6                        43
#define AUX4_7                        41
#define AUX4_8                        39
#define AUX4_9                        37
#define AUX4_10                       35
#define AUX4_11                       33
#define AUX4_12                       31
#define AUX4_13                       29
#define AUX4_14                       27
#define AUX4_15                       25
#define AUX4_16                       23
#define AUX4_17                       17
#define AUX4_18                       16
