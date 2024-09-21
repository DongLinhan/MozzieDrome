/*                                 DC Motor PWM at any pin DCM_PWM_AP.h V1.0, SEP 2021

This library is designed to provide 'software' PWM DC Motor control for chips with Enable, PWM, and direction
like MAX14870 and similar, but with any arbitrary pin in Arduino (no need for PWM pins) also allows to control
the PWM frequency to best match motor driving. All tests are made on an Arduino MEGA that is where makes more sense
taking adventage of extra pins as PWM.

The library is a fork made and  maintained by Richard Hormigo at the ZMBBI at Columbia University derived from the Marco Colli
(Magic Design) MD_PWM library for PWM atarbitrary pins, just adding the DC motor funtionality and some modifications to better
support quick duty changes in DC motor control systems. Also this fork maintains synchronous pulsing across pins. 

While this library and MD_PWM are under GNU LGP license, The Magic design folks have a nice repository of Arduino libraries
and they would apreciate a small donation if you use and like it here:  (https://paypal.me/MajicDesigns/4USD)

NOTE: The library uses AVR TIMER1 or TIMER2 (default) and a PWM Frequency of 75Hz (default) but configurably externally. 
*/

#include <Arduino.h>

//TIMER1 is 16bit used byoriginal servo lib, and analogWrite at pins 11 and 12 (Mega)
//TIMER2 is 8 bit used by original tone lib, and analogWrite at pins 9 and 10 (Mega)
#ifndef USE_TIMER
#define USE_TIMER 2     ///< Set to 1 or 2
#endif

class DCM_PWM_AP{
  public:
    static const uint16_t FREQUENCY = 75;       // Current PWM frequency 
    static const uint16_t MAX_FREQUENCY = 300;  // the maximum PWM frequency allowed is 300
    static const uint8_t MAX_PWM_PIN = 5;       // total number of concurrent PWM pins that can be used

    //Class constructor. Needs the pins for the driver modules, PWM, Direction, and active low Enable.
    //These pins are only for the instanciation, and are not used untill the class is initatied with begin()
    //So it is posible create an object array for multiple drivers (up to 5 tested) sharing the pins with other functions. 
    DCM_PWM_AP(uint8_t PWM, uint8_t DIR, uint8_t nEN);

    //Destructor
    ~DCM_PWM_AP(void);
    
    //begin initates the object, pins, frequency, etc... Only one frequency is allowed across all instances
    bool begin(uint16_t freq = FREQUENCY);
   
    // set Motor Velocity is a -100 to 100 value that defines the duty cycle of the PWM in % ,
    // where the sign determines the direction. Values below -100 or above 100, are constrained to these limits respectivelly.  
    void setVelocity(int8_t velocity);
    
    //Flip reverses the polarity of direction, useful when and motor instance must run reversed respstect another one (direct drive). 
    void flip(bool flip);

    //Disable PWM output for this driver and pin. The motor outputs will be floating, no brake.
    void disable(void);

    //Enable driver and timer resources
    bool enable(void);

private:
#if USE_TIMER == 1
    static const uint32_t TIMER_RESOLUTION = 65535;    ///< Timer1 is 16 bit
#elif USE_TIMER == 2
    static const uint32_t TIMER_RESOLUTION = 256;    ///< Timer2 is 8 bit
#endif
    uint8_t _pin;         // PWM digital pin
    uint8_t _DIR;
    uint8_t _nEN;
    bool _flip = false;

    volatile uint8_t * _outReg;    //output port register for fast writing
    volatile uint8_t _outRegMask;  //output pin mask in port register for fast writing
    volatile uint8_t _pwmDutyPS;   //Presync PWM duty set point
    volatile uint8_t _pwmDuty;     //PWM duty set point
    
    // Write raw PWM driving value 0-255.
    void write(uint8_t duty);

    void setFrequency(uint32_t freq); // set TIMER frequency
    inline void setTimerMode(void);   // set TIMER mode
    inline void attachISR(void);      // attach to TIMER ISR
    inline void detachISR(void);      // detach from TIMER ISR
    inline void stop(void);           // stop the timer

public:
  static bool _bInitialised;          ///< ISR - Global vector initialization flag
  static volatile uint8_t _pinCount;  ///< ISR - Number of PWM pins currently configured
  static DCM_PWM_AP* _cbInstance[];   ///< ISR - Callback instance handle per PWM pin slot

  void setPin(void); // ISR pin handler (this runs 256 times per PWM cycle)

};

