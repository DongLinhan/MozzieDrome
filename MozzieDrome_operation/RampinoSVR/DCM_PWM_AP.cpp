// DC Motor PWM at any pin Class
#include "DCM_PWM_AP.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Global data
volatile uint8_t gCnt;      //Global timeslot counter controlled by timer ISR
bool DCM_PWM_AP::_bInitialised = false;
volatile uint8_t DCM_PWM_AP::_pinCount; 
DCM_PWM_AP* DCM_PWM_AP::_cbInstance[MAX_PWM_PIN];  //Array of pointers to the instances

//Constructor with pin parametrers for PWM, Direction and Active low enable
DCM_PWM_AP::DCM_PWM_AP(uint8_t PWM, uint8_t DIR, uint8_t nEN) : _pin(PWM), _DIR(DIR), _nEN(nEN) { };

//Destructor 
DCM_PWM_AP::~DCM_PWM_AP(void) { 
  disable();
  if (_pinCount == 0){
    stop();
    detachISR();
  }
}

//begin initates and enables the object, pins, frequency, etc... Only one frequency is allowed across all instances
bool DCM_PWM_AP::begin(uint16_t freq){
  if (freq > MAX_FREQUENCY) // Max frequency allowed
    return(false);
  _outReg=portOutputRegister(digitalPinToPort(_pin)); //Load in instance output pin register
  _outRegMask=digitalPinToBitMask(_pin);              //Load in instance the mask for the pin at the output register
  // Set up global data and hardware
  if (!_bInitialised){
    for (uint8_t i = 0; i < MAX_PWM_PIN; i++)
      _cbInstance[i] = nullptr;
    _pinCount = 0;    // we have no pins globally allocated yet
    setTimerMode();
    setFrequency(freq);
    attachISR();
    _bInitialised = true;
  }
  // Set pins for each instance
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  pinMode(_DIR, OUTPUT);
  digitalWrite(_DIR, LOW);
  pinMode(_nEN, OUTPUT);
  digitalWrite(_nEN, HIGH); // Initially off untill enable validated
  return(enable());
}

// Enable the PWM pin and driver from initial conditions, PWM low (Brake state in motor driver)
bool DCM_PWM_AP::enable(void){
  bool found = false;
   //retake Pins
  pinMode(_nEN, OUTPUT);
  pinMode(_DIR, OUTPUT);
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  digitalWrite(_nEN, LOW);
  for (uint8_t i = 0; i < MAX_PWM_PIN; i++){
    if (_cbInstance[i] == nullptr){
      found = true;
      _cbInstance[i] = this;  // save ourselves in this slot
      _pinCount++;        // one less PWM pin to allocate
      write(0);           // initialize the duty cycle
      break;
    }
  }
  return(found);
}

// delete PWM pin reference and float driver (no brake), instance becomes available
void DCM_PWM_AP::disable(void) {
  //Free Pins
  pinMode(_nEN, INPUT_PULLUP);
  pinMode(_DIR, INPUT);
  pinMode(_pin, INPUT);
  for (uint8_t i = 0; i < MAX_PWM_PIN; i++){
    if (_cbInstance[i] == this){
      _cbInstance[i] = nullptr;         // erase ourselves from the slot
      if (_pinCount > 0) _pinCount--;   // one slot is now free
      break;
    }
  }
}

// Write raw PWM driving value 0-255.
void DCM_PWM_AP::write(uint8_t duty){
  _pwmDutyPS = duty;  // save the new value Before synchronization at next time slot 0
}

// set Motor Velocity is a -100 to 100 value that defines the duty cycle of the PWM in % ,
// where the sign determines the direction. Values below -100 or above 100, are constrained to these limits respectivelly. 
void DCM_PWM_AP::setVelocity(int8_t velocity){
  boolean reverse = false;
  int speed; 
  velocity = constrain(velocity, -100, 100);
  if(velocity >=0)
    speed = 255*velocity/100;
  else{
    speed = -255*velocity/100;
    reverse=1;
  }
  write(speed); 
  if (reverse ^ _flip) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_DIR, HIGH);
  else
    digitalWrite(_DIR, LOW);
}

//Flip reverses the polarity of direction, useful when and motor instance must run reversed respstect another one (direct drive). 
void DCM_PWM_AP::flip(bool flip) 
{
  _flip = flip;
}

// -------------------------------------------------------------
// ---- Interrupt and Hardware Management
// -------------------------------------------------------------

// Handle the actual PWM pin active time or duty cycle (Called directly from ISR)
void DCM_PWM_AP::setPin(void){
  if (gCnt==0){
    _pwmDuty=_pwmDutyPS; //Get new duty sync with global count
    if (_pwmDuty){
      *_outReg |= _outRegMask;    //Fast digitalWrite HIGH (800nS vs 6us)
    }
  }
  if (gCnt == _pwmDuty && _pwmDuty != 0xff)
    *_outReg &= ~_outRegMask; // Fast digitalWrite LOW (800nS vs 6us)   
}

// ISR for the timer
//WARNING ABOUT FREQUENCY:
//This ISR runs 256 times per Pulse cycle (i.e at 75Hz= 13.3mS / 256 = 52.1uS)
#if USE_TIMER == 1
ISR(TIMER1_OVF_vect){
#elif USE_TIMER == 2
ISR(TIMER2_OVF_vect){
#endif
  for (uint8_t i = 0; i < DCM_PWM_AP::MAX_PWM_PIN; i++)
    if (DCM_PWM_AP::_cbInstance[i] != nullptr) DCM_PWM_AP::_cbInstance[i]->setPin();
  gCnt++; // Global counter to set desired duty 
}

//Timer handling
inline void DCM_PWM_AP::setTimerMode(void){
#if USE_TIMER == 1
  TCCR1B = _BV(WGM13);
#elif USE_TIMER == 2
  TCCR2B = _BV(WGM22);
#endif
}

// Set the timer to count closest to the required frequency * 256.
void DCM_PWM_AP::setFrequency(uint32_t freq){
  uint8_t scale = 0;
  // The counter runs backwards after TOP, interrupt is at BOTTOM -
  // so multiply frequency by 256 (<<8) divide cycles by 2
  uint32_t cycles = (F_CPU / (freq << 8))/2;
#if USE_TIMER == 1
  // Work out the prescaler for this number of cycles
  if (cycles < TIMER_RESOLUTION) scale = _BV(CS10);              // prescale /1 (full xtal)
  else if ((cycles >>= 3) < TIMER_RESOLUTION) scale = _BV(CS11);              // prescale /8
  else if ((cycles >>= 3) < TIMER_RESOLUTION) scale = _BV(CS11) | _BV(CS10);  // prescale /64
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS12);              // prescale /256
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS12) | _BV(CS10);  // prescale /1024
  else     // request was out of bounds, set as maximum
  {
    cycles = TIMER_RESOLUTION - 1;
    scale = _BV(CS12) | _BV(CS10);
  }
  // now set up the counts
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));  // clear prescaler value
  TCCR1B |= scale;
  OCR1A = cycles;     // OCR1A is TOP in phase correct pwm mode for Timer1
  TCNT1 = 0;
#elif USE_TIMER == 2
  // Work out the prescaler for this number of cycles
  if      (cycles         < TIMER_RESOLUTION) scale = _BV(CS20);              // prescale /1 (full xtal)
  else if ((cycles >>= 3) < TIMER_RESOLUTION) scale = _BV(CS21);              // prescale /8
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS21) | _BV(CS20);  // prescale /32
  else if ((cycles >>= 1) < TIMER_RESOLUTION) scale = _BV(CS22);              // prescale /64
  else if ((cycles >>= 1) < TIMER_RESOLUTION) scale = _BV(CS22) | _BV(CS20);  // prescale /128
  else if ((cycles >>= 1) < TIMER_RESOLUTION) scale = _BV(CS22) | _BV(CS21);  // prescale /256 
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS22) | _BV(CS21) | _BV(CS20); // prescale by /1024
  else     // request was out of bounds, set as maximum
  {
    cycles = TIMER_RESOLUTION - 1;
    scale = _BV(CS22) | _BV(CS21) | _BV(CS20);
  }
  // now set up the counts
  TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));  // clear prescaler value
  TCCR2B |= scale;
  OCR2A = cycles;     // OCR2A is TOP in phase correct pwm mode for Timer2
  TCNT2 = 0;
#endif
}

// Start the timer and enable interrupt
inline void DCM_PWM_AP::attachISR(void){
  // set timer overflow interrupt enable bit
#if USE_TIMER == 1
  TIMSK1 = _BV(TOIE1);
#elif USE_TIMER == 2
  TIMSK2 = _BV(TOIE2);
#endif
  sei();                // interrupts globally enabled
}

// Disable the timer interrupt 
inline void DCM_PWM_AP::detachISR(void){
  // clears timer overflow interrupt enable bit 
#if USE_TIMER == 1
  TIMSK1 &= ~_BV(TOIE1);
#elif USE_TIMER == 2
  TIMSK2 &= ~_BV(TOIE2);
#endif
}

// Stop the timer
inline void DCM_PWM_AP::stop(void){
  // clears all clock selects bits
#if USE_TIMER == 1
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
#elif USE_TIMER == 2
  TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
#endif
}
