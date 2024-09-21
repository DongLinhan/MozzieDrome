/*
  A multi tone Generator Library
  Simplified from Brett Hagman
  http://www.roguerobotics.com/
  bhagman@roguerobotics.com
  
  Simplified and maintaned by R.Hormigo at the ZMBBI Columbia to use only timers 5, 4, 3, and 2 optional in Arduino Mega
  Each timer provides a pin with independent tones that can play simultaneously to others.
  

    This library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*************************************************/
//Comment out below to use also Timer 2 for a 4th pin, other wise only three pins simultaneously using Timer 5, Timer 4, and Timer 3 
//(No compatible with PWM library that also uses timer 2 for DC motors)

// #define T2   //Timer 2


#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <pins_arduino.h>
#include "Tone.h"

#ifdef T2
volatile int32_t timer2_toggle_count;
volatile uint8_t *timer2_pin_port;
volatile uint8_t timer2_pin_mask;
#endif

volatile int32_t timer3_toggle_count;
volatile uint8_t *timer3_pin_port;
volatile uint8_t timer3_pin_mask;
volatile int32_t timer4_toggle_count;
volatile uint8_t *timer4_pin_port;
volatile uint8_t timer4_pin_mask;
volatile int32_t timer5_toggle_count;
volatile uint8_t *timer5_pin_port;
volatile uint8_t timer5_pin_mask;


#ifdef T2
  #define AVAILABLE_TONE_PINS 4 //If T2 is defined, and aditional 4th pin is possible
  const uint8_t PROGMEM tone_pin_to_timer_PGM[] = {2, 3, 4, 5}; //Use from timer 5 down to 2
  // Initialize our pin count
  uint8_t Tone::_tone_pin_count = 3; //Start in last counter 5 and go down 4, 3, 2... overlap 
#else
  #define AVAILABLE_TONE_PINS 3 //If T2 is defined, and aditional 4th pin is possible
  const uint8_t PROGMEM tone_pin_to_timer_PGM[] = {3, 4, 5}; //Use from timer 5 down to 2
  // Initialize our pin count
  uint8_t Tone::_tone_pin_count = 2; //Start in last counter 5 and go down 4, 3, 2... overlap 
#endif
void Tone::begin(uint8_t tonePin)
{
  if (_tone_pin_count < AVAILABLE_TONE_PINS)
  {
    _pin = tonePin;
    _timer = pgm_read_byte(tone_pin_to_timer_PGM + _tone_pin_count);
    //Cicle timers to pins
    if (_tone_pin_count) //keep counting down to 0
      _tone_pin_count--;
    else
      _tone_pin_count=AVAILABLE_TONE_PINS-1;

    // Set timer specific stuff
    // All timers in CTC mode
    // 8 bit timers will require changing prescalar values,
    // whereas 16 bit timers are set to either ck/1 or ck/64 prescalar
    switch (_timer)
    {
#ifdef T2
      case 2:
        // 8 bit timer
        TCCR2A = 0;
        TCCR2B = 0;
        bitWrite(TCCR2A, WGM21, 1);
        bitWrite(TCCR2B, CS20, 1);
        timer2_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer2_pin_mask = digitalPinToBitMask(_pin);
        break;
#endif
      case 3:
        // 16 bit timer
        TCCR3A = 0;
        TCCR3B = 0;
        bitWrite(TCCR3B, WGM32, 1);
        bitWrite(TCCR3B, CS30, 1);
        timer3_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer3_pin_mask = digitalPinToBitMask(_pin);
        break;
      case 4:
        // 16 bit timer
        TCCR4A = 0;
        TCCR4B = 0;
        bitWrite(TCCR4B, WGM42, 1);
        bitWrite(TCCR4B, CS40, 1);
        timer4_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer4_pin_mask = digitalPinToBitMask(_pin);
        break;
      case 5:
        // 16 bit timer
        TCCR5A = 0;
        TCCR5B = 0;
        bitWrite(TCCR5B, WGM52, 1);
        bitWrite(TCCR5B, CS50, 1);
        timer5_pin_port = portOutputRegister(digitalPinToPort(_pin));
        timer5_pin_mask = digitalPinToBitMask(_pin);
        break;
    }
  }
  else
  {
    // disabled
    _timer = -1;
  }
}


// frequency (in hertz) and duration (in milliseconds).

void Tone::play(uint16_t frequency, uint32_t duration)
{
  uint8_t prescalarbits = 0b001;
  int32_t toggle_count = 0;
  uint32_t ocr = 0;

  if (_timer >= 0)
  {
    // Set the pinMode as OUTPUT
    pinMode(_pin, OUTPUT);
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    if (_timer == 0 || _timer == 2)
    {
      ocr = F_CPU / frequency / 2 - 1;
      prescalarbits = 0b001;  // ck/1: same for both timers
      if (ocr > 255)
      {
        ocr = F_CPU / frequency / 2 / 8 - 1;
        prescalarbits = 0b010;  // ck/8: same for both timers

        if (_timer == 2 && ocr > 255)
        {
          ocr = F_CPU / frequency / 2 / 32 - 1;
          prescalarbits = 0b011;
        }

        if (ocr > 255)
        {
          ocr = F_CPU / frequency / 2 / 64 - 1;
          prescalarbits = _timer == 0 ? 0b011 : 0b100;

          if (_timer == 2 && ocr > 255)
          {
            ocr = F_CPU / frequency / 2 / 128 - 1;
            prescalarbits = 0b101;
          }

          if (ocr > 255)
          {
            ocr = F_CPU / frequency / 2 / 256 - 1;
            prescalarbits = _timer == 0 ? 0b100 : 0b110;
            if (ocr > 255)
            {
              // can't do any better than /1024
              ocr = F_CPU / frequency / 2 / 1024 - 1;
              prescalarbits = _timer == 0 ? 0b101 : 0b111;
            }
          }
        }
      }


        TCCR2B = (TCCR2B & 0b11111000) | prescalarbits;
    }
    else
    {
      // two choices for the 16 bit timers: ck/1 or ck/64
      ocr = F_CPU / frequency / 2 - 1;

      prescalarbits = 0b001;
      if (ocr > 0xffff)
      {
        ocr = F_CPU / frequency / 2 / 64 - 1;
        prescalarbits = 0b011;
      }
      if (_timer == 3)
          TCCR3B = (TCCR3B & 0b11111000) | prescalarbits;
      else if (_timer == 4)
          TCCR4B = (TCCR4B & 0b11111000) | prescalarbits;
      else if (_timer == 5)
          TCCR5B = (TCCR5B & 0b11111000) | prescalarbits;
    }
    
    // Calculate the toggle count
    if (duration > 0)
    {
      toggle_count = 2 * frequency * duration / 1000;
    }
    else
    {
      toggle_count = -1;
    }

    // Set the OCR for the given timer,
    // set the toggle count,
    // then turn on the interrupts
    switch (_timer)
    {
#ifdef T2
      case 2:
        OCR2A = ocr;
        timer2_toggle_count = toggle_count;
        bitWrite(TIMSK2, OCIE2A, 1);
        break;
#endif
      case 3:
        OCR3A = ocr;
        timer3_toggle_count = toggle_count;
        bitWrite(TIMSK3, OCIE3A, 1);
        break;
      case 4:
        OCR4A = ocr;
        timer4_toggle_count = toggle_count;
        bitWrite(TIMSK4, OCIE4A, 1);
        break;
      case 5:
        OCR5A = ocr;
        timer5_toggle_count = toggle_count;
        bitWrite(TIMSK5, OCIE5A, 1);
        break;
    }
  }
}


void Tone::stop()
{
  switch (_timer)
  {
#ifdef T2
    case 2:
      TIMSK2 &= ~(1 << OCIE2A);
      break;
#endif
    case 3:
      TIMSK3 &= ~(1 << OCIE3A);
      break;
    case 4:
      TIMSK4 &= ~(1 << OCIE4A);
      break;
    case 5:
      TIMSK5 &= ~(1 << OCIE5A);
      break;
  }
  digitalWrite(_pin, 0);
}


bool Tone::isPlaying(void)
{
  bool returnvalue = false;
  
  switch (_timer)
  {
#ifdef T2
    case 2:
      returnvalue = (TIMSK2 & (1 << OCIE2A));
      break;
#endif
    case 3:
      returnvalue = (TIMSK3 & (1 << OCIE3A));
      break;
    case 4:
      returnvalue = (TIMSK4 & (1 << OCIE4A));
      break;
    case 5:
      returnvalue = (TIMSK5 & (1 << OCIE5A));
      break;
  }
  return returnvalue;
}



#ifdef T2
ISR(TIMER2_COMPA_vect)
{
  int32_t temp_toggle_count = timer2_toggle_count;

  if (temp_toggle_count != 0)
  {
    // toggle the pin
    *timer2_pin_port ^= timer2_pin_mask;

    if (temp_toggle_count > 0)
      temp_toggle_count--;
  }
  else
  {
    TIMSK2 &= ~(1 << OCIE2A);                 // disable the interrupt
    *timer2_pin_port &= ~(timer2_pin_mask);   // keep pin low after stop
  }
  
  timer2_toggle_count = temp_toggle_count;
}
#endif

ISR(TIMER3_COMPA_vect)
{
  if (timer3_toggle_count != 0)
  {
    // toggle the pin
    *timer3_pin_port ^= timer3_pin_mask;

    if (timer3_toggle_count > 0)
      timer3_toggle_count--;
  }
  else
  {
    TIMSK3 &= ~(1 << OCIE3A);                 // disable the interrupt
    *timer3_pin_port &= ~(timer3_pin_mask);   // keep pin low after stop
  }
}

ISR(TIMER4_COMPA_vect)
{
  if (timer4_toggle_count != 0)
  {
    // toggle the pin
    *timer4_pin_port ^= timer4_pin_mask;

    if (timer4_toggle_count > 0)
      timer4_toggle_count--;
  }
  else
  {
    TIMSK4 &= ~(1 << OCIE4A);                 // disable the interrupt
    *timer4_pin_port &= ~(timer4_pin_mask);   // keep pin low after stop
  }
}

ISR(TIMER5_COMPA_vect)
{
  if (timer5_toggle_count != 0)
  {
    // toggle the pin
    *timer5_pin_port ^= timer5_pin_mask;

    if (timer5_toggle_count > 0)
      timer5_toggle_count--;
  }
  else
  {
    TIMSK5 &= ~(1 << OCIE5A);                 // disable the interrupt
    *timer5_pin_port &= ~(timer5_pin_mask);   // keep pin low after stop
  }
}

