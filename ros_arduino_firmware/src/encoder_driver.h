/* *************************************************************
   Encoder driver function definitions - by James Nugen
   
   Modified by PR;  2015-5-9
   ************************************************************ */
   
   
#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined PR_TEENSY_ENC_COUNTER
  // define the pins for reading encoder
  #define LEFT_ENC_PIN PIN_D0  // will use INT0
  #define RIGHT_ENC_PIN PIN_D1  // will use INT1
  elapsedMicros EL_micros = 0;  // counts interval for left encoder in microsec
  elapsedMicros ER_micros =0;  
  elapsedMicros EL_waiting = 0;  // microsec since last encoder interrupt
  elapsedMicros ER_waiting = 0;
  unsigned long E_timeout = 100000;  // if no encoder int in this time, assume stall
  // E_timeout operates with single tick interval
#else
  #error An encoder driver must be selected!
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
long readEncoderInterval(int i);
boolean isFreshEncoderInterval(int i);
void initEncoders();

// TODO:  review if I declared all new functions

#endif