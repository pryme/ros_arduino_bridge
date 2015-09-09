/* *************************************************************
Encoder definitions

Add a "#if defined" block to this file to include support for
a particular encoder board or library. Then add the appropriate
#define near the top of the main ROSArduinoBridge.ino file.

Modified by PR:  2015-5-9

************************************************************ */

#ifdef USE_BASE

#if defined ROBOGAIA
/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* Create the encoder shield object */
MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

/* Wrap the encoder reading function */
long readEncoder(int i) {
	if (i == LEFT) return encoders.YAxisGetCount();
	else return encoders.XAxisGetCount();
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
	if (i == LEFT) return encoders.YAxisReset();
	else return encoders.XAxisReset();
}
#elif defined ARDUINO_ENC_COUNTER
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect){
	static uint8_t enc_last=0;
	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT1_vect){
	static uint8_t enc_last=0;
	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
	if (i == LEFT) return left_enc_pos;
	else return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
	if (i == LEFT){
		left_enc_pos=0L;
		return;
	} else { 
		right_enc_pos=0L;
		return;
	}
}

#elif defined PR_TEENSY_ENC_COUNTER
volatile boolean EL_isFresh = true;  // flag fresh interval value from left enc interrupt
volatile boolean ER_isFresh = true;    
volatile long EL_last_interval = 0;  // most recent interval (us), left encoder
volatile long ER_last_interval = 0;  
volatile boolean EL_keep = false; // flag to save only every other micros count
volatile boolean ER_keep = false;  
elapsedMicros EL_micros = 0;  // counts micros for left encoder
elapsedMicros ER_micros =0;  
volatile long EL_ticks = 0L;  // accumulates tick count for left encoder
volatile long ER_ticks = 0L;
// TODO:  check whether any of the above get used out of this scope.

ISR(INT0_vect){  // interrupt routine for left encoder
	if (EL_keep){  // only save interval every other tick to eliminate comparator asymmetry
		EL_last_interval = EL_micros;  // save the interval (us)
		EL_micros = 0;  // reset the elapssedMicros object
		EL_isFresh = true;  // set fresh data flag
	}
	EL_keep = !EL_keep;  // toggle so only check interval every second tick
	
    if (leftPID.motor_dir == 1) {
        EL_ticks += 1L;  // increment the tick count every tick
    } else {
        EL_ticks -= 1L;  // if reverse dir
    }
	// TODO:  there might be a faster way using bit shifting
}
ISR(INT1_vect){  // interrupt routine for right encoder
	if (ER_keep){  // only save interval every other tick to eliminate comparator asymmetry
		ER_last_interval = ER_micros;  // save the interval (us)
		ER_micros = 0;  // reset the elapssedMicros object
		ER_isFresh = true;  // set fresh data flag
	}
	ER_keep = !ER_keep;  // toggle 
    if (rightPID.motor_dir == 1) {
	    ER_ticks += 1L;  // increment the tick count every tick
    } else {
        ER_ticks -=1L;  // if reverse dir
    }
    
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
	if (i == LEFT) {
		return EL_ticks;
	} else {
		return ER_ticks;
	}
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
	if (i == LEFT){
		EL_ticks = 0L;
		return;
	} else { 
		ER_ticks = 0L;
		return;
	}
}

/* Wrap the encoder reset function */
void resetEncoders() {
	resetEncoder(LEFT);
	resetEncoder(RIGHT);
}

/* Return the most recent encoder interval */
long readEncoderInterval(int i){
	if (i == LEFT){
        EL_isFresh = false;
		return EL_last_interval;
	} else { 
        ER_isFresh = false;
		return ER_last_interval;
	}
}  // end readEncoderInterval

/* Return flag indicating if interval data is fresh */
boolean isFreshEncoderInterval(int i){
	if (i == LEFT){
		return EL_isFresh;
	} else { 
		return ER_isFresh;
	}
}  // end isFreshEncoderInterval

/* Initialize the Teensy pin modes and interrupts for PR encoders */
void initEncoders(){
	pinMode(LEFT_ENC_PIN , INPUT);
	pinMode(RIGHT_ENC_PIN , INPUT);  
	// enable interrupts
	EIMSK |= (1<<0);  // set bit 0 of EIMSK to enable INT0
	EIMSK |= (1<<1);  // set bit 1 of EIMSK to enable INT1
	EICRA |= (1<<0);  // set bit 0 of EICRA - this and following line define edge trigger
	EICRA &= ~(1<<1);  // clear bit 1 of EICRA - edge trigger for INT0
	EICRA |= (1<<2);  // set bit 0 of EICRA - this and following line define edge trigger
	EICRA &= ~(1<<3);  // clear bit 1 of EICRA - edge trigger for INT1
	sei();  // global interrupt enable
    resetEncoders();	
}  // end initEncoders

#else
#error An encoder driver must be selected!
#endif

#endif

