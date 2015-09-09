/***************************************************************
Motor driver definitions

Add a "#elif defined" block to this file to include support
for a particular motor driver.  Then add the appropriate
#define near the top of the main ROSArduinoBridge.ino file.

Modified by PR:  2015-5-9


*************************************************************/

#ifdef USE_BASE

#if defined POLOLU_VNH5019
/* Include the Pololu library */
#include "DualVNH5019MotorShield.h"

/* Create the motor driver object */
DualVNH5019MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
	drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
	if (i == LEFT) drive.setM1Speed(spd);
	else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
	setMotorSpeed(LEFT, leftSpeed);
	setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined POLOLU_MC33926
/* Include the Pololu library */
#include "DualMC33926MotorShield.h"

/* Create the motor driver object */
DualMC33926MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
	drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
	if (i == LEFT) drive.setM1Speed(spd);
	else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
	setMotorSpeed(LEFT, leftSpeed);
	setMotorSpeed(RIGHT, rightSpeed);
}


#elif defined PR_TEENSY_MOTOR_DRIVER

/* pin map - could be in a new .h */
int ML1A_pin = PIN_B0;  // H-bridge 1A in --> Left motor, red wire
int ML2A_pin = PIN_B1;  // H-bridge 2A in --> Left motor, black wire
int MR3A_pin = PIN_B2;  // H-bridge 3A in --> Right motor, red wire
int MR4A_pin = PIN_B3;  // H-bridge 4A in --> Right motor, black wire
int MLPWM_pin = PIN_C7;  // Left motor PWM pin
int MRPWM_pin = PIN_D7;

/* initialize the motor driver stuff */
void initMotorController(){
	// set pin modes and initial states
	pinMode(ML1A_pin, OUTPUT);
	pinMode(ML2A_pin, OUTPUT);
	pinMode(MR3A_pin, OUTPUT);
	pinMode(MR4A_pin, OUTPUT);
	pinMode(MLPWM_pin, OUTPUT);
	pinMode(MRPWM_pin, OUTPUT);  
	analogWrite(MLPWM_pin, 0);  // speed setpoints = 0
	analogWrite(MRPWM_pin, 0);  
	digitalWrite(ML1A_pin, HIGH);  // set braking 
	digitalWrite(ML2A_pin, HIGH);
	digitalWrite(MR3A_pin, HIGH);
	digitalWrite(MR4A_pin, HIGH);      
}

/* Set LEFT motor speed */
void setM1Speed(int spd){  // LEFT motor
	// spd is an int between -255 (reverse) and +255 (forward)
	// patterned after Pololu DualMC33926MotorShield.cpp from github
	unsigned char reverse = 0;
	if (spd < 0) {
		spd = -spd;
		reverse = 1;  // save direction		
	}
	if (spd > 255){
		spd = 255;  // max PWM setting
	}
	if (reverse == 1){  // set direction on H-bridge
	    digitalWrite(ML1A_pin, HIGH);
        digitalWrite(ML2A_pin, LOW);	
	} else {  // forward
		digitalWrite(ML1A_pin, LOW);
        digitalWrite(ML2A_pin, HIGH);
	}
	analogWrite(MLPWM_pin, spd);  // speed setpoint
}

/* set RIGHT motor speed */
void setM2Speed(int spd){  // RIGHT motor
	// spd is an int between -255 (reverse) and +255 (forward)
	// patterned after Pololu DualMC33926MotorShield.cpp from github
	unsigned char reverse = 0;
	if (spd < 0) {
		spd = -spd;
		reverse = 1;  // save direction		
	}
	if (spd > 255){
		spd = 255;  // max PWM setting
	}
	if (reverse == 1){  // set direction on H-bridge
	    digitalWrite(MR4A_pin, LOW);
        digitalWrite(MR3A_pin, HIGH);	
	} else {  // forward
	    digitalWrite(MR4A_pin, HIGH);
        digitalWrite(MR3A_pin, LOW);
    }
	analogWrite(MRPWM_pin, spd);  // speed setpoint
} 
	
/* Wrap the motor set speed function */
void setMotorSpeed(int i, int spd) {
	if (i == LEFT) {
		setM1Speed(spd);
	} else {
		setM2Speed(spd);
	}
}	
	
// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
	setMotorSpeed(LEFT, leftSpeed);
	setMotorSpeed(RIGHT, rightSpeed);
}

#else
#error A motor driver must be selected!
#endif

#endif
