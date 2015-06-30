/**********************************************
* Outline of a set of functions that can interface with RAB
*   and read encoders and output a variable value to motors;
* This version uses struct but no class and only long math;
*  
************************************************/
//
/* Interface to ROSArduinoBridge.ino  *********
 *   What does ROSArduinoBridge call?
 *     updatePID() in loop()
 *     setMotorSpeeds() in loop()
 *     leftPID.TargetTicksPerFrame...from serial set setpoint for ticks / frame (sec)
 *       Note 16bit int input so limit of 32767;
 *     initEncoders() in setup()
 *     initMotorController() in setup()
 *     resetPID() in setup()
 *   What globals in RAB are part of the interface?
 *     int Kp, Ki, Kd, Ko;
 *     int moving (declared in diff_controller.h)
 *   What are the key #includes
 *     motor_driver.h; encoder_driver.h, diff_controller.h (or equiv)
 *   What restrictions due to serial commands?
 *     m n1, n2:  n1, n2 are 16bit ints (max ~ 32767);
 *       note about that:  my motor_driver expects in range -255:255 (I forgot that)
*/
//
/* Interface to encoder interval (interrupt-driven)  ************
 * Functions in encoder_driver.ino:
 *   unsigned long readEncoderInterval(int i);
 *   boolean isFreshEncoderInterval(int i);
 *
 * (So no worries here about globals related to ISRs)
 * This is the interface to 'pv' (which was 'Input' in PID Lib)
******************************************************************/
//
/* Interface to motor PWM  ***************************************
 * Function in motor_driver.ino:
 *   void setMotorSpeed(int i, int spd);
 *   void setMotorSpeeds(int leftSpeed, int rightSpeed);
 *
 * This is the interface to 'co' (which was 'Output' in PID Lib)
*****************************************************************/
//
/* TODO: for this version -----------------------------------------
 * fix so works in reverse with negative inputs
 * are units for motor drive args right (esp factor of 2 on encoder rez?)
 * fix the header guard or file name
 * add new defaults for PID params
 * Sort out why hesitates at start
 * Decide whether to add ADC code for batt voltage here or ROS via sensors;
 * Decide whether to re-visit class architecture
 * Clean up code and file name
 * Document changes to RAB (only 1 #include, I think, unless file = diff.controller.h...)
 * Document the velocity PID algorithm
 * Post about this
 *------------------------------------------------------------------*/


#ifndef PR_diff_controller_h
#define PR_diff_controller_h

/* enable 'if (DEBUG) Serial.println();' or similar */
#define DEBUG (1==0)
/* enable manual start button for untethered op (should disable DEBUG too) */
#define MANUAL (1==0)
//#define AUTO_STOP_INTERVAL 4000
// TODO:  remove above and restore in RAB

/* globals declared in diff_controller.h */
const long kMult = 0x400;  // 1024 multiplier for long math resolution
const int kMaxPWM = 255;      // hard-code for now
const int kMinPWM = -255;
const float kSpdMax = 3.0;    // revs / sec wheel speed
const float kSpdMin = 0.4;    // revs / sec wheel speed
const int kEncoderSegs = 48;  // encoder ticks / wheel rev
unsigned char moving = 0;  // is the base in motion?

long dt_min = (long)(1000000.0 / kSpdMax / kEncoderSegs * 2.0);  // microsec / double-tick
long dt_max = (long)(1000000.0 / kSpdMin / kEncoderSegs * 2.0); 
long dt_mid = (dt_min + dt_max) / 2;  // midpoint of dt range
bool controller_reversed = true;  // true --> control element action is reverse
int Kp = 8;  // needed for RAB command interface;  
int Ki = 8;
int Kd = 0;
int Ko = 6000;
long tau_i = 1e5;
long tau_d = 0;
bool PIDs_are_reset = false;  

/*  apparently the following is ng cause not in loop()
if (MANUAL) {  // temp for manual start switch
  int START_pin = PIN_B4;  // push sw to start untethered; TODO:  temp
  boolean isSwitchPressed = false;
  int LED_pin = PIN_D6;  // on-board LED
}
*/
/* declaring the following temporarily to get on with life */
int START_pin = PIN_B4;  // push sw to start untethered; TODO:  temp
boolean isSwitchPressed = false;
int LED_pin = PIN_D6;  // on-board LED

/* PID data structure for a side (LEFT or RIGHT) */
typedef struct {
    /* TargetTicksPerFrame - needed public property for RAB interface 
       Name must be exact to support RAB interface.
       This will be set by serial command.  
       This will be converted to setpoint value. 
       Positive values are forward; negative are reverse. */
    long TargetTicksPerFrame;    // target speed in ticks / second
    // for the above to work, BaseController.py must be modified in cmdVelCallback().
    // r_a_b PID_INTERVAL is fixed at 1000/30 Hz ms.
    long pv;  // process value == current encoder interval (double-tick)
    // Note process value can ONLY be positive with current encoders.  So need 
    // to keep track of fwd/rev direction separately.
    // need to make positive or negative ok;
    long co;  // controller output == current PWM value in -255 to +255
    // positive:  fwd;  negative: rev;
    long sp;  // setpoint value == target encoder interval (double-tick)
    long pv_old1;      // previous pv val
    long pv_old2;      // previous pv_old1 val
    long error;        // current error val;  don't really need stored?
    long error_old1;   // previous error val
    int motor_dir;     // currently commanded motor direction; +1(fwd) or -1(rev)
    // store k's and tau's?
    // TODO: should I store encoder ticks?
}
PIDData;

PIDData leftPID, rightPID;  // exact names needed for RAB interface

/* resetPID ---------------------------------------------------
 * Initializes PID data; called directly by RAB.
 * -----------------------------------------------------------*/
void resetPID(void){
    /*
    if (DEBUG) {    
      Serial.println(" "); Serial.println("resetPID"); Serial.println(" ");  // TODO:
    }
    */
    leftPID.TargetTicksPerFrame = 0L;  // native scale - will get set by serial so no kMult
    leftPID.pv = dt_max * kMult;  // kMult scale
    leftPID.co = 0L;  // native scale
    leftPID.sp = 0L * kMult;  // kMult scale
    leftPID.pv_old1 = leftPID.pv;  // kMult scale
    leftPID.pv_old2 = leftPID.pv;  // kMult scale
    leftPID.error = 0L * kMult;  // kMult scale
    leftPID.error_old1 = 0L * kMult;  // kMult scale
    leftPID.motor_dir = 1;  // default to forward
    
    rightPID.TargetTicksPerFrame = 0L;
    rightPID.pv = 100000L * kMult;
    rightPID.co = 0L;
    rightPID.sp = 0L * kMult;
    rightPID.pv_old1 = rightPID.pv;
    rightPID.pv_old2 = rightPID.pv;
    rightPID.error = 0L * kMult;
    rightPID.error_old1 = 0L * kMult;
    rightPID.motor_dir = 1;  
    
    PIDs_are_reset = true;
 }  // end resetPID
 
/* doPID -----------------------------------------------------
 * Computes new output for motor driver.
 * Called directly by RAB.
 *------------------------------------------------------------*/
void doPID(PIDData *pid) {  // would a ref be better than ptr?
    /* K's can be changed by serial command */
    // K's, tau's, dt are not scaled by kMult
    if (Kp == 0) {Kp = 1;}  // prevent divide by zero and excessively large tau_d
    // assuming above that serial command will only pass int val
    tau_d = Kd / Kp * dt_mid;
    if (Ki > 1.0e-5) {             // to prevent divide by zero
          tau_i = Kp / Ki * dt_mid;  // TODO:  experiment with dt_mid?
      } else {
          tau_i = 1.0e5;  // arbitrary max
      }
    //
    // might want to move sp into this section too
   
    long co_delta = 0L * kMult;  // increment to current co
    if (DEBUG) {  
      //Serial.print("error_old1= "); Serial.print(pid->error_old1);
      //Serial.print("; error(k)= "); Serial.print(pid->error / kMult);
      Serial.print("; leftPID.pv0= "); Serial.print(pid->pv);  Serial.println("");
      Serial.print("; motor_dir0= "); Serial.print(pid->motor_dir);
    }
    pid->error_old1 = pid->error;  // save prev value
    pid->sp = ((2L * 1000000L) / pid->TargetTicksPerFrame) * kMult;  // serial can chg TTPF
    if (pid->TargetTicksPerFrame > 0) {  // keep track of commanded direction
        pid->motor_dir = 1;
        if (DEBUG){
            //Serial.print("; setg motor_dir to +1 ");
        }
    } else {
            pid->motor_dir = -1;
        if (DEBUG){
            //Serial.print("; setg motor_dir to -1 ");
        }
        }
    if (DEBUG) {    
      //Serial.print("; TTPF= "); Serial.print(pid->TargetTicksPerFrame);
      //Serial.print("; sp(k)= "); Serial.print(pid->sp / kMult);
      //Serial.print("; motor_dir= "); Serial.print(pid->motor_dir);
    }
    pid->error = pid->sp - pid->pv;    
    if (DEBUG) {
      //Serial.print("; error_old1(k)= "); Serial.print(pid->error_old1 / kMult);
      Serial.print("; error(k)= "); Serial.print(pid->error / kMult);
    }
    long a_term = ((1 + dt_mid / tau_i) * pid->error) / Ko;
    if (DEBUG) {
      //Serial.print("; a_term= "); Serial.print(a_term);
    }
    long b_term = (tau_d / dt_mid) * pid->pv / Ko;
    if (DEBUG) {
      //Serial.print("; b_term= "); Serial.print(b_term);
    }
    long c_term = a_term - b_term - pid->error_old1 / Ko;
    if (DEBUG) {
      //Serial.print("; c_term= "); Serial.print(c_term);
    }
    long d_term = (2 * pid->pv_old1 - pid->pv_old1) * (tau_d / dt_mid) / Ko;
    if (DEBUG) {
      //Serial.print("; d_term= "); Serial.print(d_term);
    }
    co_delta = Kp * (c_term + d_term);  // at kMult scale
    if (controller_reversed) {co_delta = 0L - co_delta;}
    if (DEBUG) {
      //Serial.print("; co_delta= "); Serial.print(co_delta);
      Serial.print("; co_delta(k)= "); Serial.print(co_delta / kMult);
    }
    pid->co += (co_delta / kMult);  // return to native scale
    if (DEBUG) {   
      //Serial.print("; TTPF= "); Serial.print(pid->TargetTicksPerFrame);
      Serial.print("; sp(k)= "); Serial.print(pid->sp / kMult);
      //Serial.print("; motor_dir doPID 'done' = "); Serial.print(pid->motor_dir);    
      Serial.print("; co= "); Serial.println(pid->co);
    }
    if (pid->co > kMaxPWM) {  // limit to max possible output
      pid->co = kMaxPWM;
    } else if (pid->co < kMinPWM) {
      pid->co = kMinPWM;
    }
    if (DEBUG) {
        Serial.print("; pid->co final= ");  Serial.print(pid->co);
    }
}  // end doPID

/* updatePID  -------------------------------------------------
  *   reads encoders, updates PID calcs, sets new motor speeds;
  *   called directly by RAB;
  ------------------------------------------------------------*/
void updatePID(void){  
    if (MANUAL) {  // temp for manual start switch
      pinMode(START_pin, INPUT_PULLUP);
      pinMode(LED_pin, OUTPUT);
      if (!isSwitchPressed) {
          if (digitalRead(START_pin) == LOW) {
              isSwitchPressed = true;
              digitalWrite(LED_pin, LOW);
          }
      }
      if (isSwitchPressed) {
        lastMotorCommand = millis(); // TODO:  restore
        moving = 1;
        leftPID.TargetTicksPerFrame = 80;
        rightPID.TargetTicksPerFrame = 120;
        isSwitchPressed = false;
      } 
    }  // end temp for manual start switch 
    
    /* If commanded, get motors moving so interrupts can happen */
    if (!moving) {
        if (!PIDs_are_reset) {resetPID(); }
        return;  // nothing more to do
    }
    
    if (moving && PIDs_are_reset) {  // we're just starting
        // set max to overcome stiction
        // using TTPF just to get direction correct (motor_dir isn't set yet).
        setMotorSpeeds(leftPID.TargetTicksPerFrame * kMaxPWM, rightPID.TargetTicksPerFrame * kMaxPWM);
        PIDs_are_reset = false;
        } 
    digitalWrite(LED_pin, LOW);  // TODO:  temp for manual switch
    
    if (isFreshEncoderInterval(LEFT)){
      leftPID.pv_old2 = leftPID.pv_old1;  // save prior values
      leftPID.pv_old1 = leftPID.pv;      
      if (DEBUG){
          Serial.println(" ");
          Serial.print("; leftPID.motor_dir b4 readE= "); Serial.print(leftPID.motor_dir); 
        //  Serial.print("; leftPID.TTPF b4 readE= "); Serial.print(leftPID.TargetTicksPerFrame);
      }
      cli();  // interrupts off
      leftPID.pv = readEncoderInterval(LEFT);
      sei();  // interrupts on
      if (DEBUG) {
        Serial.print("; leftPID.pv1= "); Serial.print(leftPID.pv);
        Serial.print("; leftPID.motor_dir after readE= "); Serial.print(leftPID.motor_dir); 
      }
      if (leftPID.pv > 100000){leftPID.pv = 100000; }  // avoid overflow
      leftPID.pv *= leftPID.motor_dir;  // if rev dir invert the interval value
      leftPID.pv *= kMult;  // scale
      if (DEBUG) {      
        Serial.print(" leftPID.pv(k)2= "); Serial.print(leftPID.pv / kMult);
      }
      doPID(&leftPID);  
      setMotorSpeed(LEFT, leftPID.co);
    if (DEBUG) {
      //Serial.print("; leftPID.co= "); Serial.print(leftPID.co);
      //Serial.println(" "); 
      //Serial.print("K's:  "); Serial.print(Kp); Serial.print("; ");
      //Serial.print(Ki); Serial.print("; "); Serial.print(Kd);  
      //Serial.print("; ");  Serial.println(Ko);        
    }
   }
   
   if (isFreshEncoderInterval(RIGHT)){
      rightPID.pv_old2 = rightPID.pv_old1;  // save prior values
      rightPID.pv_old1 = rightPID.pv;      
      cli();  // interrupts off
      rightPID.pv = readEncoderInterval(RIGHT);
      sei();  // interrupts on
      if (rightPID.pv > 100000){rightPID.pv = 100000; }  // avoid overflow
      rightPID.pv *= rightPID.motor_dir;  // if rev dir invert the interval value
      rightPID.pv *= kMult;  // scale
      doPID(&rightPID);  
      setMotorSpeed(RIGHT, rightPID.co);
   }
   
}  // end updatePID()

#endif
