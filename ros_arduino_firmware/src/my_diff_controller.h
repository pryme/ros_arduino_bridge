/**********************************************
* A set of functions that can interface with RAB to output a PID controlled 
*   value to motors based on encoder readings.
* The PID control operates on the time interval between the ticks of my low
*   resolution encoders using a velocity form of the PID algorithm.
*  
************************************************/
/* Interface to ROSArduinoBridge.ino  
 *   ---- What does ROSArduinoBridge call? ----
 *     updatePID() in loop()
 *     setMotorSpeeds() in loop()
 *     leftPID.TargetTicksPerFrame...from serial set setpoint for ticks / frame (sec)
 *       Note 16bit int input so limit of 32767;
 *     initEncoders() in setup()
 *     initMotorController() in setup()
 *     resetPID() in setup()
 *   ---- What globals in RAB are part of the interface? ----
 *     int Kp, Ki, Kd, Ko;
 *     unsigned char moving (declared in my_diff_controller.h)
 *   ---- What are the key #includes? ----
 *     motor_driver.h; encoder_driver.h, diff_controller.h (or equiv)
 *   ---- What restrictions due to serial commands? ----
 *     m n1, n2:  n1, n2 are 16bit ints (max 32767)
 *       note about that:  my motor_driver expects in range -255:255
 ************************************************/
/* Interface to encoder interval (interrupt-driven)
 *   ---- Functions in encoder_driver.ino: ----
 *     unsigned long readEncoderInterval(int i);
 *     boolean isFreshEncoderInterval(int i);
 *
 *     This is the interface to 'pv' ('process value' in PID lingo)
 ************************************************/
/* Interface to motor PWM 
 *   ---- Functions in motor_driver.ino: ----
 *     void setMotorSpeed(int i, int spd);
 *     void setMotorSpeeds(int leftSpeed, int rightSpeed);
 *
 *     This is the interface to 'co' ('controller output' in PID lingo)
 ************************************************/
/* TODO: for this version -----------------------------------------
 * are units for motor drive args right (esp factor of 2 on encoder rez?)
 * Decide whether to add ADC code for batt voltage here or ROS via sensors;
 * Document changes required to RAB
 * Document the velocity PID algorithm
 *------------------------------------------------------------------*/


#ifndef my_diff_controller_h
#define my_diff_controller_h

/* enable 'if (DEBUG) Serial.println();' or similar */
#define DEBUG (1==0)

/* globals declared here */
const int kMaxPWM = 255;      // hard-code for now
const int kMinPWM = -255;     // needed for startup case
const float kSpdMax = 3.0;    // revs / sec wheel speed
const float kSpdMin = 0.4;    // revs / sec wheel speed
const int kEncoderSegs = 48;  // encoder ticks / wheel rev
unsigned char moving = 0;  // is base in motion?;  used by RosArduinoBridge.ino

float dt_min = (1000000.0 / kSpdMax / kEncoderSegs * 2.0);  // microsec / double-tick
float dt_max = (1000000.0 / kSpdMin / kEncoderSegs * 2.0); 
float dt_mid = (dt_min + dt_max) / 2.0;  // midpoint of dt range
bool controller_reversed = true;  // true: higher motor drive DECREASES enc. interval
int Kp = 10;  // needed for RAB command interface;  
int Ki = 6;  
int Kd = 0;
int Ko = 6000;
float tau_i = 1.0e5;
float tau_d = 0.0;
bool PIDs_are_reset = false;  

/* PID data structure for a side (LEFT or RIGHT) */
typedef struct {
    /* TargetTicksPerFrame - needed public property for RAB interface 
       Name must be exact to support RAB interface.
       Will be set by serial command.  
       Will be converted to PID setpoint value. 
       Positive values drive forward; negative drive reverse. */
    float TargetTicksPerFrame;    // target speed in ticks / second
      // for the above to work, BaseController.py must be modified in cmdVelCallback().
      //TODO:  clarify above comment
      // r_a_b PID_INTERVAL is fixed at 1000/30 Hz ms.
    float pv;  // process value: current encoder interval (double-tick)
      // Note process value can ONLY be positive with current encoders.  So need 
      // to keep track of fwd/rev direction separately.
    float co;  // controller output: PWM value in 0 to +255
      // in this version pv and co are always positive;  direction handled separately
    float sp;  // setpoint value:  target encoder interval (double-tick)
      // in this version sp is always positive
    float pv_old1;      // previous pv val
    float pv_old2;      // previous pv_old1 val
    float error;        // current error val
    float error_old1;   // previous error val
    int motor_dir;     // currently commanded motor direction; +1(fwd) or -1(rev)
    // include k's and tau's so L could have differnt k's than R?
    // TODO: should I include encoder ticks?
}
PIDData;

PIDData leftPID, rightPID;  // exact names required for RAB interface

/* resetPID ---------------------------------------------------
 * Initializes PID data; called directly by RAB.
 * -----------------------------------------------------------*/
void resetPID(void){
    leftPID.TargetTicksPerFrame = 0.0;  // will get set by serial
    leftPID.pv = dt_max; // initialize to slowest rate
    leftPID.co = 0.0;   
    leftPID.sp = 0.0;  
    leftPID.pv_old1 = leftPID.pv;  
    leftPID.pv_old2 = leftPID.pv; 
    leftPID.error = 0.0;  
    leftPID.error_old1 = 0.0;
    leftPID.motor_dir = 1;  // default to forward; will get set by serial
    
    rightPID.TargetTicksPerFrame = 0.0;
    rightPID.pv = dt_max;
    rightPID.co = 0.0;
    rightPID.sp = 0.0;
    rightPID.pv_old1 = rightPID.pv;
    rightPID.pv_old2 = rightPID.pv;
    rightPID.error = 0.0;
    rightPID.error_old1 = 0.0;
    rightPID.motor_dir = 1;  
    
    PIDs_are_reset = true;
 }  // end resetPID
 
/* doPID -----------------------------------------------------
 * Computes new output for motor driver.
 * Called directly by RAB.
 *------------------------------------------------------------*/
void doPID(PIDData *pid) {  
    /* K's can be changed by serial command */
    if (Kp == 0) {Kp = 1;}  // prevent divide by zero and excessively large tau_d
    // assuming above that serial command will only pass int val
    tau_d = Kd / Kp * dt_mid;
    if (Ki > 1.0e-5) {             // to prevent divide by zero
          tau_i = Kp / Ki * dt_mid;  // TODO:  experiment with dt_mid?
      } else {
          tau_i = 1.0e5;  // arbitrary max
      }
        
    float co_delta = 0.0;  // initialize increment to current co
    float speed;  // positive speed magnitude
    // need to check every time since TTPF can be changed by serial
    if (pid->TargetTicksPerFrame >= 0) {  
        pid->motor_dir = 1;  // keep track of commanded direction
        speed = pid->TargetTicksPerFrame;
    } else {
        pid->motor_dir = -1;
        speed = -pid->TargetTicksPerFrame;  
        }
        
    pid->sp = ((2.0 * 1000000L) / speed);  // serial can chg TTPF
    
    pid->error_old1 = pid->error;  // save prev value
    pid->error = pid->sp - pid->pv;  
    // below computes various terms in PID algorithm, then combines them    
    float a_term = ((1 + dt_mid / tau_i) * pid->error) / Ko;
    float b_term = (tau_d / dt_mid) * pid->pv / Ko;
    float c_term = a_term - b_term - pid->error_old1 / Ko;
    float d_term = (2 * pid->pv_old1 - pid->pv_old2) * (tau_d / dt_mid) / Ko;
    co_delta = Kp * (c_term + d_term); 
    if (controller_reversed) {co_delta = 0.0 - co_delta;}  // means inverse action
    pid->co += co_delta;  // adjust the controller output
    if (pid->co <= 0){  // limit co to positive values
        pid->co = 0;
    }
    if (pid->co > kMaxPWM) {  // limit to max possible output
      pid->co = kMaxPWM;
    } 
}  // end doPID

/* updatePID  -------------------------------------------------
  *   reads encoders, uses co from doPID to set new motor speeds;
  *   called directly by RAB;
  ------------------------------------------------------------*/
void updatePID(void){  
    if (!moving) {   // there's no command to move
        if (!PIDs_are_reset) {resetPID(); }
        EL_micros = 0;  // reset encoder interval every update until commanded to move
        ER_micros = 0; 
        return;  // nothing more to do
    }
    
    /* If commanded and just starting, get motors moving so interrupts can happen */
    if (moving && PIDs_are_reset) {  // we're just starting;  moving set by RAB
        PIDs_are_reset = false;
        /*
        float deadband = 5.0;  // ignore if abs(speed) <= deadband
        int left_speed = 0;  // just for starting
        int right_speed = 0;
        if (leftPID.TargetTicksPerFrame > deadband){
            left_speed = kMaxPWM;  // set max to overcome stiction
            leftPID.motor_dir = 1;
        } else if (leftPID.TargetTicksPerFrame < -deadband) {
            left_speed = kMinPWM;
            leftPID.motor_dir = -1;
        } else {
            left_speed = 0;
            leftPID.motor_dir = 1;
        }
        if (rightPID.TargetTicksPerFrame > deadband){
            right_speed = kMaxPWM;
            rightPID.motor_dir = 1;
        } else if (rightPID.TargetTicksPerFrame < -deadband) {
            right_speed = kMinPWM;
            rightPID.motor_dir = -1;
        } else {
            right_speed = 0;
            rightPID.motor_dir = 1;
        }
        setMotorSpeeds(left_speed, right_speed);
      /* TODO:  remove debug code
      if (DEBUG){
          Serial.print("init spds:  "); Serial.print(left_speed); 
          Serial.print("; "); Serial.println(right_speed);
        }
      */  
     /*   
        for (int iii = 0; iii < 15; iii++){
          delayMicroseconds(15000);  // time to overcome stiction;
          // not sure this is still needed, or what min time needed is
          // delayMicroseconds fn can't go beyond ~16383 us
        }
     */
    }  // end if !moving 
    
    if (isFreshEncoderInterval(LEFT)){
      leftPID.pv_old2 = leftPID.pv_old1;  // save prior values
      leftPID.pv_old1 = leftPID.pv;      
      cli();  // interrupts off
      leftPID.pv = readEncoderInterval(LEFT);
      sei();  // interrupts on
      if (leftPID.pv > 100000){leftPID.pv = 100000; }  // TODO:  is this needed?
      if (DEBUG) {
        Serial.print("L DT int (pv): "); Serial.print(leftPID.pv);
      }
      doPID(&leftPID);  // update the PID vals
      setMotorSpeed(LEFT, leftPID.co * leftPID.motor_dir);
      if (DEBUG){
          Serial.print("set L M:  "); Serial.println(leftPID.co * leftPID.motor_dir);
      }
    }
   
   if (EL_waiting > E_timeout) {  // too long since last encoder interrupt
       // drive full-on to break stiction / stall
       setMotorSpeed(LEFT, 255 * leftPID.motor_dir);  
   }
       
   if (isFreshEncoderInterval(RIGHT)){
      rightPID.pv_old2 = rightPID.pv_old1;  // save prior values
      rightPID.pv_old1 = rightPID.pv;      
      cli();  // interrupts off
      rightPID.pv = readEncoderInterval(RIGHT);
      sei();  // interrupts on
      if (rightPID.pv > 100000){rightPID.pv = 100000; }  // TODO:  is this needed?
      if (DEBUG) {
        Serial.print("R DT int (pv): "); Serial.print(rightPID.pv);
      }
      doPID(&rightPID);  // update the PID vals
      setMotorSpeed(RIGHT, rightPID.co * rightPID.motor_dir);
      if (DEBUG){
          Serial.print("; set R M:  "); Serial.println(rightPID.co * rightPID.motor_dir);
      }
   }

   if (ER_waiting > E_timeout) {  // too long since last encoder interrupt
       // drive full-on to break stiction / stall
       setMotorSpeed(RIGHT, 255 * rightPID.motor_dir);  
   }
   
}  // end updatePID()

#endif
