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
 * Remove MANUAL code
 *------------------------------------------------------------------*/


#ifndef my_diff_controller_h
#define my_diff_controller_h

/* enable 'if (DEBUG) Serial.println();' or similar */
#define DEBUG (1==1)
/* enable manual start button for untethered op (should disable DEBUG too) */
#define MANUAL (1==0)

/* globals declared here */
const int kMaxPWM = 255;      // hard-code for now
const int kMinPWM = -255;     // still need for startup case
const float kSpdMax = 3.0;    // revs / sec wheel speed
const float kSpdMin = 0.4;    // revs / sec wheel speed
const int kEncoderSegs = 48;  // encoder ticks / wheel rev
unsigned char moving = 0;  // is the base in motion?

float dt_min = (1000000.0 / kSpdMax / kEncoderSegs * 2.0);  // microsec / double-tick
float dt_max = (1000000.0 / kSpdMin / kEncoderSegs * 2.0); 
float dt_mid = (dt_min + dt_max) / 2.0;  // midpoint of dt range
bool controller_reversed = true;  // true --> control element action is reverse
int Kp = 10;  // needed for RAB command interface;  
int Ki = 6;  // TODO:  set default Ki val
int Kd = 0;
int Ko = 6000;
float tau_i = 1.0e5;
float tau_d = 0.0;
bool PIDs_are_reset = false;  

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
    float TargetTicksPerFrame;    // target speed in ticks / second
    // for the above to work, BaseController.py must be modified in cmdVelCallback().
    // r_a_b PID_INTERVAL is fixed at 1000/30 Hz ms.
    float pv;  // process value == current encoder interval (double-tick)
    // Note process value can ONLY be positive with current encoders.  So need 
    // to keep track of fwd/rev direction separately.
    // need to make positive or negative ok;
    float co;  // controller output == current PWM value in -255 to +255
    // positive:  fwd;  negative: rev;
    float sp;  // setpoint value == target encoder interval (double-tick)
    float pv_old1;      // previous pv val
    float pv_old2;      // previous pv_old1 val
    float error;        // current error val;  don't really need stored?
    float error_old1;   // previous error val
    int motor_dir;     // currently commanded motor direction; +1(fwd) or -1(rev)
    // include k's and tau's?
    // TODO: should I include encoder ticks?
}
PIDData;

PIDData leftPID, rightPID;  // exact names needed for RAB interface

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
    
    // might want to move sp into this section too
        
    float co_delta = 0.0;  // increment to current co
    float speed;  // positive speed magnitude
    if (pid->TargetTicksPerFrame >= 0) {  
        pid->motor_dir = 1;  // keep track of commanded direction
        speed = pid->TargetTicksPerFrame;
    } else {
        pid->motor_dir = -1;
        speed = -pid->TargetTicksPerFrame;  
        }
    // TODO:  could do above as speed=pid->TTPF*pid->motor_dir...?
    
    pid->error_old1 = pid->error;  // save prev value
    pid->sp = ((2.0 * 1000000L) / speed);  // serial can chg TTPF
    
    pid->error = pid->sp - pid->pv;    
    float a_term = ((1 + dt_mid / tau_i) * pid->error) / Ko;
    float b_term = (tau_d / dt_mid) * pid->pv / Ko;
    float c_term = a_term - b_term - pid->error_old1 / Ko;
    float d_term = (2 * pid->pv_old1 - pid->pv_old2) * (tau_d / dt_mid) / Ko;
    co_delta = Kp * (c_term + d_term); 
    if (controller_reversed) {co_delta = 0.0 - co_delta;}  // means rev. action
    pid->co += co_delta;
    if (pid->co > kMaxPWM) {  // limit to max possible output
      pid->co = kMaxPWM;
    } 
    //if (pid->motor_dir < 0) {  // invert if direction is reverse
    //    pid->co = -pid->co;  // TODO:  problem with sign?
    //}
}  // end doPID

/* updatePID  -------------------------------------------------
  *   reads encoders, updates PID calcs, sets new motor speeds;
  *   called directly by RAB;
  ------------------------------------------------------------*/
void updatePID(void){  
    if (MANUAL) {  // temp for manual start switch  TODO:  remove
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
    float deadband = 5.0;  // ignore if abs(speed) <= deadband
    if (moving && PIDs_are_reset) {  // we're just starting
        // set max to overcome stiction
        // using TTPF just to get direction correct (motor_dir isn't set yet).
        int left_speed = 0;  // just for starting
        int right_speed = 0;
        if (leftPID.TargetTicksPerFrame > deadband){
            left_speed = kMaxPWM;
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
        if (DEBUG){
          Serial.print("init spds:  "); Serial.print(left_speed); 
          Serial.print("; "); Serial.println(right_speed);
        }
        PIDs_are_reset = false;
        delayMicroseconds(500000);  // time to overcome stiction; was 250k us
        } 
    digitalWrite(LED_pin, LOW);  // TODO:  temp for manual switch
    
    if (isFreshEncoderInterval(LEFT)){
      leftPID.pv_old2 = leftPID.pv_old1;  // save prior values
      leftPID.pv_old1 = leftPID.pv;      
      cli();  // interrupts off
      leftPID.pv = readEncoderInterval(LEFT);
      sei();  // interrupts on
      if (leftPID.pv > 100000){leftPID.pv = 100000; }  // TODO:  is this needed?
      // leftPID.pv *= leftPID.motor_dir;  // if rev dir invert the interval value
      if (DEBUG) {
        Serial.print("L DT int (pv): "); Serial.println(leftPID.pv);
      }
      doPID(&leftPID);  
      setMotorSpeed(LEFT, leftPID.co * leftPID.motor_dir);
      if (DEBUG){
          //Serial.print("set L M:  "); Serial.println(leftPID.co * leftPID.motor_dir);
      }
    
    }
   
   if (isFreshEncoderInterval(RIGHT)){
      rightPID.pv_old2 = rightPID.pv_old1;  // save prior values
      rightPID.pv_old1 = rightPID.pv;      
      cli();  // interrupts off
      rightPID.pv = readEncoderInterval(RIGHT);
      sei();  // interrupts on
      if (rightPID.pv > 100000){rightPID.pv = 100000; }  // TODO:  is this needed?
      // rightPID.pv *= rightPID.motor_dir;  // if rev dir invert the interval value
      if (DEBUG) {
        Serial.print("R DT int (pv): "); Serial.print(rightPID.pv);
      }
      doPID(&rightPID);  
      setMotorSpeed(RIGHT, rightPID.co * rightPID.motor_dir);
      if (DEBUG){
          Serial.print("; set R M:  "); Serial.println(rightPID.co * rightPID.motor_dir);
      }
   } 
}  // end updatePID()

#endif
