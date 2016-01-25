/*  GARRA UFSM Quadruped Controller
    date: 5/24/2015
    code by: Rodrigo da Silva Guerra
 
   License: CC-SA 3.0, feel free to use this code however you'd like.
   Please improve upon it! Let me know how you've made it better.

 */

#include <TimerOne.h>

#define BRAKEVCC 0
#define CW   1 // 01
#define CCW  2 // 10
#define BRAKE 3
#define CS_THRESHOLD 100

int inApin[12] = {   45,   31,   43,   32,   28,   33,   26,   25,   40,   36,   44,   29}; // INA: Clockwise input
int inBpin[12] = {   39,   37,   41,   34,   22,   35,   24,   27,   42,   30,   38,   23}; // INB: Counter-clockwise input
int pwmpin[12] = {   13,   11,   12,    4,    3,   10,    2,    8,    6,    5,    7,    9}; // PWM input

float kp[12]   = {  0.1,  0.1,  0.1,  0.1, 0.05, 0.05,  0.1,  0.1,  0.1, 0.03, 0.05, 0.05};
float ki[12]   = {0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001};
float kd[12]   = { 0.05, 0.05, 0.05, 0.01,0.005,0.001, 0.05, 0.05, 0.05,0.005,0.001,0.005};

int sensor[12] = {   A2,   A3,   A4,   A5,   A6,  A12,   A8,   A9,  A10,  A11,   A7,  A13};
float err[12]  = { +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0};
float erri[12] = { +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0};

float goal[12] = { +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0, +0.0};

int j0[12]     = {  588,  462,  448,  734,  381,  710,  671,  682,  515,  526,  866,  260};
int j1[12]     = {   20,  766,   10,  992,  564,  600,  971,  977,  780,  766,  785,  469};
float jvar[12];
float a0[12]   = {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,};
float a1[12]   = { 90.0, 90.0, 85.0, 85.0, 90.0, 90.0, 90.0, 90.0, 85.0, 85.0, 90.0,  90.0};
float avar[12];

void setup()
{
  // Timer setup
  Timer1.initialize(50000);
  Timer1.attachInterrupt(control);

  for (int i = 0 ; i < 12 ; i++)
  {
    jvar[i] = (float)(j1[i] - j0[i]);
    avar[i] = (float)(a1[i] - a0[i]);
  }
  
  analogReference(EXTERNAL);
  
  Serial.begin(9600);
  
  // Initialize digital pins as outputs
  for (int i = 0 ; i < 12 ; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0 ; i < 12 ; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}

void control()
{
  //for (int i = 0 ; i < 12 ; i++)
  //{
//      motorTorque(0, pid(0));
//      motorTorque(1, pid(1));
//      motorTorque(6, pid(6));
//      motorTorque(7, pid(7));
//      motorTorque(5, pid(5));
//      motorTorque(11, pid(11));
//      motorTorque(4, pid(4));
//      motorTorque(10, pid(10));
//      motorTorque(2, pid(2));
//      motorTorque(3, pid(3));
      motorTorque(9, pid(9));
  //}
}

void loop()
{
  //for (int i = 0 ; i < 12 ; i++)
  //{
      goal[0] = 0.0;
      goal[1] = 0.0;
      goal[6] = 0.0;
      goal[7] = 0.0;
      goal[5] = 0.0;
      goal[11] = 0.0;
      goal[4] = 0.0;
      goal[10] = 0.0;
      goal[2] = 45.0;
      goal[3] = 45.0;
      goal[9] = 45.0;
      delay(2500);
      goal[0] = 75.0;
      goal[1] = 75.0;
      goal[6] = 75.0;
      goal[7] = 75.0;
      goal[5] = 90.0;
      goal[11] = 90.0;
      goal[4] = 90.0;
      goal[10] = 90.0;
      goal[2] =-45.0;
      goal[3] =-45.0;
      goal[9] =-45.0;
      delay(2500);
  //}
}

void testAllJoints()
{
  for (int i = 0 ; i < 12 ; i++)
  {
    motorTorque(i, 1.0);
    delay(150);

    motorTorque(i,-1.0);
    delay(150);
    
    motorTorque(i, 0.0);
  }  
}

void motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<12; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() is a lower level PWM control of the motor.
   Please use motorTorque instead! (see below)
 
 motor: this should be a integer between 0 and 11, will select
 which of the motors to be controlled
 
 direct: Should be between CW, CCW, BRAKE or BRAKEVCC
 BRAKEVCC: Brake to VCC
 CW:  Hips go down/out, knee goes down
 CCW: Hips go up/in, knee goes up
 BRAKE: Brake to GND
 
 pwm: should be a value between 0 and 255,
      higher the number, the faster it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor < 12)
  {
    if (direct <= 3)
    {
      // Set inA[motor]
      if (direct == CW)
      {
        digitalWrite(inApin[motor], HIGH);
        digitalWrite(inBpin[motor], LOW);
      } else if (direct == CCW) {
        digitalWrite(inApin[motor], LOW);
        digitalWrite(inBpin[motor], HIGH);
      } else if (direct == BRAKE) {
        digitalWrite(inApin[motor], LOW);
        digitalWrite(inBpin[motor], HIGH);
      }      
      analogWrite(pwmpin[motor], pwm);
    }
  }
}

  /* motorTorque() sets a torque between -1.0 and 1.0 to a motor.
   
   motor: this should be a integer between 0 and 11, will select
   which of the motors to be controlled
    
   torque: float between -1.0 and 1.0
   */
void motorTorque(uint8_t motor, float torque)
{
  if (torque > 0.0 && torque <= 1.0) {
    uint8_t pwm = (uint8_t)(255.0 * torque);
    motorGo(motor, CW, pwm);
  } else if (torque < 0.0 && torque >= -1.0) {
    uint8_t pwm = (uint8_t)(-255.0 * torque);
    motorGo(motor, CCW, pwm);
  } else if (torque == 0.0) {
    motorGo(motor, BRAKE, 0);
  }
}

/* readAngle() reads the angle of a joint

   joint: integer between 0 and 11 specifying the joint
   
   returns the angle of the joint as a float in degrees
 */
float readAngle(uint8_t joint)
{
  int ad_reading = analogRead(sensor[joint]);
  float scale = (float)(ad_reading - j0[joint]) / jvar[joint];
  float angle = a0[joint] + scale*avar[joint];
//  Serial.print(" R:");
//  Serial.print(ad_reading);
  return angle;
}

/* pid() implements the PID controller of a joint

   joint: integer between 0 and 11 specifying the joint
   
   returns the torque signal to be set to the motors
 */   
float pid(uint8_t joint)
{
  float err_before = err[joint];
  // proportional error
  float angle = readAngle(joint);
  err[joint] = goal[joint] - angle;
  // integral error
  erri[joint] += err[joint];
  // derivative error
  float errd = err[joint] - err_before;
  // controller
  float p = kp[joint] * err[joint];
  float i = ki[joint] * erri[joint];
  float d = kd[joint] * errd;
  float control = p + d;
//  Serial.print(" G:");
//  Serial.print(goal[joint]);
//  Serial.print(" A:");
//  Serial.print(angle);
//  Serial.print(" E:");
//  Serial.print(err[joint]);
//  Serial.print(" C:");
//  Serial.println(control);
  if (control >  1.0) control =  1.0;
  if (control < -1.0) control = -1.0;
  return control;
}
