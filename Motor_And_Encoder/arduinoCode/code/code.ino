#include <math.h>

////////////////////////////////////////////////////////////
// PID SECTION
////////////////////////////////////////////////////////////
double Kp = 1.000;  // proportional coifficient
double Ki = 0.000;  // integral     coifficient
double Kd = 0.025;  // derivative   coifficient

uint64_t CurrTime=0;
uint64_t PrevTime=0;
double   dt=0;

int32_t Error=0;
int32_t PrevError=0;

double  Proportional=0; 
double  Integeral=0;
double  Derivative=0;

double PIDvalue=0;
/**********************************************************/

////////////////////////////////////////////////////////////
// MOTOR1 SECTION
////////////////////////////////////////////////////////////
#define MOTOR1_R 		4
#define MOTOR1_L 		5
#define MOTOR1_PWM  6

int32_t CurrentMotorAngle; 
int32_t TargetMotorAngle;
uint8_t MotorSpeed; 
/**********************************************************/

////////////////////////////////////////////////////////////
// ENCODER SECTION
////////////////////////////////////////////////////////////
#define ENCODER_PINA    2
#define ENCODER_PINB    3
/**********************************************************/

void setup () 
{
  ////////////////////////////////////////////////////////////
  // MOTOR SECTION
  ///////////////////////////////////////////////////////////
  CurrentMotorAngle = 0;

  pinMode(MOTOR1_R, OUTPUT);
  pinMode(MOTOR1_L, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  /**********************************************************/

  ////////////////////////////////////////////////////////////
  // ENCODER SECTION
  ///////////////////////////////////////////////////////////
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), ENCODER_voidPinaISR, FALLING);
  /**********************************************************/

  Serial.begin(9600);

}

void loop() 
{
  // float carla_value = 1.0;
  // float floatAnlge = mapFloat(carla_value, -1.0, 1.0, -360.0, 360.0);
  // TargetMotorAngle = (floatAnlge > 0)? ceil(floatAnlge) : floor(floatAnlge);
  TargetMotorAngle = 360;
  PID_voidSetMotorSpeed();
  MOTOR_voidUpdateAngle();
   
}


////////////////////////////////////////////////////////////
// ENCODER SECTION
////////////////////////////////////////////////////////////

/**********************************************/
// DESC.... isr function of pinA that increase 
//          position.
// RET_VAL. nothing
/**********************************************/
void ENCODER_voidPinaISR(void)
{
  //ENCODER_PINA status will be LOW
  //check ENCODER_PINB to detect direction
  if(digitalRead(ENCODER_PINB) == LOW) // if ENCODER_PINB status IS the same as ENCODER_PINA
  {
    // then the encoder rotate CCW
    CurrentMotorAngle ++;
  }
  else if(digitalRead(ENCODER_PINB) == HIGH) // if ENCODER_PINB status IS NOT the same as ENCODER_PINA
  {
    // then the encoder rotate CW
    CurrentMotorAngle --;
  }
}

////////////////////////////////////////////////////////////
// PID SECTION
////////////////////////////////////////////////////////////

//**************************************************/
// @DESC    <!> function calculate the motor speed.
// @RET_VAL <!> nothing
//**************************************************/
void PID_voidSetMotorSpeed(void)
{
  // Calculate time elapsed since last iteration
  CurrTime = millis();
  dt = (CurrTime - PrevTime) / 1000.0; // Convert to seconds

  // Calculate angle error
  Error = TargetMotorAngle - CurrentMotorAngle;

  // Calculate PID terms
  Proportional = Kp * Error;
  Integeral   += Ki * Error * dt; // Accumulate error over time
  Derivative   = Kd * ((Error - PrevError) / dt); // Calculate derivative over time

  // Calculate PID output
  PIDvalue = Proportional + Integeral + Derivative;
  
  // Set motor speed
  PIDvalue = abs(PIDvalue);

  // Limit output to motor speed range
  if (PIDvalue > 255) 
  {
    PIDvalue = 255;
  }

  // Set motor speed
  MotorSpeed = PIDvalue;

  // Update previous time and angle error for next iteration
  PrevTime  = CurrTime;
  PrevError = Error;
}


////////////////////////////////////////////////////////////
// MOTOR SECTION
////////////////////////////////////////////////////////////

//**************************************************/
// @DESC    <!> function to rotate motor to target
//              angel.
// @RET_VAL <!> nothing
//**************************************************/
void MOTOR_voidUpdateAngle(void)
{  
  // check error sign to detect direction of motor
  if(Error > 0) // if error is positive
  {
    // then we need to increase CurrentMotorAngle, we need to rotate CCW
    analogWrite(MOTOR1_PWM, MotorSpeed);
    digitalWrite(MOTOR1_L, HIGH);
    digitalWrite(MOTOR1_R, LOW);
  }
  else if(Error < 0) // if error is negative
  {
    // then we need to decrease CurrentMotorAngle, we need to rotate CW
    analogWrite(MOTOR1_PWM, MotorSpeed);
    digitalWrite(MOTOR1_L, LOW);
    digitalWrite(MOTOR1_R, HIGH);
  }
  else // if error is zero
  {
    // then we need to stop motor
    analogWrite(MOTOR1_PWM, 0);
    digitalWrite(MOTOR1_L, LOW);
    digitalWrite(MOTOR1_R, LOW);
  }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

