#include <Adafruit_PWMServoDriver.h>
#include <std_msgs/Float32.h>

/*****************************************************************************
 * Variables
 *****************************************************************************/

#define MIN_PULSE_WIDTH       650  // variable taken from library example
#define MAX_PULSE_WIDTH       2350 // variable taken from library example
#define DEFAULT_PULSE_WIDTH   1500 // variable taken from library example
#define FREQUENCY             50   // variable taken from library example

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*****************************************************************************
 * Program
 *****************************************************************************/

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

void motors_setup()
{
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}

void motors_update()
{
  pwm.setPWM(RUDDER_PIN,0,pulseWidth(u_rudder));
  pwm.setPWM(SAIL_PIN,0,pulseWidth(u_sail));
}
