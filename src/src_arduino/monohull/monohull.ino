#include <I2Cdev.h>
#include <ros.h>
#include <Wire.h>
#include "config_monohull.h"

/*****************************************************************************
 * Variables
 *****************************************************************************/

#define BAUDRATE 115200//250000

ros::NodeHandle nh;
I2Cdev   I2C_M;
bool is_rc_on = 0;

/*****************************************************************************
 * Program
 *****************************************************************************/
 
void setup() {
  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();
  Serial.begin(BAUDRATE);
  Wire.begin();

  imu_groove_setup();
  rc_receiver_setup();
  wind_setup();
  motors_setup();
}

void loop() {
  nh.spinOnce();
  imu_groove_update();
  rc_receiver_update();
  wind_update();
  motors_update();
}
