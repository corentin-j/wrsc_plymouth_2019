#include "MPU9250.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

/*****************************************************************************
 * Variables
 *****************************************************************************/

MPU9250 accelgyro;
sensor_msgs::Imu imuMsgs;
ros::Publisher pubImu("ardu_send_imu",&imuMsgs);
sensor_msgs::MagneticField magMsgs;
ros::Publisher pubMag("ardu_send_mag",&magMsgs);

float Axyz[3],Gxyz[3],Mxyz[3];
int16_t ax,ay,az, gx,gy,gz, mx,my,mz;

/*****************************************************************************
 * Program
 *****************************************************************************/
 
void imu_groove_setup() {
  accelgyro.initialize();
  nh.advertise(pubImu);
  nh.advertise(pubMag);
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
}

void imu_groove_update() {
  imu_groove_getdata();
  imu_groove_publish();
}

void imu_groove_publish(){
  long temps = millis();
  imuMsgs.header.stamp.sec = temps/1000;
  imuMsgs.header.stamp.nsec = temps;
  imuMsgs.linear_acceleration.x = Axyz[0];
  imuMsgs.linear_acceleration.y = Axyz[1];
  imuMsgs.linear_acceleration.z = Axyz[2];
  imuMsgs.angular_velocity.x = Gxyz[0];
  imuMsgs.angular_velocity.y = Gxyz[1];
  imuMsgs.angular_velocity.z = Gxyz[2];
  pubImu.publish(&imuMsgs);
  
  magMsgs.header.stamp.sec = temps/1000;
  magMsgs.header.stamp.nsec = temps;
  magMsgs.magnetic_field.x = Mxyz[0];
  magMsgs.magnetic_field.y = Mxyz[1];
  magMsgs.magnetic_field.z = Mxyz[2];
  pubMag.publish(&magMsgs);  
}

void imu_groove_getdata(void){
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  // rawAcc to accInG
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
  // rawGyro to deg/sec to rad/sec
  Gxyz[0] = (double) gx * 250 / 32768 * PI/180;
  Gxyz[1] = (double) gy * 250 / 32768 * PI/180;
  Gxyz[2] = (double) gz * 250 / 32768 * PI/180;
  // rawMag to T
  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}
