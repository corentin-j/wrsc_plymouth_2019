#include <I2Cdev.h>
#include <Wire.h>
#include "MPU9250.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

MPU9250 front;
float f_Axyz[3],f_Gxyz[3],f_Mxyz[3];
MPU9250 back(0x69);
float b_Axyz[3],b_Gxyz[3],b_Mxyz[3];

int16_t ax,ay,az, gx,gy,gz, mx,my,mz;
int16_t b_ax,b_ay,b_az, b_gx,b_gy,b_gz, b_mx,b_my,b_mz;

sensor_msgs::Imu front_imu;
ros::Publisher pub_front_imu("front_imu",&front_imu);
sensor_msgs::MagneticField front_mag;
ros::Publisher pub_front_mag("front_mag",&front_mag);

sensor_msgs::Imu back_imu;
ros::Publisher pub_back_imu("back_imu",&back_imu);
sensor_msgs::MagneticField back_mag;
ros::Publisher pub_back_mag("back_mag",&back_mag);

ros::NodeHandle nh;
I2Cdev   I2C_M;

void setup() {


  nh.getHardware()->setBaud(115200);
  nh.initNode();

  Serial.begin(115200);
  Wire.begin();
  // put your setup code here, to run once:
  front.initialize();
  back.initialize();
  nh.advertise(pub_front_imu);
  nh.advertise(pub_front_mag);
  nh.advertise(pub_back_imu);
  nh.advertise(pub_back_mag);
  
  Serial.println("Testing device connections...");
  Serial.println(front.testConnection() ? "Front MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println(back.testConnection() ? "Back MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  Serial.println("Starting loop");
}

void front_getdata(void){
  front.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  // rawAcc to accInG
  f_Axyz[0] = (double) ax / 16384;
  f_Axyz[1] = (double) ay / 16384;
  f_Axyz[2] = (double) az / 16384; 
  // rawGyro to deg/sec to rad/sec
  f_Gxyz[0] = (double) gx * 250 / 32768 * PI/180;
  f_Gxyz[1] = (double) gy * 250 / 32768 * PI/180;
  f_Gxyz[2] = (double) gz * 250 / 32768 * PI/180;
  // rawMag to T
  f_Mxyz[0] = (double) mx * 1200 / 4096;
  f_Mxyz[1] = (double) my * 1200 / 4096;
  f_Mxyz[2] = (double) mz * 1200 / 4096;
}

void back_getdata(void){
  back.getMotion9(&b_ax, &b_ay, &b_az, &b_gx, &b_gy, &b_gz, &b_mx, &b_my, &b_mz);
  // rawAcc to accInG
  b_Axyz[0] = (double) b_ax / 16384;
  b_Axyz[1] = (double) b_ay / 16384;
  b_Axyz[2] = (double) b_az / 16384; 
  // rawGyro to deg/sec to rad/sec
  b_Gxyz[0] = (double) b_gx * 250 / 32768 * PI/180;
  b_Gxyz[1] = (double) b_gy * 250 / 32768 * PI/180;
  b_Gxyz[2] = (double) b_gz * 250 / 32768 * PI/180;
  // rawMag to T
  b_Mxyz[0] = (double) b_mx * 1200 / 4096;
  b_Mxyz[1] = (double) b_my * 1200 / 4096;
  b_Mxyz[2] = (double) b_mz * 1200 / 4096;
}

void front_publish(){
  long temps = millis();
  front_imu.header.stamp.sec = temps/1000;
  front_imu.header.stamp.nsec = temps;
  front_imu.linear_acceleration.x = f_Axyz[0];
  front_imu.linear_acceleration.y = f_Axyz[1];
  front_imu.linear_acceleration.z = f_Axyz[2];
  front_imu.angular_velocity.x = f_Gxyz[0];
  front_imu.angular_velocity.y = f_Gxyz[1];
  front_imu.angular_velocity.z = f_Gxyz[2];
  pub_front_imu.publish(&front_imu);
  
  front_mag.header.stamp.sec = temps/1000;
  front_mag.header.stamp.nsec = temps;
  front_mag.magnetic_field.x = f_Mxyz[0];
  front_mag.magnetic_field.y = f_Mxyz[1];
  front_mag.magnetic_field.z = f_Mxyz[2];
  pub_front_mag.publish(&front_mag);
}

void back_publish(){
  long temps = millis();
  back_imu.header.stamp.sec = temps/1000;
  back_imu.header.stamp.nsec = temps;
  back_imu.linear_acceleration.x = b_Axyz[0];
  back_imu.linear_acceleration.y = b_Axyz[1];
  back_imu.linear_acceleration.z = b_Axyz[2];
  back_imu.angular_velocity.x = b_Gxyz[0];
  back_imu.angular_velocity.y = b_Gxyz[1];
  back_imu.angular_velocity.z = b_Gxyz[2];
  pub_back_imu.publish(&back_imu);
  
  back_mag.header.stamp.sec = temps/1000;
  back_mag.header.stamp.nsec = temps;
  back_mag.magnetic_field.x = b_Mxyz[0];
  back_mag.magnetic_field.y = b_Mxyz[1];
  back_mag.magnetic_field.z = b_Mxyz[2];
  pub_back_mag.publish(&back_mag);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  nh.spinOnce();
  front_getdata();
  back_getdata();
  front_publish();
  back_publish();
}
