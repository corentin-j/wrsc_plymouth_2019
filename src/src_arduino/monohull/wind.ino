#include <std_msgs/Float32.h>

/*****************************************************************************
 * Variables
 *****************************************************************************/

#define PIN_WIND A1
#define MAX_WIND 1023
#define MIN_WIND 0

std_msgs::Float32 windMsg;
ros::Publisher pubWind("ardu_send_wind_direction",&windMsg);
std_msgs::Float32 windSpeedMsg;
ros::Publisher pubWindSpeed("ardu_send_wind_speed",&windSpeedMsg);

float sensorValue =0;
float angleWind =0;
float ref = 0;

const byte pinAnemo= 2;
unsigned long t0wind;
unsigned long t1wind;
double windSpeed;
int validWind = 0;
double dt;


/*****************************************************************************
 * Program
 *****************************************************************************/

void wind_setup(){
  pinMode(PIN_WIND,INPUT);
  pinMode(pinAnemo,INPUT);
  t0wind = millis();
  validWind = 0;
  nh.advertise(pubWind);
  nh.advertise(pubWindSpeed);

  attachInterrupt(digitalPinToInterrupt(pinAnemo), anemoInterrupt,FALLING);
}

void wind_update(){
  sensorValue = analogRead(A1);
  angleWind = ((sensorValue-ref-MIN_WIND)/(MAX_WIND-MIN_WIND))*2*PI;
  angleWind = -2*atan(tan(angleWind/2));

  wind_direction_publish();
}

void anemoInterrupt(){
  if (validWind == 1){
    t1wind = millis();
    dt = (double)(t1wind-t0wind)/1000;
    windSpeed =  2*3.14/dt*0.055*3; 
    //Serial.println(windSpeed);
    wind_speed_publish();
    t0wind = t1wind;
  }
  else{
    validWind =1;
    t0wind= millis();
  }
}

void wind_direction_publish(){
  windMsg.data = angleWind;
  pubWind.publish(&windMsg);
}


void wind_speed_publish(){
  windSpeedMsg.data = windSpeed;
  pubWindSpeed.publish(&windSpeedMsg);
}
