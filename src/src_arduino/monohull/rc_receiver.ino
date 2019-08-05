#include <geometry_msgs/Vector3.h>

/*****************************************************************************
 * Variables
 *****************************************************************************/
geometry_msgs::Vector3 rcMsg;
ros::Publisher pubrcMsg("ardu_send_rc",&rcMsg);

const int chPinRudder = 4; // channel 1 sur le pin 4
const int chPinSail = 5; // channel 2 sur le pin 5
float chRudder;
float chSail;

unsigned long t0,t1;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*****************************************************************************
 * Program
 *****************************************************************************/
 
void rc_receiver_setup() {
  pinMode(chPinRudder, INPUT);
  pinMode(chPinSail,INPUT);
  
  nh.advertise(pubrcMsg);
}

void rc_receiver_update() {
  t1 = millis();
  chRudder = pulseIn(chPinRudder,HIGH,6000);
  if (chRudder==0)
  {
    t0 += millis()-t1;
  }
  else
  {
    t0 = 0; // reset if there is a new data
    chRudder = pulseIn(chPinRudder,HIGH);
    u_rudder = map(chRudder,minRudderRc,maxRudderRc,minRudderAngle,maxRudderAngle); // conversion
    u_rudder = max(minRudderAngle,u_rudder);                                        // security
    u_rudder = min(maxRudderAngle,u_rudder);                                        // security
  }
  
  if (t0 > 100)
  {
    //Serial.print("No rc data");
    is_rc_on = 0;
  }
  else
  {
    is_rc_on = 1;
    chSail = pulseIn(chPinSail,HIGH,15000);
    rcMsg.y = chSail;
    if (chSail != 0)
    {
      u_sail  = map(chSail,minSailRc,maxSailRc,minSailAngle,maxSailAngle); // conversion
      u_sail  = max(minSailAngle,u_sail);                                  // security
      u_sail  = min(maxSailAngle,u_sail);                                  // security
    }
  }
  rcMsg.x  = u_rudder;//mapfloat(u_rudder,minRudderAngle,maxRudderAngle,-PI/4,PI/4);         // send the sail angle
  //rcMsg.y = u_sail;//mapfloat(u_sail,minSailAngle,maxSailAngle,0,PI/2);         // send the sail angle
  rcMsg.z = is_rc_on;
  pubrcMsg.publish(&rcMsg);
}
