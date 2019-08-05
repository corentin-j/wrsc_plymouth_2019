#include "I2Cdev.h"
#include <ros.h>

const int chPinRudder = 4; // channel 1 sur le pin 4
const int chPinSail = 5; // channel 2 sur le pin 5
float chRudder;
float chSail;

unsigned long max_duration = 30000;
unsigned long min_duration = 0;
bool found = 0;

unsigned long t0,t1;

void setup() {
  pinMode(chPinRudder, INPUT);
  pinMode(chPinSail,INPUT);
  Serial.begin(115200);

  for (unsigned long duration=0;duration<max_duration;duration=duration+100)
    {
      chRudder = pulseIn(chPinRudder,HIGH,duration);
      //Serial.print(duration);
      //Serial.print(" : ");
      //Serial.println(chRudder);
      if ((min_duration != 0) && (chRudder != 0) && (found == 0))
      {
        found = 1;
        min_duration = duration*1.1;
        
      }
      if (found == 0)
      {
        min_duration = chRudder;
      }
    }
  Serial.println();
  Serial.println(min_duration);
  t0 = 0;
}

void loop() {
  t1 = millis();
  chRudder = pulseIn(chPinRudder,HIGH,5000);
  if (chRudder==0)
  {
    t0 += millis()-t1;
  }
  else
  {
    t0 = 0; // reset if there is a new data
    chRudder = pulseIn(chPinRudder,HIGH);
    Serial.print("Rudder : ");
    Serial.print(chRudder);
    Serial.print(", dt : ");
    Serial.println(millis()-t1);
  }
  if (t0 > 300)
  {
    Serial.print("No rc data, dt : ");
    Serial.println(millis()-t1);
  }
  else
  {
    chSail = pulseIn(chPinSail,HIGH,20000);
    if (chSail != 0)
    {
      Serial.print("Sail : ");
      Serial.print(chSail);
      Serial.print(", dt : ");
      Serial.println(millis()-t1);
    }
  }

}
