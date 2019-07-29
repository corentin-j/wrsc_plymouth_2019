
/*****************************************************************************
 * Variables
 *****************************************************************************/

const int chPinRudder = 4; // channel 1 sur le pin 4
const int chPinSail = 5; // channel 2 sur le pin 5
float chRudder;
float chSail;

unsigned long t0,t1;

/*****************************************************************************
 * Program
 *****************************************************************************/
 
void rc_receiver_setup() {
  pinMode(chPinRudder, INPUT);
  pinMode(chPinSail,INPUT);
}

void rc_receiver_update() {
  t1 = millis();
  chRudder = pulseIn(chPinRudder,HIGH,5000);
  if (chRudder==0)
  {
    t0 += millis()-t1;
  }
  else
  {
    t0 = 0; // reset if there is a new data
    /*Serial.print("Rudder : ");
    Serial.print(chRudder);
    Serial.print(", dt : ");
    Serial.println(millis()-t1);*/
  }
  if (t0 > 100)
  {
    /*Serial.print("No rc data, dt : ");
    Serial.println(millis()-t1);*/
    is_rc_on = 0;
  }
  else
  {
    is_rc_on = 1;
    chSail = pulseIn(chPinSail,HIGH,5000);
    if (chSail != 0)
    {
      /*Serial.print("Sail : ");
      Serial.print(chSail);
      Serial.print(", dt : ");
      Serial.println(millis()-t1);*/
    }
  }
}
