
#define PIN_WIND A1
#define MAX_WIND 970
#define MIN_WIND 45
float sensorValue =0;
float angleWind =0;
float ref = 0;

const byte pinAnemo= 2;
int counterAnemo = 0;
unsigned long t0;
unsigned long t1;
double windSpeed;
int validWind = 0;
double dt;


void setup(){
  pinMode(PIN_WIND,INPUT);
  pinMode(pinAnemo,INPUT);
  t0 = millis();
  validWind = 0;
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(pinAnemo), anemoInterrupt,FALLING);
}

void loop(){
  updateWind();
}

void updateWind(){
  sensorValue = analogRead(A1);
  angleWind = ((sensorValue-ref-MIN_WIND)/(MAX_WIND-MIN_WIND))*2*PI;
  angleWind = 2*atan(tan(angleWind/2));

  //Serial.println(angleWind);
}


/*******anemo**********/

void anemoInterrupt(){
  if (validWind == 1){
    t1 = millis();
    dt = (double)(t1-t0)/1000;
    windSpeed =  2*3.14/dt*0.055*3; 
    Serial.println(windSpeed);
    t0 = t1;
  }
  else{
    validWind =1;
    t0= millis();
  }
}
