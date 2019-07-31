/*****************************************************************************
 * Angles
 *****************************************************************************/

float u_rudder =70;
float maxRudderAngle = 115;
float minRudderAngle = 25;

float u_sail = 22;
float maxSailAngle = 55;
float minSailAngle = 0;

/*****************************************************************************
 * Pin
 *****************************************************************************/

#define RUDDER_PIN 12
#define SAIL_PIN   1

/*****************************************************************************
 * RC
 *****************************************************************************/

#define SIGNAL_DURATION   11000

float maxRudderRc = 2000;
float minRudderRc = 1000;

float maxSailRc = 2000;
float minSailRc = 1000;
