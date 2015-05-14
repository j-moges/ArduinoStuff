/*************************************************************************
* Sample sketch based on OBD-II library for Arduino
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2014 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>

COBD obd;

void setup()
{
  // we'll use the debug LED as output
  pinMode(13, OUTPUT);  
  // start communication with OBD-II UART adapter
  obd.begin();
  // initiate OBD-II connection until success
  while (!obd.init());  
}

void loop()
{
  int RPMvalue, throttlePos;
  if (obd.read(PID_RPM, RPMvalue)) {
    if(obd.read(PID_THROTTLE, throttlePos)){
      Serial.print("RPM: "); Serial.print(RPMvalue);
      Serial.println("\tThrottle Position: " + throttlePos);
    }
  }
}
