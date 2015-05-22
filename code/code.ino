#define BAUD_RATE 115200 //general baud rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, and 115200

#include <Wire.h>
#include "PID.h"

void Task100Hz(void);

extern void initI2CMPU(void);
extern void getMPUValues(void);
extern void printMPUValues(void);

extern void measureIMUSensors(void);
extern void evaluateAccelRate(void);
extern void evaluateGyroRate(void);

int16_t temperature;

extern void initReceiver(void);
extern void printReceiverInput(void);

void setup()
{
  initI2CMPU();
  initReceiver();
  Serial.begin(BAUD_RATE);
}

void loop()
{
  static unsigned long int previousTime=0;
  unsigned long int currentTime=millis();
  unsigned long int deltaTime=currentTime-previousTime;
  measureIMUSensors();
  printReceiverInput();
  //printMPUValues();
  if (deltaTime>10000)
  {
     Task100Hz();
     previousTime=currentTime;
     //Task50Hz()
     /*The operations which the 50Hz task loop performs are:
     **reads pilot commands i.e. sets different modes.*/
     //Task10Hz1->magnetometer and calculated yaw fusion for heading...
     //Task10Hz2&3->Battery Monitor, telemetry,OSD etc.
     //Task1Hz->Mavlink.
  }
}

void Task100Hz(void)
{
  evaluateGyroRate();
  evaluateAccelRate();
  /*TODO: Maybe implement a fourth order filter.
  **TODO: Calculate Kinematics
  **TODO: Estimate Vz for altitude hold
  **TODO: Flight error calculator..(Maybe for two modes:(attitude and rate mode
  **The former uses dual PID wheras the latter uses a single PID.
  **TODO: Code for calibration of offset in radio values read}*/
}
