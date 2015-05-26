#define BAUD_RATE 9600 //general baud rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, and 115200

#define SD_CARD_PIN 53

#include <SD.h>
#include <Wire.h>
#include "PID.h"

void Task100Hz(void);

extern void initI2CMPU(void);
extern void getMPUValues(void);
extern void printMPUValues(void);

extern void measureIMUSensors(void);
extern void evaluateAccelRate(void);
extern void evaluateGyroRate(void);

extern void initReceiver(void);
extern void printReceiverInput(void);

extern void initMotors(void);

extern void flightErrorCalculator(void);

extern void setUpPIDs(void);

int16_t temperature;
File myFile;

void setup()
{
  initI2CMPU();
  initReceiver();
  initMotors();
  Serial.begin(BAUD_RATE);
  if (!SD.begin(SD_CARD_PIN)) Serial.println("Initialization failed!!");
  else Serial.println("Initialization Complete");
  setUpPIDs();
}

void loop()
{
  static unsigned long int previousTime=0;
  unsigned long int currentTime=micros();
  unsigned long int deltaTime=currentTime-previousTime;
  measureIMUSensors();
  
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
     /**************DEBUG**************************************************************/
     myFile=SD.open("data.csv",FILE_WRITE);
     //printMPUValues();
     //printReceiverInput();
     myFile.close();
  }
}

void Task100Hz(void)
{
  evaluateGyroRate();
  evaluateAccelRate();
  /*TODO: Maybe implement a fourth order filter.
  **TODO: Calculate Kinematics
  **TODO: Estimate Vz for altitude hold*/
 flightErrorCalculator();//for roll and pitch
  //processHeading();
  /*The former uses dual PID wheras the latter uses a single PID.
  **TODO: Code for calibration of offset in radio values read}*/
}
