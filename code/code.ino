#define BAUD_RATE 115200 //general baud rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, and 115200

#include <Wire.h>
#include "PID.h"

void Task100Hz(void);
extern void initI2CMPU(void);
extern void getMPUValues(void);

void printMPUValues(void);
int16_t temperature;

extern void initReceiver(void);
extern void printReceiverInput(void);

#define MANUAL 0
#define AUTOMATIC 1

void setup()
{
  //initI2CMPU();
  initReceiver();
  Serial.begin(BAUD_RATE);
}

void loop()
{
  static unsigned long int previousTime=0;
  unsigned long int currentTime=millis();
  unsigned long int deltaTime=currentTime-previousTime;
  //getMPUValues();
  printReceiverInput();
  //printMPUValues();
  if (deltaTime>10000)
  {
     Task100Hz();
     previousTime=currentTime;
  }
}

void Task100Hz(void)
{
}
