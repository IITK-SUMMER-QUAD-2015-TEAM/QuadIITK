#define BAUD_RATE 115200 //general baud rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, and 115200
#define SD_CARD_PIN 53

#define MEGA 0
#define DUE  1

#define PLATFORM DUE

#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#if PLATFORM==MEGA
  #include "ReceiverMega.h"
#elif PLATFORM==DUE
  #include "ReceiverDue.h"
#else
  #error Please choose a valid platform!
#endif

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

extern void armedCheck(void);
extern void setIZero(void);

extern void setUpPIDs(void);

extern void sendHeartbeat(void);

int16_t temperature;
File myFile;
boolean isArmed=false,isPrevArmed=false;

uint8_t count100Hz=0;

void setup()
{
  
  initReceiver();
  initMotors();
  Serial.begin(BAUD_RATE);
  //if (!SD.begin(SD_CARD_PIN)) Serial.println("Initialization failed!!");
  //else Serial.println("Initialization Complete");
  setUpPIDs();
  initI2CMPU();
  int i;
  pinMode(53,OUTPUT);
  digitalWrite(53,LOW);
}

void loop()
{
  static unsigned long int previousTime=0;
  unsigned long int currentTime=micros();
  unsigned long int deltaTime=currentTime-previousTime;
  measureIMUSensors();
  
  if (deltaTime>10000)
  {
    uint32_t curr=micros();
    previousTime=currentTime;
    armedCheck();
    if(isPrevArmed&&!(isArmed))
    {
      setMotorsZero(); 
      setIZero();  
    }
    if(isArmed){
      
      digitalWrite(53,HIGH);
    }
    else
      digitalWrite(53,LOW);
    isPrevArmed=isArmed;
    if(count100Hz==100)
    {
      count100Hz=0;
      sendHeartbeat();
      Serial.print("Hi");
    }
    else
      ++count100Hz;
    Task100Hz();
     //Serial.print("a:\t");Serial.println(isArmed);
     //Task50Hz()
     /*The operations which the 50Hz task loop performs are:
     **reads pilot commands i.e. sets different modes.*/
     //Task10Hz1->magnetometer and calculated yaw fusion for heading...
     //Task10Hz2&3->Battery Monitor, telemetry,OSD etc.
     //Task1Hz->Mavlink.
     /**************DEBUG**************************************************************/
     //myFile=SD.open("data.csv",FILE_WRITE);
     //printMPUValues();
     //printReceiverInput();
     //myFile.close();
     Serial.print("t:\t");Serial.println(micros()-curr);
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
