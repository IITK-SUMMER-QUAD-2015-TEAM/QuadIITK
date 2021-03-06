#define BAUD_RATE 57600//general baud rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, and 115200
#define SD_CARD_PIN 53
#define LED_PIN 50

#define MEGA 0
#define DUE  1

#define PLATFORM DUE

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <mavlink.h>

#if (PLATFORM==MEGA)
  #include "ReceiverMega.h"
#elif (PLATFORM==DUE)
  #include "ReceiverDue.h"
#else
  #error Please choose a valid platform!
#endif
/*TODO: Have to check orientation of accelerometer before calibrating it.*/

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

void Task100Hz(void);

extern int systemStatus;

extern void initI2CMPU(void);
extern void getMPUValues(void);
extern void printMPUValues(void);
extern void printSDIMU(void);

extern void measureIMUSensors(void);
extern void evaluateAccelRate(void);
extern void evaluateGyroRate(void);

extern void initMagnet(void);
extern void getRawVal(void);
extern void calibrateMagnetometer(void);
extern void printMagnet(void);

extern void initReceiver(void);
extern void printReceiverInput(void);

extern void initMotors(void);

extern void flightErrorCalculator(void);

extern void armedCheck(void);
extern void setIZero(void);

extern void setUpPIDs(void);

extern void sendHeartbeat(void);
extern void receiveCommunication(void);
extern void sendInformation(void);

extern void calibrateGyro(void);
extern void computeAccelBias(void);

int16_t temperature;
File myFile;

boolean isArmed=false,isPrevArmed=false;

extern void printSDMotors(void);

extern void calculateAngleOffset(void);
extern void calculateKinematics(void);

extern void fourthOrderFilter(void);

uint8_t count100Hz=0;

extern struct fourthOrderFilter fourthOrder[3];

void setup()
{
  
  initReceiver();
  initMotors();
  
  Serial.begin(BAUD_RATE);
    
  initI2CMPU();
  
  //initMagnet();
  
  //calibrateGyro();
   // Serial.println("gyro");
  //computeAccelBias();
  
  //getCoefficients();//For the second order filter.
  calculateAngleOffset();
  initParameters();//For MAVLINK paramters
  initEEPROM();
  setUpPIDs();
  
  
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);
  systemStatus=MAV_STATE_STANDBY;
}

void loop()
{
  
  static unsigned long int previousTime=0;
  unsigned long int currentTime=micros();
  unsigned long int deltaTime=currentTime-previousTime;
  if (deltaTime>10000)
  {
    uint32_t curr=micros();
    //myFile=SD.open("IMU.csv",FILE_WRITE);
    //kalman();
    
   measureIMUSensors();
   fourthOrderFilter();
   calculateKinematics();
//printAngles();
   //getMagnet();
    //calibrateMagnetometer();
    //printMagnet();   
    //Serial.println(deltaTime);
    previousTime=currentTime;
    armedCheck();
    if(isPrevArmed&&!(isArmed))
    {
      setMotorsZero(); 
      setIZero();  
    }
    isPrevArmed=isArmed;
    if(isArmed)
      Task100Hz();
    //printSDMotor();
    //myFile.pr--intln("Hello World");
    //Serial.println("Hello");
    //printSDIMU();
    //myFile.println(receivers[ROLL].getDiff());
    //myFile.close();
    //Task1Hz()
    {
      if(count100Hz==100)
      {
        count100Hz=0;
        sendHeartbeat();
      }
      else
        ++count100Hz;
      
    }
    //Task10Hz()
    
    if((count100Hz%5)==0)
    {
      receiveCommunication();
      sendInformation();
    }
     //Serial.print("a:\t");Serial.println(isArmed);
     //Task50Hz()
     /*The operations which the 50Hz task loop performs are:
     **reads pilot commands i.e. sets different modes.*/
     //Task10Hz1->magnetometer and calculated yaw fusion for heading...
     //Task10Hz2&3->Battery Monitor, telemetry,OSD etc.
     //Task1Hz->Mavlink.
     /**************DEBUG**************************************************************/
     
     //printMPUValues();
     //printReceiverInput();
    //Serial.print("t:\t");
  //  Serial.println(micros()-curr);
  }
  
}

void Task100Hz(void)
{
  //evaluateGyroRate();
  //evaluateAccelRate();
  /*TODO: Maybe implement a fourth order filter.
  **TODO: Calculate Kinematics
  **TODO: Estimate Vz for altitude hold*/
 flightErrorCalculator();//for roll and pitch
 
  //processHeading();
  /*The former uses dual PID wheras the latter uses a single PID.
  **TODO: Code for calibration of offset in radio values read}*/
}
