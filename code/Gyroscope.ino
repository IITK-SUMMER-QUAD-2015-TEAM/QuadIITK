#define DEG_TO_RAD  0.017453292519943295769236907684886
#define GYRO_RANGE 2*250.0
#define GYRO_SCALING_FACTOR GYRO_RANGE*DEG_TO_RAD/65536.0

#define FINDZERO_NUM 49//for calibration. Range can be set from 1 to 255

#define GYRO_CALIBRATION_TRESHOLD 25

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

float gyroHeading =0.0;
int16_t gyroRawData[3] = {0,0,0};
int16_t gyroSample[3] = {0,0,0};
float gyroRate[3] = {0.0,0.0,0.0};

uint8_t gyroSampleCount=0;

extern int16_t gyroRaw[3];
extern int16_t gyroZero[3];

extern int16_t findMedianIntWithDiff(int *data, int arraySize, int * diff);

void gyroUpdateHeading()
{
  static unsigned long int currentTime, lastTime;
  currentTime=micros();
  if (gyroRate[ZAXIS] > DEG_TO_RAD || gyroRate[ZAXIS] < DEG_TO_RAD) {//if drift is greater then 1deg/s
    gyroHeading += gyroRate[ZAXIS]*((currentTime - lastTime) / 1000000.0);
  }
  lastTime = currentTime;
}

void measureGyro() {
  //readMPU6000Gyro(); //TODO: write code for readMPU6000Gyro
  int gyroADC[3];
  gyroRate[XAXIS] = (gyroRaw[XAXIS]  - gyroZero[XAXIS])*GYRO_SCALING_FACTOR;
  gyroRate[YAXIS] = (gyroRaw[YAXIS]  - gyroZero[YAXIS])*GYRO_SCALING_FACTOR;
  gyroRate[ZAXIS] = (gyroRaw[ZAXIS]  - gyroZero[ZAXIS])*GYRO_SCALING_FACTOR;

  gyroUpdateHeading();
}

void measureGyroSum() {
  //readMPU6000Gyro(); //TODO: write code for readMPU6000Gyro
  gyroSample[XAXIS] += gyroRaw[XAXIS];
  gyroSample[YAXIS] += gyroRaw[YAXIS];
  gyroSample[ZAXIS] += gyroRaw[ZAXIS];

  gyroSampleCount++;
}

void evaluateGyroRate() //WARNING:GyroSampleCount!=0
{
  
  gyroRate[XAXIS] = ((gyroSample[XAXIS]/gyroSampleCount)  - gyroZero[XAXIS])*GYRO_SCALING_FACTOR;
  gyroRate[YAXIS] = ((gyroSample[YAXIS]/gyroSampleCount)   - gyroZero[YAXIS])*GYRO_SCALING_FACTOR;
  gyroRate[ZAXIS] = ((gyroSample[ZAXIS]/gyroSampleCount)   - gyroZero[ZAXIS])*GYRO_SCALING_FACTOR;

  gyroSample[XAXIS] = 0;
  gyroSample[YAXIS] = 0;
  gyroSample[ZAXIS] = 0;
  gyroSampleCount = 0;

  gyroUpdateHeading();
}

boolean calibrateGyro() {
  
  int16_t findZero[FINDZERO_NUM];
  int16_t diff = 0; 
  for (uint8_t axis = 0; axis < 3; ++axis) 
  {
    for (uint8_t i=0; i<FINDZERO_NUM; i++) 
    {
      //readMPU6000Sensors();TODO:edit this part.
      findZero[i]=gyroRaw[axis];
      delay(10);
    }
    int16_t tmp = findMedianIntWithDiff(findZero, FINDZERO_NUM, &diff);
    if (diff <= GYRO_CALIBRATION_TRESHOLD) 
      gyroZero[axis] = tmp; 
    else
      return false; //Calibration failed.
  }
  return true;
}
