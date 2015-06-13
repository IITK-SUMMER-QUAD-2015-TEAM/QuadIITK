#define GYRO_SCALING_FACTOR 0.0001332273112807464f

#define FINDZERO_NUM 1000//for calibration. Range can be set from 1 to 255

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

float gyroHeading =0.0;

int16_t gyroSample[3] = {0,0,0};
float gyroRate[3] = {0.0,0.0,0.0};
int16_t gyroZero[3]={0,0,0};

uint8_t gyroSampleCount=0;

extern int16_t gyroRaw[3];

extern int16_t findMedianIntWithDiff(int *data, int arraySize, int *diff);

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
  gyroRate[XAXIS] = ((gyroRaw[XAXIS]  - gyroZero[XAXIS])/131)*(3.1415/180);
  gyroRate[YAXIS] = ((gyroRaw[YAXIS]  - gyroZero[YAXIS])/131)*(3.1415/180);
  gyroRate[ZAXIS] = ((gyroRaw[ZAXIS]  - gyroZero[ZAXIS])/131)*(3.1415/180);

  gyroUpdateHeading();
}

void measureGyroSum() {
  //readMPU6050Gyro(); //TODO: write code for readMPU6050Gyro.. This line may not be needed..
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

void calibrateGyro() {
  int32_t gyroSum[3]={0,0,0};
  for (uint16_t i=0; i<FINDZERO_NUM; ++i) 
  {
      getMPUValues();
      gyroSum[XAXIS]+=gyroRaw[XAXIS];
      gyroSum[YAXIS]+=gyroRaw[YAXIS];
      gyroSum[ZAXIS]+=gyroRaw[ZAXIS];
      delay(10);
  }
  gyroZero[XAXIS]=gyroSum[XAXIS]/FINDZERO_NUM;
  gyroZero[YAXIS]=gyroSum[YAXIS]/FINDZERO_NUM;
  gyroZero[ZAXIS]=gyroSum[ZAXIS]/FINDZERO_NUM;
}


