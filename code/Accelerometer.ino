#define SAMPLE_COUNT 250 //for accelerometer calibration. Should lie between 1 and 256.

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

//float accelOneG = 9.80665;

#define STANDARD_GRAVITY -16384

#define ACCELO_RANGE 4*STANDARD_GRAVITY //Range is +- 2g
const long ACCELO_SCALING_FACTOR =  16384;

int accelSampleCount=0;

int32_t accelSample[3] = {0,0,0};
float accelRate[3] = {0.0,0.0,0.0};
float accelZero[3] = {0.0,0.0,0.0};

extern int16_t accelRaw[3];

void measureAccel() {
  //readMPU6000Accel();TODO:Wirte appropriate code.

  accelRate[XAXIS] = (float)(accelRaw[XAXIS]-accelZero[XAXIS])/ ACCELO_SCALING_FACTOR;
  accelRate[YAXIS] = (float)(accelRaw[YAXIS]-accelZero[YAXIS])/ ACCELO_SCALING_FACTOR;
  accelRate[ZAXIS] = (float)(accelZero[ZAXIS]-accelRaw[ZAXIS])/ ACCELO_SCALING_FACTOR;
}

void measureAccelSum() {
  //readMPU6050Accel();TODO:Write appropriate code. This line may not be needed..
  accelSample[XAXIS] += accelRaw[XAXIS];
  accelSample[YAXIS] += accelRaw[YAXIS];
  accelSample[ZAXIS] += accelRaw[ZAXIS];

  accelSampleCount++;
}

/*void evaluateAccelRate() //WARNING:AccelSampleCount!=0
{
  accelRate[XAXIS] = (float)(accelSample[XAXIS]/accelSampleCount)/ ACCELO_SCALING_FACTOR + accelBias[XAXIS];
  accelRate[YAXIS] = (float)(accelSample[YAXIS]/accelSampleCount)/ ACCELO_SCALING_FACTOR + accelBias[YAXIS];
  accelRate[ZAXIS] = (float)(accelSample[ZAXIS]/accelSampleCount)/ ACCELO_SCALING_FACTOR + accelBias[ZAXIS];
  
  accelSample[XAXIS]=0;
  accelSample[YAXIS]=0;
  accelSample[ZAXIS]=0;
  
  accelSampleCount = 0;
}*/

void computeAccelBias() {
  for (uint8_t samples = 0; samples < SAMPLE_COUNT; ++samples) 
  {
    getMPUValues();
    measureAccelSum();
    delay(10);
  }

  accelZero[XAXIS] = (accelSample[XAXIS]/accelSampleCount);
  accelZero[YAXIS] = (accelSample[YAXIS]/accelSampleCount);
  accelZero[ZAXIS] = STANDARD_GRAVITY + (accelSample[ZAXIS]/accelSampleCount);
  
  accelSample[XAXIS]=0;
  accelSample[YAXIS]=0;
  accelSample[ZAXIS]=0;
  
  accelSampleCount = 0;  
}

