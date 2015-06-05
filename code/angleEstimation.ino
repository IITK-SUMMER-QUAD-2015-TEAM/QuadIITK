
/*Gyro output rate: 8kHz (if we do'nt use DLPF and 1kHz if do use DLPF)
**Accelo output rate: 1kHz*/

/*All of the below regiater addresses are for the most significant bits, i.e. for bits[15:8]*/
/*
#include <Wire.h>

#define GYRO_XOUT_REGISTER  0x43
#define GYRO_YOUT_REGISTER  0x45
#define GYRO_ZOUT_REGISTER  0x47

#define ACCELO_XOUT_REGISTER 0x3B
#define ACCELO_YOUT_REGISTER 0x3D
#define ACCELO_ZOUT_REGISTER 0x3F

#define TEMP_OUT_REGISTER 0x41

/*The below register addresses are for the configuration bits of the gyrometer and accelometer ranges*/
#define GYRO_CONFIG_REGISTER 0x1B
#define FS_SEL_BIT 3//0-> +-250 deg/s 1-> +-500 deg/s 0-> +-1000 deg/s 0-> +-2000 deg/s

#define ACCEL0_CONFIG_REGISTER 0x1C
#define AFS_SEL_BIT 3//0-> +-2g 1-> +-4g 0-> +-8g 0-> +-16g

/*One of the power registers.  Contains the Sleep mode bit. The sleep mode bit should be set to 0 for sleep mode to be deactivated.*/
/*#define PWR_MGMT_1_REGISTER 0x6B

#define MPU_ADDRESS 0x68 // Can be made 0x69 as well using the AD0 pin of the IMU.

#define CONFIG_REGISTER 0x1A //The register containing the DLPF bits.

#define CONFIG_VALUE  4  //0->260Hz  1->184Hz  2->94Hz  3->44Hz  4->21Hz  5->260Hz  6->10Hz  7->5Hz
//It is should be noted that decreasing the bandwidth increases the delay.
/*The dividing factors. They change with different modes of operation of the gyro and accelerometer*/
/*#define ACCELO_DIVIDING_FACTOR 16384//2g->16384  4g->8192  8g->4096  16g->2048
#define GYRO_DIVIDING_FACTOR 131  //250dps->131  500dps->65.5  1000dps->32.8  2000dps->16.4 

/*For temperature sensor*/
/**#define TEMP_DIVIDING_FACTOR 340
#define TEMP_OFFSET  36.53


float Kp = 0.0;                   				
float Ki = 0.0;                   					
float halfT = 0.0;                					
float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;      
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		

float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;
float Angle[3];
float currentTime;
float previousTime=0.0;
float diff;
float ax;
float ay;
float az;
float wx;
float wy;
float wz;
float temperature;
float gyroOffsetx=0.0;
float gyroOffsety=0.0;
float gyroOffsetz=0.0;
float offset[3];

int16_t accx;
int16_t accy;
int16_t accz;

int16_t wwx;
int16_t wwy;
int16_t wwz;

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

boolean changedSign(float a,float b){
  if ((a>0&&b<0)||(a<0&&b>0)) return true;
  else return false;
  
}

void quaternionECF() {
  
  float norm;
  float gx, gy, gz;
  float q0i, q1i, q2i, q3i;
  float ex, ey, ez;
    
  halfT = diff/2;
  
  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
     	
  // STEP 1 estimated direction of gravity [G]b=[R]t*[G]e;
  gx = 2*(q1*q3 - q0*q2);
  gy = 2*(q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
  // STEP 2 error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (gy*az - gz*ay);
  ey = (gz*ax - gx*az);
  ez = (gx*ay - gy*ax);
    
  // STEP 3 integral error scaled integral gain
  exInt = exInt + ex*Ki;
  if (changedSign(previousEx,ex)) exInt=0.0;
  previousEx=ex;
  
  eyInt = eyInt + ey*Ki;
  if (changedSign(previousEy,ey)) eyInt=0.0;
  previousEy=ey;
  
  ezInt = ezInt + ez*Ki;
  if (changedSign(previousEz,ez)) ezInt=0.0;
  previousEz=ez;
 
  // STEP 4 adjust gyroscope measurements
  wx = wx + Kp*ex + exInt;
  wy = wy + Kp*ey + eyInt;
  wz = wz + Kp*ez + ezInt;
    
  // integrate quaternion rate and normalise
  q0i = (-q1*wx - q2*wy - q3*wz) * halfT;
  q1i = ( q0*wx + q2*wz - q3*wy) * halfT;
  q2i = ( q0*wy - q1*wz + q3*wx) * halfT;
  q3i = ( q0*wz + q1*wy - q2*wx) * halfT;
  q0 =q0+ q0i;
  q1 =q1+ q1i;
  q2 =q2+ q2i;
  q3 =q3+ q3i;
    
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
}
  
void eulerAngles()
{
  Angle[XAXIS]  =(180/3.14)*atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  if (Angle[XAXIS]<0) Angle[XAXIS]=Angle[XAXIS]+180;
  else Angle[XAXIS]=Angle[XAXIS]-180;
  
  Angle[YAXIS] = -(180/3.14)*asin(2 * (q0*q2 - q1*q3));
  Angle[ZAXIS]   = (180/3.14)*atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}


void initializeKinematics() 
{
  
  offset[XAXIS]=0.0;
  offset[YAXIS]=0.0;
  offset[ZAXIS]=0.0;
  Angle[XAXIS]=0.0;
  Angle[YAXIS]=0.0;
  Angle[ZAXIS]=0.0;
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
	
  previousEx = 0;
  previousEy = 0;
  previousEz = 0;

  Kp = 32.3;
  Ki = 12.00;
}

void calculateKinematics() {
    
  quaternionECF();
  eulerAngles();
}

void initI2CMPU(void) //Begins I2C communication with  MPU using it's internal 8MHz oscillator for CLKSEL and also wakes up the MPU.
{
  Wire.begin();

  Wire.beginTransmission(MPU_ADDRESS);

  Wire.write(PWR_MGMT_1_REGISTER);
  Wire.write(0);

  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDRESS);

  Wire.write(CONFIG_REGISTER);
  Wire.write(CONFIG_VALUE);

  Wire.endTransmission(true);

}

void getMPUValues(void)
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(ACCELO_XOUT_REGISTER);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDRESS, 14, true);

  if (Wire.available())
  {
    accx = (Wire.read() << 8) | Wire.read();
    accy = (Wire.read() << 8) | Wire.read();
    accz = (Wire.read() << 8) | Wire.read();

    temperature = (Wire.read() << 8) | Wire.read();

    wwx = (Wire.read() << 8) | Wire.read();
    wwy = (Wire.read() << 8) | Wire.read();
    wwz = (Wire.read() << 8) | Wire.read();
  }
  
  ax=(float)accx/16384;
  ay=(float)accy/16384;
  az=(float)accz/16384;
  
  wx=(wwx/131)*(3.14/180);
  wy=(wwy/131)*(3.14/180);
  wz=(wwz/131)*(3.14/180);

}

void printMPUangles(void)
{
  
  
   
  Serial.print(ax); Serial.print('\t');
  Serial.print(ay);Serial.print('\t');
  Serial.print(az); Serial.print('\t');
  
  Serial.print(wx); Serial.print('\t');
  Serial.print(wy);Serial.print('\t');
  Serial.print(wz); Serial.print('\t');
  Serial.print('\t');
  Serial.print(diff);
  Serial.print('\t');
  Serial.print('\t');
  Serial.print(Angle[XAXIS]-offset[XAXIS]); Serial.print('\t');
  Serial.print(Angle[YAXIS]-offset[YAXIS]);Serial.print('\t');
  Serial.print(Angle[ZAXIS]-offset[ZAXIS]); Serial.print('\n');
  

 
}

void calculateOffset(){
  Serial.println("Calculating offset");
  int i;
  
  for (i=0;i<100;i=i+1)
  {
    getMPUValues();
  
    currentTime=millis()/1000.0; 
    diff=currentTime-previousTime;
    previousTime=currentTime;
  
    calculateKinematics(); 
  }
  for (i=0;i<100;i=i+1)
  {
    getMPUValues();
  
    currentTime=millis()/1000.0; 
    diff=currentTime-previousTime;
    previousTime=currentTime;
  
    calculateKinematics();
   
    offset[XAXIS]=offset[XAXIS] + Angle[XAXIS]; 
    offset[YAXIS]=offset[YAXIS] + Angle[YAXIS]; 
    offset[ZAXIS]=offset[ZAXIS] + Angle[ZAXIS]; 
    
  }
  
  offset[XAXIS]=offset[XAXIS]/100;
  offset[YAXIS]=offset[YAXIS]/100;
  offset[ZAXIS]=offset[ZAXIS]/100;
  Serial.println("Offset calculated");
}**/



