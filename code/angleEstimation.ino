/*Gyro output rate: 8kHz (if we do'nt use DLPF and 1kHz if do use DLPF)
**Accelo output rate: 1kHz*/

/*All of the below regiater addresses are for the most significant bits, i.e. for bits[15:8]*/

#define DIFF 0.01

const float Kp = 32.3;
const float  Ki = 12.00;
					
float halfT = 0.0;                					
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;      
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		

float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;
float angle[3]={0.0,0.0,0.0};
float angleCorrected[3]={0.0,0.0,0.0};
float ax;
float ay;
float az;
float wx;
float wy;
float wz;


float offset[3]={0.0,0.0,0.0};

boolean changedSign(float a,float b){
  if ((a>0&&b<0)||(a<0&&b>0)) return true;
  else return false;
  
}

void quaternionECF() {
  
  float norm;
  float gx, gy, gz;
  float q0i, q1i, q2i, q3i;
  float ex, ey, ez;
    
  halfT = DIFF/2;
  
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
  angle[XAXIS]  =(180/3.14)*atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  if (angle[XAXIS]<0) angle[XAXIS]=angle[XAXIS]+180;
  else angle[XAXIS]=angle[XAXIS]-180;
  
  angle[YAXIS] = -(180/3.14)*asin(2 * (q0*q2 - q1*q3));
  angle[ZAXIS]   = (180/3.14)*atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
  
  angleCorrected[XAXIS]=angleWithOffset(XAXIS)*-1;
  angleCorrected[YAXIS]=angleWithOffset(YAXIS);
  angleCorrected[ZAXIS]=angleWithOffset(ZAXIS);
}


void calculateKinematics() 
{
   ax=accelRate[XAXIS];
   ay=accelRate[YAXIS];
   az=accelRate[ZAXIS];
   
   wx=gyroRate[XAXIS];
   wy=gyroRate[YAXIS];
   wz=gyroRate[ZAXIS];
   
   quaternionECF();
   eulerAngles();
}



void printAngles(void)
{
  Serial.print(angleCorrected[XAXIS]); Serial.print('\t');
  Serial.print(angleCorrected[YAXIS]);Serial.print('\t');
  Serial.print(angleCorrected[ZAXIS]); Serial.print('\n');   
}

void calculateAngleOffset(){
  Serial.println("Calculating offset");
  int i;
  
  for (i=0;i<100;++i)
  {
    measureIMUSensors();
    calculateKinematics();
    delay(10);
  }
  for (i=0;i<100;++i)
  {
    measureIMUSensors();
  
    calculateKinematics();
   
    offset[XAXIS]=offset[XAXIS] + angle[XAXIS]; 
    offset[YAXIS]=offset[YAXIS] + angle[YAXIS]; 
    offset[ZAXIS]=offset[ZAXIS] + angle[ZAXIS]; 
    delay(10);
  }
  
  offset[XAXIS]=offset[XAXIS]/100;
  offset[YAXIS]=offset[YAXIS]/100;
  offset[ZAXIS]=offset[ZAXIS]/100;
 // Serial.println("Offset calculated");
}

float angleWithOffset(uint8_t axis)
{
  return angle[axis]-offset[axis];
}

