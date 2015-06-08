#define HMC5883_ADDRESS 0x1E
#define MODE_ADDRESS 0x02

#define DEC_ANGLE 0.5 //in degrees
#define NUM_AXIS 3
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define M1
#define M1
#define M1
#define MAGNET_SCALING_FACTOR 
double heading; 
int16_t magnetRaw[NUM_AXIS]={0,0,0};
float magnetVal[NUM_AXIS]={0,0,0};

void initMagnet(void)
{
  Wire.begin();
  
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(MODE_ADDRESS);
  Wire.write(0x00);//continuous mode enable
  Wire.endTransmission(true);
  
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(0x01);
  Wire.write(0x20);//setting range to +/- 1.3 gauss.
  Wire.endTransmission(true);  
}

void getMagnet(void)
{
   Wire.beginTransmission(HMC5883_ADDRESS);
   Wire.write(0x03);
   Wire.endTransmission(false);
   Wire.requestFrom(HMC5883_ADDRESS,6,true);
   
   magnetRaw[XAXIS]=Wire.read()<<8|Wire.read();
   magnetRaw[YAXIS]=(Wire.read()<<8|Wire.read())*-1;
   magnetRaw[ZAXIS]=(Wire.read()<<8|Wire.read())*-1;
   
}

void printMagnet(void)
{
  Serial.print(magnetRaw[XAXIS]);Serial.print('\t');
  Serial.print(magnetRaw[YAXIS]);Serial.print('\t');
  Serial.print(magnetRaw[ZAXIS]);Serial.print('\n');
}

void getHeading(){
  heading=(float)atan2((float)magnetRaw[YAXIS],(float)magnetRaw[XAXIS])*180/PI;
 
  heading+=DEC_ANGLE;
  
  if(heading<-PI)
   heading+=2*PI;
  else if(heading>PI)
   heading-=2*PI;
}
