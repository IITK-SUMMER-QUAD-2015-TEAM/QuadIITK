//#include <SD.h>
#include <SPI.h>
#include<Wire.h>
#define MPU 0x68

//File myfile;
int i; 
int cutoff=21;
int sampleRate=100;
int16_t MPURawData[6];
int coeff[5]={0,0,0,0,0};
int16_t Tmp;

struct filter{
  int16_t output[3]={0,0,0};
  int16_t input[3]={0,0,0};
};

struct filter MPUFilterData[6];

void setup() {
  // put your setup code here, to run once:
  init_MPU();
  delay(2000);
  getCoefficients(cutoff,sampleRate);
  Serial.begin(9600);
  //digitalWrite(53,LOW);
  //if (!SD.begin(53)) {Serial.println("Initialization failed");return;}
  //Serial.println("Initialization done");
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long previousTime=micros();
  unsigned long currentTime=micros();
   if((currentTime-previousTime)>10000)
   {
    // put your main code here, to run repeatedly:
      //myfile=SD.open("data1.csv",FILE_WRITE);
      getMPUValues();
      
      for(i=0;i<6;++i){
      //for(count=0;count<50;++count)
        MPUFilterData[i].input[0]=MPURawData[i]; 
        MPUFilterData[i].output[0]=coeff[0]*MPUFilterData[i].input[0]+coeff[1]*MPUFilterData[i].input[1]+coeff[2]*MPUFilterData[i].input[2]-coeff[3]*MPUFilterData[i].output[2]-coeff[4]*MPUFilterData[i].output[1];
        
        //Serial.print(MPUFilterData[i].output[0]);
       // if(i<3){
       // myfile.print(MPUFilterData[i].output[0]/16384);
        //}
        //else{
          //myfile.print(MPUFilterData[i].output[0]/131);
        //}
        //if (i!=5) 
          //{myfile.print(",");}//Serial.print('\t');}
        //else 
          //{myfile.print('\n');}//Serial.print('\n');}
        MPUFilterData[i].output[2]=MPUFilterData[i].output[1];
        MPUFilterData[i].output[1]=MPUFilterData[i].output[0];
        MPUFilterData[i].input[2]=MPUFilterData[i].input[1];
        MPUFilterData[i].input[1]=MPUFilterData[i].input[0];
        if(i<3){
        Serial.print(MPURawData[i]/16384);Serial.print('\t');
        Serial.print(MPUFilterData[i].output[0]/16384);Serial.print('\n'); 
        }
      else{ 
       Serial.print(MPURawData[i]/131);Serial.print('\t');
        Serial.print(MPUFilterData[i].output[0]/131);Serial.print('\n'); 
      }
 }
    //myfile.close();
   }
   Serial.print('\n');
   delay(1000);
 }

void getCoefficients(int cutoff,int sampleRate){
  
    double sqrt2 = sqrt(2);

    double QcRaw  = (2 * PI * cutoff) / sampleRate; 
    double QcWarp = tan(QcRaw); 
    double gain = 1 / ( 1 + sqrt2 / QcWarp + 2 / ( QcWarp * QcWarp ) );

    coeff[4] = ( 1 - sqrt2 / QcWarp + 2 / ( QcWarp * QcWarp ) ) * gain;
    coeff[3] = ( 2 - 2 * 2 / ( QcWarp * QcWarp ) ) * gain;
    
    coeff[0] = 1 * gain;
    coeff[1] = 2 * gain;
    coeff[2] = 1 * gain;
 }

void init_MPU(){
   Wire.begin();
   Wire.beginTransmission(MPU);
   Wire.write(0x6B);//disabling sleep mode
   Wire.write(0);
   Wire.endTransmission(true);
   Wire.beginTransmission(MPU);
   Wire.write(0x3B);//using dlp configuration to activate low pass filter
   Wire.write(2);
   Wire.endTransmission(true);
}

void getMPUValues(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);
  
  MPURawData[0]=Wire.read()<<8|Wire.read();
  MPURawData[1]=Wire.read()<<8|Wire.read();
  MPURawData[2]=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  MPURawData[3]=Wire.read()<<8|Wire.read();
  MPURawData[4]=Wire.read()<<8|Wire.read();
  MPURawData[5]=Wire.read()<<8|Wire.read();
  
  
  /*myfile.print(MPURawData[0]);myfile.print(",");
  myfile.print(MPURawData[1]);myfile.print(",");
  myfile.print(MPURawData[2]);myfile.print(",");
  myfile.print(MPURawData[3]);myfile.print(",");
  myfile.print(MPURawData[4]);myfile.print(",");
  myfile.print(MPURawData[5]);myfile.print(",");*/
  
  
}
