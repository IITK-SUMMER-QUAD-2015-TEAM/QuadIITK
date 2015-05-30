#include<SPI.h>
#include<Wire.h>

#define CUTOFF 21// The cutoff frequency
#define SAMPLE_RATE 100

#define NUM_FILTERS 6

enum {ACCEL_X , ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z};

int16_t Tmp;

class filter
{
  public:
  static float coeff[5];
  float filterInput(float input);
  private:
  float output[3]={0,0,0};
  float input[3]={0,0,0};
};
float filter::coeff[5]; 

float filter::filterInput(float Input)
{
  input[0]=Input; 
  output[0]=coeff[0]*input[0]+coeff[1]*input[1]+coeff[2]*input[2]-coeff[3]*output[2]-coeff[4]*output[1];
        
  output[2]=output[1];
  output[1]=output[0];
  input[2]=input[1];
  input[1]=input[0];
  
  return output[0];
}

filter filters[NUM_FILTERS];

void getCoefficients(){
  
    double sqrt2 = sqrt(2);

    double QcRaw  = (2 * PI * CUTOFF) / SAMPLE_RATE; 
    double QcWarp = tan(QcRaw); 
    double gain = 1 / ( 1 + sqrt2 / QcWarp + 2 / ( QcWarp * QcWarp ) );

    filter::coeff[4] = ( 1 - sqrt2 / QcWarp + 2 / ( QcWarp * QcWarp ) ) * gain;
    filter::coeff[3] = ( 2 - 2 * 2 / ( QcWarp * QcWarp ) ) * gain;
    
    filter::coeff[0] = 1 * gain;
    filter::coeff[1] = 2 * gain;
    filter::coeff[2] = 1 * gain;
 }
