#define _b0  0.001893594048567
#define _b1 -0.002220262954039
#define _b2  0.003389066536478
#define _b3 -0.002220262954039
#define _b4  0.001893594048567
  
#define _a1 -3.362256889209355
#define _a2  4.282608240117919
#define _a3 -2.444765517272841
#define _a4  0.527149895089809

float filteredAccel[3];

struct fourthOrderFilter
{
  float  inputTm1=0,  inputTm2=0,  inputTm3=0,  inputTm4=0;
  float outputTm1=0, outputTm2=0, outputTm3=0, outputTm4=0; 
} fourthOrder[3];

void setupFourthOrder(void)
{
  fourthOrder[ZAXIS].inputTm1=-1.0;
  fourthOrder[ZAXIS].inputTm2=-1.0;
  fourthOrder[ZAXIS].inputTm3=-1.0;
  fourthOrder[ZAXIS].inputTm4=-1.0;
  
  fourthOrder[ZAXIS].outputTm1=-1.0;
  fourthOrder[ZAXIS].outputTm2=-1.0;
  fourthOrder[ZAXIS].outputTm3=-1.0;
  fourthOrder[ZAXIS].outputTm4=-1.0;
 
}

void fourthOrderFilter(void)
{
   filteredAccel[XAXIS]=computeFourthOrder(accelRate[XAXIS],&fourthOrder[XAXIS]);
   filteredAccel[YAXIS]=computeFourthOrder(accelRate[YAXIS],&fourthOrder[YAXIS]);
   filteredAccel[ZAXIS]=computeFourthOrder(accelRate[ZAXIS],&fourthOrder[ZAXIS]);
}

float computeFourthOrder(float currentInput,struct fourthOrderFilter *filterParameters)
{
   // cheby2(4,60,12.5/50)
  #define _b0  0.001893594048567
  #define _b1 -0.002220262954039
  #define _b2  0.003389066536478
  #define _b3 -0.002220262954039
  #define _b4  0.001893594048567
  
  #define _a1 -3.362256889209355
  #define _a2  4.282608240117919
  #define _a3 -2.444765517272841
  #define _a4  0.527149895089809
  
  float output;
  
  output = _b0 * currentInput                + 
           _b1 * filterParameters->inputTm1  + 
           _b2 * filterParameters->inputTm2  +
           _b3 * filterParameters->inputTm3  +
           _b4 * filterParameters->inputTm4  -
           _a1 * filterParameters->outputTm1 -
           _a2 * filterParameters->outputTm2 -
           _a3 * filterParameters->outputTm3 -
           _a4 * filterParameters->outputTm4;

  filterParameters->inputTm4 = filterParameters->inputTm3;
  filterParameters->inputTm3 = filterParameters->inputTm2;
  filterParameters->inputTm2 = filterParameters->inputTm1;
  filterParameters->inputTm1 = currentInput;
  
  filterParameters->outputTm4 = filterParameters->outputTm3;
  filterParameters->outputTm3 = filterParameters->outputTm2;
  filterParameters->outputTm2 = filterParameters->outputTm1;
  filterParameters->outputTm1 = output;
    
  return output;
}
