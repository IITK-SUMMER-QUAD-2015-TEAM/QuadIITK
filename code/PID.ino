#include "PID.h"

PID::PID(float *inputVar):input(inputVar){};

void PID::setSampleTime(uint8_t newSampleTime)
{
  sampleTime=newSampleTime;
}

void PID::setOutputLimits(float Min,float Max)
{
  outMax=Max;
  outMin=Min;
}

void PID::setTunings(float kP, float kI, float kD)
{
  kp=kP;
  ki=kI*sampleTime/1000;
  kd=kD*1000/sampleTime;
}

void PID::setMode(int mode)
{
    if(mode && !inAuto)
      init();//back to AUTOMATIC mode  
    inAuto = mode;
}

void PID::init(void)
{
  lastInput=*input;
  iTerm=output;
  if(iTerm>outMax)
    iTerm=outMax;
  else if(iTerm<outMin)
    iTerm=outMin;
}

float PID::compute(float setPoint)
{
  if(!inAuto) 
    return 0.0;

  SetPoint=setPoint;

  float error = setPoint - *input;
  iTerm += (ki * error);
  
  if(iTerm> outMax) 
    iTerm= outMax;
  else if(iTerm< outMin) 
    iTerm= outMin;
    
  float dInput = (*input - lastInput);
  
  output = kp * error + iTerm- kd * dInput;
  
  if(output> outMax) 
    output = outMax;
  else if(output < outMin) 
    output = outMin;
  
  lastInput = *input;
  
  return output;
}

float PID::getSetPoint(void)
{
  return SetPoint;
}

