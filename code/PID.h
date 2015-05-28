#ifndef PID_H__
#define PID_H__

#define MANUAL 0
#define AUTOMATIC 1

class PID
{
  private:
  unsigned long lastTime;
  float output;
  float lastInput;
  float kp, ki, kd;
  float outMin, outMax;
  bool inAuto = false;
  uint8_t sampleTime;//in milliseconds
  float *input;
  float SetPoint;
  public:
  PID(float *inputVar);
  float iTerm;
  void setTunings(float kP, float kI, float kD);
  void setSampleTime(uint8_t newSampleTime);
  void setOutputLimits(float Min, float Max);
  void setMode(int mode);
  void init(void);
  float compute(float setPoint);
  float getSetPoint(void);
};


#endif
