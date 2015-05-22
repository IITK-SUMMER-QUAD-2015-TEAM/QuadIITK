#ifndef PID_H__
#define PID_H__

class PID
{
  private:
  unsigned long lastTime;
  double output;
  double iTerm, lastInput;
  double kp, ki, kd;
  double outMin, outMax;
  bool inAuto = false;
  uint8_t sampleTime;
  double *input;
  public:
  PID(double &inputVar);
  void setTunings(double kP, double kI, double kD);
  void setSampleTime(uint8_t newSampleTime);
  void setOutputLimits(double Min, double Max);
  void setMode(int mode);
  void init(void);
  float compute(double setPoint);
};


#endif