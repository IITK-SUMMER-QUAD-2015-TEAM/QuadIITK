#define RAD2DEG 57.2957795131

#define ROLL_KP  50
#define ROLL_KI  50
#define ROLL_KD  50

#define PITCH_KP  50
#define PITCH_KI  50
#define PITCH_KD  50

#define YAW_KP  50
#define YAW_KI  50
#define YAW_KD  50

#define ROLL_MIN 300
#define ROLL_MAX  300

#define PITCH_MIN  300
#define PITCH_MAX  300

#define YAW_MIN  300
#define YAW_MAX  300

#define MANUAL 0
#define AUTOMATIC 1

PID rollPID(&gyroRate[XAXIS]);
PID pitchPID(&gyroRate[YAXIS]);
PID yawPID(&gyroRate[ZAXIS]);

void flightErrorCalculator(void);
void processHeading(void);
void setUpPIDs(void);

double heading=0;

double motorRollCommand,motorYawCommand,motorPitchCommand;

extern double gyroHeading;

void setUpPIDs(void)
{
  rollPID.setTunings(ROLL_KP,ROLL_KI,ROLL_KD);
  pitchPID.setTunings(PITCH_KP,PITCH_KI,PITCH_KD);
  yawPID.setTunings(YAW_KP,YAW_KI,YAW_KD);
  
  rollPID.setSampleTime(10);
  pitchPID.setSampleTime(10);
  yawPID.setSampleTime(10);
  
  rollPID.setMode(AUTOMATIC);
  pitchPID.setMode(AUTOMATIC);
  yawPID.setMode(AUTOMATIC);
  
  pitchPID.setOutputLimits(PITCH_MIN,PITCH_MAX);
  rollPID.setOutputLimits(ROLL_MIN,ROLL_MAX);
  yawPID.setOutputLimits(YAW_MIN,YAW_MAX);
}
void flightErrorCalculator(void)
{
  motorRollCommand = rollPID.compute(setChannelOutput(ROLL));
  motorPitchCommand = pitchPID.compute(setChannelOutput(PITCH));
  motorYawCommand = yawPID.compute(setChannelOutput(YAW));
  
  writeMotorValues();
  //motorAxisCommandPitch = updatePID(getReceiverSIData(YAXIS), -gyroRate[YAXIS]*rotationSpeedFactor, &PID[RATE_YAXIS_PID_IDX]);
  //using gyro rates...
  
}

void processHeading(void)
{
  /*heading=gyroHeading*RAD2DEG;
  relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) {
      relativeHeading += 360;
    }
    if (heading >= (setHeading + 180)) {
      relativeHeading -= 360;
    }*/
  
}
