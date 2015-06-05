#define RAD2DEG 57.2957795131

#define ROLL_KP  65
#define ROLL_KI 0
#define ROLL_KD  1

#define PITCH_KP  65
#define PITCH_KI  0
#define PITCH_KD  1

#define YAW_KP  120
#define YAW_KI  0
#define YAW_KD  0

#define ROLL_MIN -300
#define ROLL_MAX  300

#define PITCH_MIN  -300
#define PITCH_MAX  300

#define YAW_MIN  -700
#define YAW_MAX  700

#define MANUAL 0
#define AUTOMATIC 1

#define ARMING_THRESHOLD_MIN 1200
#define ARMING_THRESHOLD_MAX 1800

PID rollPID(&gyroRate[XAXIS]);
PID pitchPID(&gyroRate[YAXIS]);
PID yawPID(&gyroRate[ZAXIS]);

void flightErrorCalculator(void);
void armedCheck(void);
void processHeading(void);

void setUpPIDs(void);
void setIZero(void);

//float heading=0;

float motorRollCommand=0,motorYawCommand=0,motorPitchCommand=0;

extern float gyroHeading;

extern int systemStatus;

void setUpPIDs(void)
{
  rollPID.setSampleTime(10);
  pitchPID.setSampleTime(10);
  yawPID.setSampleTime(10);
  
  rollPID.setTunings(ROLL_KP,ROLL_KI,ROLL_KD);
  pitchPID.setTunings(PITCH_KP,PITCH_KI,PITCH_KD);
  yawPID.setTunings(YAW_KP,YAW_KI,YAW_KD);
  
  pitchPID.setOutputLimits(PITCH_MIN,PITCH_MAX);
  rollPID.setOutputLimits(ROLL_MIN,ROLL_MAX);
  yawPID.setOutputLimits(YAW_MIN,YAW_MAX);
  
  rollPID.setMode(AUTOMATIC);
  pitchPID.setMode(AUTOMATIC);
  yawPID.setMode(AUTOMATIC);
}
void flightErrorCalculator(void)
{
  motorRollCommand = rollPID.compute(setChannelOutput(ROLL));
  motorPitchCommand = pitchPID.compute(setChannelOutput(PITCH));
  motorYawCommand = yawPID.compute(setChannelOutput(YAW));
  /*Serial.print(gyroRate[XAXIS]);
  Serial.print('\t');
  Serial.println(rollPID.getSetPoint());
  Serial.print(motorPitchCommand);Serial.print('\t');
  Serial.print(motorRollCommand);Serial.print('\t');
  Serial.print(motorYawCommand);Serial.print('\n');*/
  
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

void armedCheck(void)
{
  if(isLow(THROTTLE))
  {
    if(isArmed)
    {
      if(isHigh(YAW))
      {
        isArmed=false;
        digitalWrite(LED_PIN,LOW);
        systemStatus=MAV_STATE_STANDBY;
      }
    }
    else
    {
      if(isLow(YAW))
      {
        isArmed=true;
        digitalWrite(LED_PIN,HIGH);
        systemStatus=MAV_STATE_ACTIVE;
      }
    }
  }
}

void setIZero(void)
{
  rollPID.iTerm=0;
  yawPID.iTerm=0;
  pitchPID.iTerm=0;
}
