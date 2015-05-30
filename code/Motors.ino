#include<Servo.h>
//Write pins..
#define MOTOR_FRONT_LEFT_PIN 7
#define MOTOR_FRONT_RIGHT_PIN 6
#define MOTOR_BACK_LEFT_PIN 5
#define MOTOR_BACK_RIGHT_PIN  4

Servo frontLeft,frontRight,backLeft,backRight;

extern float motorRollCommand,motorYawCommand,motorPitchCommand;
int16_t  frontLeftMotorCommand=1000,frontRightMotorCommand=1000,backLeftMotorCommand=1000,backRightMotorCommand=1000;

extern File myFile;;
void initMotors(void);
void writeMotorValues(void);

void initMotors(void)
{
  frontLeft.attach(MOTOR_FRONT_LEFT_PIN);
  frontRight.attach(MOTOR_FRONT_RIGHT_PIN);
  backRight.attach(MOTOR_BACK_RIGHT_PIN);
  backLeft.attach(MOTOR_BACK_LEFT_PIN);
  
   frontLeft.writeMicroseconds(1000);
   frontRight.writeMicroseconds(1000);
   backLeft.writeMicroseconds(1000);
   backRight.writeMicroseconds(1000);
}

void writeMotorValues(void)
{
 int throttle=getThrottle();
 
 frontLeftMotorCommand=throttle+motorRollCommand-motorPitchCommand-motorYawCommand;
 frontRightMotorCommand=throttle-motorRollCommand-motorPitchCommand+motorYawCommand;
 backLeftMotorCommand=throttle+motorRollCommand+motorPitchCommand+motorYawCommand;
 backRightMotorCommand=throttle-motorRollCommand+motorPitchCommand-motorYawCommand;
 
 
 frontLeft.writeMicroseconds(frontLeftMotorCommand);
 frontRight.writeMicroseconds(frontRightMotorCommand);
 backLeft.writeMicroseconds(backLeftMotorCommand);
 backRight.writeMicroseconds(backRightMotorCommand);
 //printMotorCommands();
}

void setMotorsZero(void)
{
 frontLeftMotorCommand=0;
 frontRightMotorCommand=0;
 backLeftMotorCommand=0;
 frontRightMotorCommand=0;
  
 frontLeft.writeMicroseconds(1000);
 frontRight.writeMicroseconds(1000);
 backLeft.writeMicroseconds(1000);
 backRight.writeMicroseconds(1000);
}

void printMotorCommands(void)
{
  Serial.print(frontLeftMotorCommand);Serial.print('\t');
  Serial.print(frontRightMotorCommand);Serial.print('\t');
  Serial.print(backLeftMotorCommand);Serial.print('\t');
  Serial.print(backRightMotorCommand);Serial.print('\n');
}

void printSDMotor(void)
{
  myFile.print(frontLeftMotorCommand);myFile.print('\t');
  myFile.print(frontRightMotorCommand);myFile.print('\t');
  myFile.print(backLeftMotorCommand);myFile.print('\t');
  myFile.print(backRightMotorCommand);myFile.print('\n');
}
