#include<Servo.h>
//Write pins..
#define MOTOR_FRONT_LEFT_PIN 2
#define MOTOR_FRONT_RIGHT_PIN  3
#define MOTOR_BACK_RIGHT_PIN 4
#define MOTOR_BACK_LEFT_PIN  5

Servo frontLeft,frontRight,backLeft,backRight;

extern float motorRollCommand,motorYawCommand,motorPitchCommand;
uint16_t  frontLeftMotorCommand=1000,frontRightMotorCommand=1000,backLeftMotorCommand=1000,backRightMotorCommand=1000;

void initMotors(void);
void writeMotorValues(void);

void initMotors(void)
{
  frontLeft.attach(MOTOR_FRONT_LEFT_PIN);
  frontRight.attach(MOTOR_FRONT_RIGHT_PIN);
  backRight.attach(MOTOR_BACK_RIGHT_PIN);
  backLeft.attach(MOTOR_BACK_LEFT_PIN);
}

void writeMotorValues(void)
{
 int throttle=getThrottle();
 
 frontLeftMotorCommand=throttle+motorRollCommand+motorPitchCommand+motorYawCommand;
 frontRightMotorCommand=throttle-motorRollCommand+motorPitchCommand-motorYawCommand;
 backLeftMotorCommand=throttle+motorRollCommand-motorPitchCommand-motorYawCommand;
 frontRightMotorCommand=throttle-motorRollCommand-motorPitchCommand+motorYawCommand;
 
 /*
 frontLeft.writeMicroseconds(frontLeftMotorCommand);
 frontRight.writeMicroseconds(frontRightMotorCommand);
 backLeft.writeMicroseconds(backLeftMotorCommand);
 backRight.writeMicroseconds(backRightMotorCommand);
 */
}

void printMotorCommands(void)
{
  Serial.print(frontLeftMotorCommand);Serial.print('\t');
  Serial.print(frontRightMotorCommand);Serial.print('\t');
  Serial.print(backRightMotorCommand);Serial.print('\t');
  Serial.print(backLeftMotorCommand);Serial.print('\n');
}
