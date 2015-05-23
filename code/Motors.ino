#include<Servo.h>
//Write pins..
#define MOTOR_FRONT_PIN 2
#define MOTOR_LEFT_PIN  3
#define MOTOR_RIGHT_PIN 4
#define MOTOR_BACK_PIN  5

Servo front,left,right,back;

extern float motorRollCommand,motorYawCommand,motorPitchCommand;

void initMotors(void);
void writeMotorValues(void);

void initMotors(void)
{
  front.attach(MOTOR_FRONT_PIN);
  left.attach(MOTOR_LEFT_PIN);
  right.attach(MOTOR_RIGHT_PIN);
  back.attach(MOTOR_BACK_PIN);
}

void writeMotorValues(void)
{
 int throttle=getThrottle();
 int frontMotor,leftMotor,rightMotor,backMotor;
 frontMotor=throttle-motorPitchCommand/2+motorYawCommand;
 leftMotor=throttle+motorRollCommand/2-motorYawCommand;
 rightMotor=throttle-motorRollCommand/2-motorYawCommand;
 backMotor=throttle+motorPitchCommand/2+motorYawCommand;
 /*Serial.print(frontMotor);Serial.print('\t');
 Serial.print(leftMotor);Serial.print('\t');
 Serial.print(rightMotor);Serial.print('\t');
 Serial.print(backMotor);Serial.print('\n');*/
 /*front.writeMicroseconds(throttle-motorPitchCommand/2+motorYawCommand);
 left.writeMicroseconds(throttle+motorRollCommand/2-motorYawCommand);
 right.writeMicroseconds(throttle-motorRollCommand/2-motorYawCommand);
 back.writeMicroseconds(throttle+motorPitchCommand/2+motorYawCommand);
 */
}
