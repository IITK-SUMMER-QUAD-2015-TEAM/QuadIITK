#include<Servo.h>
//Write pins..
#define MOTOR_FRONT_PIN 2
#define MOTOR_LEFT_PIN  3
#define MOTOR_RIGHT_PIN 4
#define MOTOR_BACK_PIN  5

Servo front,left,right,back;

extern double motorRollCommand,motorYawCommand,motorPitchCommand;

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
 uint16_t throttle=getThrottle();
 front.writeMicroseconds(throttle-motorPitchCommand/2+motorYawCommand);
 left.writeMicroseconds(throttle+motorRollCommand/2-motorYawCommand);
 right.writeMicroseconds(throttle-motorRollCommand/2-motorYawCommand);
 back.writeMicroseconds(throttle+motorPitchCommand/2+motorYawCommand);
}
