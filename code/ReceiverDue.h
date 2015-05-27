#include "Receiver_v2.h"

#define RECEIVER_PIN1  10  
#define RECEIVER_PIN2  11  
#define RECEIVER_PIN3  12  
#define RECEIVER_PIN4  13

void rise1(void);
void rise2(void);
void rise3(void);
void rise4(void);

void fall1(void);
void fall2(void);
void fall3(void);
void fall4(void);

void initReceiver(void)
{
  attachInterrupt(RECEIVER_PIN1,rise1,RISING);
  attachInterrupt(RECEIVER_PIN2,rise2,RISING);
  attachInterrupt(RECEIVER_PIN3,rise3,RISING);
  attachInterrupt(RECEIVER_PIN4,rise4,RISING);
}

void rise1(){
  receivers[ROLL].setValues(micros(),1);
  attachInterrupt(RECEIVER_PIN1,fall1,FALLING);
}

void fall1(){
  receivers[ROLL].setValues(micros(),0);
  attachInterrupt(RECEIVER_PIN1,rise1,RISING);
}

void rise2(){
  receivers[PITCH].setValues(micros(),1);
  attachInterrupt(RECEIVER_PIN2,fall2,FALLING);
}

void fall2(){
  receivers[PITCH].setValues(micros(),0);
  attachInterrupt(RECEIVER_PIN2,rise2,RISING);
}

void rise3(){
  receivers[THROTTLE].setValues(micros(),1);
  attachInterrupt(RECEIVER_PIN3,fall3,FALLING);
}

void fall3(){
  receivers[THROTTLE].setValues(micros(),0);
  attachInterrupt(RECEIVER_PIN3,rise3,RISING);
}

void rise4(){
  receivers[YAW].setValues(micros(),1);
  attachInterrupt(RECEIVER_PIN4,fall4,FALLING);
}

void fall4(){
  receivers[YAW].setValues(micros(),0);
  attachInterrupt(RECEIVER_PIN4,rise4,RISING);
}
