/*#define speed_int1 INT0
#define speed_int2 INT1
#define speed_int3 INT4
#define speed_int4 INT5

volatile unsigned long upTime1;
volatile unsigned long downTime1;
volatile unsigned long upTime2;
volatile unsigned long downTime2;
volatile unsigned long upTime3;
volatile unsigned long downTime3;
volatile unsigned long upTime4;
volatile unsigned long downTime4;

volatile int diff1;
volatile int diff2;
volatile int diff3;
volatile int diff4;

void initReceiver(void);
void printReceiverInput(void);

void initReceiver(void)
{
  attachInterrupt(speed_int1,rise1,RISING);
  attachInterrupt(speed_int2,rise2,RISING);
  attachInterrupt(speed_int3,rise3,RISING);
  attachInterrupt(speed_int4,rise4,RISING);
}

void printReceiverInput(void)
{
  Serial.print(diff1);Serial.print("\t");
  Serial.print(diff2);Serial.print("\t");
  Serial.print(diff3);Serial.print("\t");
  Serial.print(diff4);Serial.print("\n");
}

void rise1(){
  upTime1=micros();
  attachInterrupt(speed_int1,fall1,FALLING);
}

void fall1(){
  downTime1=micros();
  diff1=downTime1-upTime1;
  attachInterrupt(speed_int1,rise1,RISING);
}

void rise2(){
  upTime2=micros();
  attachInterrupt(speed_int2,fall2,FALLING);
}

void fall2(){
  downTime2=micros();
  diff2=downTime2-upTime2;
  attachInterrupt(speed_int2,rise2,RISING);
}

void rise3(){
  upTime3=micros();
  attachInterrupt(speed_int3,fall3,FALLING);
}

void fall3(){
  downTime3=micros();
  diff3=downTime3-upTime3;
  attachInterrupt(speed_int3,rise3,RISING);
}

void rise4(){
  upTime4=micros();
  attachInterrupt(speed_int4,fall4,FALLING);
}

void fall4(){
  downTime4=micros();
  diff4=downTime4-upTime4;
  attachInterrupt(speed_int4,rise4,RISING);
}*/
