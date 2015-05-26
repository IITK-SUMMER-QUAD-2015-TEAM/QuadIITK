/*This version of Receiver uses pins 10, 11, 12, 13 of Arduino which are pins PCINT4, PCINT5, PCINT6, PCINT 7 of the Arduino. We'll be using the interrupt registers to achieve our goal
**of reading PPM*/

/*The below code is just my implementation of PinChangeInterrupt. It can be used only for pins 10,11,12&13.*/
#define RECIVER_PIN1_MASK  0x10
#define RECIVER_PIN2_MASK  0x20
#define RECIVER_PIN3_MASK  0x40
#define RECIVER_PIN4_MASK  0x80

#define RECEIVERPIN1  10
#define RECEIVERPIN2  11
#define RECEIVERPIN3  12
#define RECEIVERPIN4  13
//yaw->10 throttle 11 pitch roll
#define YAW 3
#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define TOTAL_CHANNELS 4//total no. of channels,can be changed in future

#define NUM_CYCLES 25

#define RECIEVER_SCALING_FACTOR 0.005

int8_t receiverOffset[TOTAL_CHANNELS]={0,0,0,0};

void initReceiver(void);
void printReceiverInput(void);

uint8_t receiverPin(void)
{
   return PINB;
}
class Receiver
{
  private:
  volatile unsigned long upTime;
  volatile unsigned long diff;
  public:
  Receiver(void);
  void setValues(unsigned long presentTime, uint8_t condition);
  unsigned long getDiff(void);
  
};

Receiver::Receiver()
{
}

unsigned long Receiver::getDiff(void)
{
  return diff;
}

void Receiver::setValues(unsigned long presentTime,uint8_t condition)
{
  if(condition)
    upTime=presentTime;
  else
     diff=presentTime-upTime;//presentTime here is the time when there is afalling edge.
}

Receiver receivers[TOTAL_CHANNELS];

void setOffset()
{
   unsigned long receiverSum[TOTAL_CHANNELS]={0,0,0,0};
   for (int i=0;i<NUM_CYCLES;++i)
   {
     receiverSum[YAW]+=receivers[YAW].getDiff();
     receiverSum[ROLL]+=receivers[ROLL].getDiff();
     receiverSum[PITCH]+=receivers[PITCH].getDiff();
     receiverSum[THROTTLE]+=receivers[THROTTLE].getDiff();
     delay(20);
   }
   receiverOffset[YAW]=(receiverSum[YAW]/NUM_CYCLES)-1500;
   receiverOffset[ROLL]=(receiverSum[ROLL]/NUM_CYCLES)-1500;
   receiverOffset[PITCH]=(receiverSum[PITCH]/NUM_CYCLES)-1500;
   receiverOffset[THROTTLE]=(receiverSum[THROTTLE]/NUM_CYCLES)-1500;
}

float setChannelOutput(uint8_t channel)
{
    int temp=receivers[channel].getDiff()-receiverOffset[channel];
    return (float)((temp-1500)*RECIEVER_SCALING_FACTOR);
}

uint16_t getThrottle(void)
{
  return receivers[THROTTLE].getDiff();
}

void initReceiver(void)
{
  DDRB &=(0x0F);//setting the pins as input.
  PORTB &=(0x0F);//pulling down the pins.
  
  PCICR|=(1<<PCIE0);//to enable Pin Change Interrupt on pins PCINT0-7.
  PCMSK0=0xF0;//to allow pinchange interrupt on pins PCINT4-7 i.e. digital pins 10-13
  
  sei();
}

void printReceiverInput(void)
{
  Serial.print(receivers[ROLL].getDiff());Serial.print("\t");
  Serial.print(receivers[PITCH].getDiff());Serial.print("\t");
  Serial.print(receivers[THROTTLE].getDiff());Serial.print("\t");
  Serial.print(receivers[YAW].getDiff());Serial.print("\n");
}

ISR(PCINT0_vect)
{
  //Serial.print("apple");
  static uint8_t previousValue;
  uint8_t currentValue=receiverPin();
  uint8_t changedPins=(previousValue^currentValue)&0xF0;
  
  if(changedPins&RECIVER_PIN1_MASK)
    receivers[ROLL].setValues(micros(),currentValue&RECIVER_PIN1_MASK);
  if(changedPins&RECIVER_PIN2_MASK)
    receivers[PITCH].setValues(micros(),currentValue&RECIVER_PIN2_MASK);
  if(changedPins&RECIVER_PIN3_MASK)
    receivers[THROTTLE].setValues(micros(),currentValue&RECIVER_PIN3_MASK);
  if(changedPins&RECIVER_PIN4_MASK)
    receivers[YAW].setValues(micros(),currentValue&RECIVER_PIN4_MASK);
    
  previousValue=currentValue;
}
