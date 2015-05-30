#define ARMING_THRESHOLD_MAX  1800
#define ARMING_THRESHOLD_MIN  1200

#define NUM_CYCLES 25

#define RECIEVER_SCALING_FACTOR 0.0025

#define YAW 3
#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define TOTAL_CHANNELS 4//total no. of channels,can be changed in future

int8_t receiverOffset[TOTAL_CHANNELS]={0,0,0,0};

void initReceiver(void);
void printReceiverInput(void);

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

void printReceiverInput(void)
{
  Serial.print(receivers[ROLL].getDiff());Serial.print("\t");
  Serial.print(receivers[PITCH].getDiff());Serial.print("\t");
  Serial.print(receivers[THROTTLE].getDiff());Serial.print("\t");
  Serial.print(receivers[YAW].getDiff());Serial.print("\n");
}

boolean isHigh(uint8_t channel)
{
  return ((receivers[channel].getDiff()-receiverOffset[channel])>ARMING_THRESHOLD_MAX);
}

boolean isLow(uint8_t channel)
{
  return ((receivers[channel].getDiff()-receiverOffset[channel])<ARMING_THRESHOLD_MIN);
}

