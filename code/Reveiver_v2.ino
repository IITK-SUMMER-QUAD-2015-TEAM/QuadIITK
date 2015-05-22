/*This version of Receiver uses pins 10, 11, 12, 13 of Arduino which are pins PCINT4, PCINT5, PCINT6, PCINT 7 of the Arduino. We'll be using the interrupt registers to achieve our goal
**of reading PPM*/

/*The below code is just my implementation of PinChangeInterrupt. It can be used only for pins 10,11,12&13.*/
/*#define cbi(sfr, bit)(_SFR_BYTE(sfr) &= ~_BV(bit))//to clear bit at sfr address
#define sbi(sfr, bit)(_SFR_BYTE(sfr) != _BV(bit))//to set bit at sfr address*/

uint8_t receiverPin(void)
{
   return PINB;
}

#define RECIVER_PIN1_MASK  0x10
#define RECIVER_PIN2_MASK  0x20
#define RECIVER_PIN3_MASK  0x40
#define RECIVER_PIN4_MASK  0x80

#define RECEIVERPIN1  10
#define RECEIVERPIN2  11
#define RECEIVERPIN3  12
#define RECEIVERPIN4  13

class Receiver
{
  private:
  volatile unsigned long upTime;
  volatile unsigned long diff;
  //void setUp(unsigned long up);
  //void setDown(unsigned long down);
  public:
  Receiver(void);
  void setValues(unsigned long presentTime, uint8_t condition);
  unsigned long getDiff(void);
  
} receiver1,receiver2,receiver3,receiver4;

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

void initReceiver(void);
void printReceiverInput(void);

void initReceiver(void)
{
  /*pinMode(RECEIVERPIN1,INPUT);
  pinMode(RECEIVERPIN2,INPUT);
  pinMode(RECEIVERPIN3,INPUT);
  pinMode(RECEIVERPIN4,INPUT);
  
  digitalWrite(RECIVER_PIN1_MASK,LOW);
  digitalWrite(RECIVER_PIN2_MASK,LOW);
  digitalWrite(RECIVER_PIN3_MASK,LOW);
  digitalWrite(RECIVER_PIN4_MASK,LOW);*/
  DDRB &=(0x0F);//setting the pins as input.
  PORTB &=(0x0F);//pulling down the pins.
  
  PCICR|=(1<<PCIE0);//to enable Pin Change Interrupt on pins PCINT0-7.
  //Serial.print(PCICR);
  PCMSK0=0xF0;//to allow pinchange interrupt on pins PCINT4-7 i.e. digital pins 10-13
  //delay(2000);
  //Serial.print(PCMSK0);
  //delay(2000);
  sei();
}

void printReceiverInput(void)
{
  Serial.print(receiver1.getDiff());Serial.print("\t");
  Serial.print(receiver2.getDiff());Serial.print("\t");
  Serial.print(receiver3.getDiff());Serial.print("\t");
  Serial.print(receiver4.getDiff());Serial.print("\n");
}

ISR(PCINT0_vect)
{
  //Serial.print("apple");
  static uint8_t previousValue;
  uint8_t currentValue=receiverPin();
  uint8_t changedPins=(previousValue^currentValue)&0xF0;
  
  if(changedPins&RECIVER_PIN1_MASK)
    receiver1.setValues(micros(),currentValue&RECIVER_PIN1_MASK);
  if(changedPins&RECIVER_PIN2_MASK)
    receiver2.setValues(micros(),currentValue&RECIVER_PIN2_MASK);
  if(changedPins&RECIVER_PIN3_MASK)
    receiver3.setValues(micros(),currentValue&RECIVER_PIN3_MASK);
  if(changedPins&RECIVER_PIN4_MASK)
    receiver4.setValues(micros(),currentValue&RECIVER_PIN4_MASK);
    
  previousValue=currentValue;
}
