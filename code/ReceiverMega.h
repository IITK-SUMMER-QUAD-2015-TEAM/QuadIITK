/*This version of Receiver uses pins 10, 11, 12, 13 of Arduino Mega which are pins PCINT4, PCINT5, PCINT6, PCINT 7 of the Arduino. We'll be using the interrupt registers to achieve our goal
**of reading PPM*/

/*The below code is just my implementation of PinChangeInterrupt. It can be used only for pins 10,11,12&13.*/
#define RECIVER_PIN1_MASK  0x10
#define RECIVER_PIN2_MASK  0x20
#define RECIVER_PIN3_MASK  0x40
#define RECIVER_PIN4_MASK  0x80

//#define RECEIVER_PIN1  10  #define RECEIVER_PIN2  11  #define RECEIVER_PIN3  12  #define RECEIVER_PIN4  13

#include "Receiver_v2.h"

uint8_t receiverPin(void)
{
   return PINB;
}

void initReceiver(void)
{
  DDRB &=(0x0F);//setting the pins as input.
  PORTB &=(0x0F);//pulling down the pins.
  
  PCICR|=(1<<PCIE0);//to enable Pin Change Interrupt on pins PCINT0-7.
  PCMSK0=0xF0;//to allow pinchange interrupt on pins PCINT4-7 i.e. digital pins 10-13
  
  sei();
}

ISR(PCINT0_vect)
{
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
