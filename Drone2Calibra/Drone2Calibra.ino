#include<Servo.h>


byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4,counter_channel_5, counter_channel_6, counter_channel_7, loop_counter;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, timer_channel_6, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;
unsigned long loop_timer;
long btnRight,btnLeft;

Servo esc1, esc2, esc3, esc4;
void setup() {
  // put your setup code here, to run once:

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);

  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT23);


  esc1.attach(2);
  esc2.attach(3);
  esc3.attach(4);
  esc4.attach(5);

  
}
//1610～
int availa;
int aileron, elevator, throttle = 0, rudder, button;
int num = 1;
int propoCounter = 0;

int setPointX = 0, setPointY = 0, setPointZ = 0;

void loop() {
/*
  if(propoCounter == 4)button = pulseIn(12, HIGH); 
  propoCounter++;
  if(propoCounter == 5) propoCounter = 0;


  if(elevator >= 1450 && elevator <= 1520){
    setPointX = 0;
  }
  else{
    setPointX = map(elevator, 1060, 1900, -15, 15);
  }
  if(aileron >= 1450 && aileron <= 1520){
    setPointY = 0;
  }
  else{
    setPointY = map(aileron, 1072, 1902, -15, 15);
  }
  if(rudder >= 1450 && rudder <= 1520){
    setPointZ = 0;
  }
  else{
    setPointZ = map(rudder, 1060, 1900, -15, 15);
  }
   /*
  Serial.print("\tSAYU : ");
  Serial.print(aileron);
  Serial.print("\tZENGO : ");
  Serial.print(elevator);
  Serial.print("\tJOUGE : ");
  Serial.print(throttle);
  Serial.print("\tKAITEN : ");
  Serial.print(rudder);
  Serial.print("\tBUTTON : ");
  Serial.println(button);
  

  Serial.print("\tSetPointX : ");
  Serial.print(setPointX);
  Serial.print("\tSetPointY : ");
  Serial.print(setPointY);
  Serial.print("\tSetPointZ : ");
  Serial.println(setPointZ);
 */
/*
  if(millis() > 15000){
    throttle = 1084;
  }
 */
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);

}

//プロポからの入力値を読み込む

ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001) {                                                   //Is input 8 high?
    if (last_channel_1 == 0) {                                              //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1) {                                           //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    aileron = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if (PINB & B00000010 ) {                                                  //Is input 9 high?
    if (last_channel_2 == 0) {                                              //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1) {                                           //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    elevator = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if (PINB & B00000100 ) {                                                  //Is input 10 high?
    if (last_channel_3 == 0) {                                              //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1) {                                           //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    throttle = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if (PINB & B00001000 ) {                                                  //Is input 11 high?
    if (last_channel_4 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1) {                                           //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    rudder = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  if (PINB & B00010000 ) {                                                  //Is input 11 high?
    if (last_channel_5 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_5 == 1) {
    last_channel_5 = 0;
    btnRight = current_time - timer_5;
  }
}

ISR(PCINT2_vect) {
  current_time = micros();
  if (PIND & B10000000) {
    if (last_channel_6 == 0) {
      last_channel_6 = 1;
      timer_6 = current_time;
    }
  }
  else if (last_channel_6 == 1) {
    last_channel_6 = 0;
    btnLeft = current_time - timer_6;
  }
}
