#include<Servo.h>

int pls = 1000;

Servo servo1, servo2;

bool mode = false;

int inp = 0;

void setup() {
  servo1.attach(4);
  servo2.attach(7);
  Serial.begin(115200);
}

void loop() {
  /*
  if(Serial.available() > 0){
    char recv = Serial.read();
    if(recv == 'a') pls++;
    if(recv == 'b') pls--;
  }
  */
/*
  if(pls >= 2200) mode = true;
  if(pls <= 700) mode = false;

  if(mode)  pls--;
  else pls++;

  delay(10);
  */
/*
  inp = analogRead(5) * 0.1 + inp * 0.9;
  //inp = analogRead(5);
  pls = map(inp, 4, 1018, 560, 2400);
  */
  Serial.println(pls);  
  
  servo1.writeMicroseconds(1450);
  
}
