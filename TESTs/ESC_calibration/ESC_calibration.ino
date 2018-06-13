#include<Servo.h>

Servo ESC1, ESC2, ESC3, ESC4;

void setup() {
  // put your setup code here, to run once:
  ESC1.attach(3);
  ESC2.attach(6);
  ESC3.attach(10);
  ESC4.attach(9);
  
  Serial.begin(115200);
}
int n = 2000;
int p = 0;
void loop() {
  ESC1.writeMicroseconds(n);
  ESC2.writeMicroseconds(n);
  ESC3.writeMicroseconds(n);
  ESC4.writeMicroseconds(n);
/*
  if(Serial.read() == 'a'){
    n = 300;
  }
  */
  if(p > 10000){
  n = 300;
  }
    Serial.println(n);
  p++;
  
  // put your main code here, to run repeatedly:

}
