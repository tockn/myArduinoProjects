#include<Servo.h>


Servo servo;

int PWM = 1200;
bool up = true;

void setup() {
  servo.attach(8);
  Serial.begin(9600);
}
int angle = 0;
void loop() {
  
  if(up){
    PWM += 10;
  }else{
    PWM -= 10;
  }
  if(PWM >= 2200) PWM = 2200;
  if(PWM <= 900) PWM = 900;

  if(Serial.available() > 0){
    char data = (char)Serial.read();
    if(data == 'a'){
      up = false;
    }else if(data == 'b'){
      up = true;
    }
  }
  servo.writeMicroseconds(PWM);
  Serial.println(PWM);
}
