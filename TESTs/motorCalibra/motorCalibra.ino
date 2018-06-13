#include<Servo.h>

Servo esc;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  esc.attach(3);

  
}
float n = 1599;
bool sw = false;
void loop() {
  // put your main code here, to run repeatedly:
  char recv_data = Serial.read();
  if(recv_data == 'a'){
    n+=1;
  }
  if(recv_data == 'b'){
    n+=100;
  }
  if(recv_data == 'c'){
    n-=1;
  }
  if(recv_data == 'd'){
    n-=100;
  }
  

  esc.writeMicroseconds(n);
  if(sw){
  n+=10;
  }
  Serial.println(n);
  delay(200);
}
