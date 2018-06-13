
/*
 * サーボの赤い線-Arduinoの5v
 * サーボの茶色い線-ArduinoのGND
 * サーボのオレンジ線-Arduinoの10番ピン
 */


#include<Servo.h>

Servo servo;

void setup() {
  servo.attach(5);
}

void loop() {
  servo.writeMicroseconds(2000);
}
