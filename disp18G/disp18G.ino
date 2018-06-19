#include <SoftwareSerial.h>
#include <Servo.h>

Servo yawServo;

char recv;
char r, p, y;

long yawServoAngle;

bool buzz = false;
long buzzTimer = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("launch");
  yawServo.attach(6);
}

void loop() {
  if (Serial.available() >= 12) {
    if(header('r')) r = (char)Serial.read();
    if(header('p')) p = (char)Serial.read();
    if(header('y')) y = (char)Serial.read();
  }
  yawServoAngle = map(y, -90, 90, 0, 180);
  if (yawServoAngle > 180){
    yawServoAngle = 180;
  }
  if (yawServoAngle < 0) {
    yawServoAngle = 0;
  }
  yawServo.write(yawServoAngle);
  Serial.println((int)y);
  if (y > 45 || y < -45) {
    buzz = true;
  } else {
    buzz = false;
  }

  if (buzz == true) {
    if (millis() - buzzTimer > 400) {
      tone(3, 2349, 300);
      buzzTimer = millis();
    }
  }
} 

bool header(char c) {
  char cc;
  cc = (char)Serial.read();
  if (cc == c) {
    cc = (char)Serial.read();
    if (cc == c) {
      cc = (char)Serial.read();
      if (cc == c) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
  return false;
}

