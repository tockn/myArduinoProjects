#include <Servo.h>

class Hatch{
private:
  int leftPWM, rightPWM;
  bool state;
  double distance;
  bool closeSwitch, openSwitch;
  bool openBuzz, closeBuzz, buzzed;
  int buzzCount;
  int openPin, closePin, echoPin, trigPin;
  int fastInterval = 7, slowInterval = 5;

  long timer, buzzTimer;
  
  const int OPEN = 0, CLOSE = 1;
  const int LEFTMAX = 2350, LEFTMIN = 800;
  const int RIGHTMIN = 850, RIGHTMAX = 2350;
  const int TONEPIN = 3;
  const int CLOSEDISTANCE = 30;
public:
  Hatch() {

    buzzCount = 0;

    openPin = 7;
    closePin = 1;
    echoPin = 6;
    trigPin = 5;
    
    leftPWM = LEFTMIN;
    rightPWM = RIGHTMAX;
    state = OPEN;
    closeSwitch = false;
    openSwitch = false;
    openBuzz = false;
    closeBuzz = false;
    buzzed = false;

    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);

    timer = millis();
    buzzTimer = 0;
    
  }
  void calcDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite( trigPin, HIGH ); //超音波を出力
    delayMicroseconds( 10 ); //
    digitalWrite( trigPin, LOW );
    double d = pulseIn( echoPin, HIGH ); //センサからの入力
    if (d > 0) {
      d = d / 2; //往復距離を半分にする
      d = d * 340 * 100 / 1000000; // 音速を340m/sに設定 
      this->distance = d;
    }
  }
  void setOpen() {
    state = OPEN;
  }
  void setClose() {
    state = CLOSE;
  }
  void update() {
    if (analogRead(closePin) > 1000) {
      closeSwitch = true;
      if (!closeBuzz) {
        tone(TONEPIN, 1047, 200);
        closeBuzz = true;
      }
    } else {
      closeBuzz = false;
      closeSwitch = false;
    }
    Serial.println(analogRead(openPin));
    if (analogRead(openPin) > 1000) {
      openSwitch = true;
      if(!openBuzz) {
        tone(TONEPIN, 2093, 100);
        openBuzz = true;
      }
    } else {
      openBuzz = false;
      openSwitch = false;
    }
    
    if(closeSwitch && !openSwitch) state = CLOSE;
    else if(closeSwitch && openSwitch) state = OPEN;

    if (state == OPEN) {
      buzz3();
      leftPWM -= slowInterval;
      rightPWM += fastInterval;
    } else {
      leftPWM += fastInterval;
      rightPWM -= slowInterval;
    }

    if (leftPWM >= LEFTMAX) leftPWM = LEFTMAX;
    if (leftPWM <= LEFTMIN) leftPWM = LEFTMIN;

    if (rightPWM >= RIGHTMAX) rightPWM = RIGHTMAX;
    if (rightPWM <= RIGHTMIN) rightPWM = RIGHTMIN;

    if (millis() - timer >= 500 && state == OPEN) {
      calcDistance();
      timer = millis();
      //Serial.println(this->distance);
    }

    if (this->distance <= CLOSEDISTANCE) {
      state = OPEN;
    }
    
  }
  void buzz3() {
    if (!buzzed) {
      if (millis() - buzzTimer > 1000) {
        tone(TONEPIN, 1047, 300);
        buzzTimer = millis();
      }
      buzzCount++;
      if (buzzCount >= 3) {
        buzzed = true;
      }
    } else {
      buzzCount = 0;
    }
  }
  int getRight() { return rightPWM; }
  int getLeft() { return leftPWM; }
};

Hatch h;
Servo left, right;

void setup() {
  Serial.begin(9600);
  left.attach(2);
  right.attach(10);
}

void loop() {
  h.update();
  left.writeMicroseconds(h.getLeft());
  right.writeMicroseconds(h.getRight());
  /*
  Serial.print(h.getLeft());
  Serial.print("\t");
  Serial.println(h.getRight());
  */
}
