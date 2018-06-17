#include <Servo.h>

class Hatch {
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

  public:
    const int sonicCloseInterval = 3000; //超音波センサで乗り込みを検知してから、ハッチを閉じるまでの間隔（ミリ秒）
    const int OPEN = 0, CLOSE = 1;
    const int LEFTOPEN = 2700, LEFTCLOSE = 1280;
    const int RIGHTOPEN = 730, RIGHTCLOSE = 1850;
    const int TONEPIN = 3;
    const int CLOSEDISTANCE = 30;
    Hatch() {

      buzzCount = 0;

      openPin = 7;
      closePin = 1;
      echoPin = 6;
      trigPin = 5;

      leftPWM = LEFTOPEN;
      rightPWM = RIGHTOPEN;
      state = OPEN;
      closeSwitch = false;
      openSwitch = false;
      openBuzz = false;
      closeBuzz = false;
      buzzed = false;

      distance = 100;

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
      if (analogRead(openPin) > 1000) {
        openSwitch = true;
        if (!openBuzz) {
          tone(TONEPIN, 2093, 100);
          openBuzz = true;
        }
      } else {
        openBuzz = false;
        openSwitch = false;
      }

      if (closeSwitch && !openSwitch) state = CLOSE;
      else if (closeSwitch && openSwitch) {
        state = OPEN;
        buzzed = false;
      }

      if (state == OPEN) {
        leftPWM += slowInterval;
        rightPWM -= fastInterval;
      } else {
        leftPWM -= fastInterval;
        rightPWM += slowInterval;
      }

      if (leftPWM >= LEFTOPEN) leftPWM = LEFTOPEN;
      if (leftPWM <= LEFTCLOSE) leftPWM = LEFTCLOSE;

      if (rightPWM >= RIGHTCLOSE) rightPWM = RIGHTCLOSE;
      if (rightPWM <= RIGHTOPEN) rightPWM = RIGHTOPEN;
      if (millis() - timer >= 500 && state == OPEN) {
        calcDistance();
        timer = millis();
      }
      if (this->distance <= CLOSEDISTANCE) {
        buzz3();
      } else {
        buzzCount = 0;
      }
    }
    void buzz3() {
      Serial.println(buzzCount);
      if (!buzzed) {
        if (millis() - buzzTimer > sonicCloseInterval / 3) {
          if (buzzCount == 2){
            tone(TONEPIN, 1047, 600);
          } else {
            tone(TONEPIN, 1047, 300);
          }
          buzzTimer = millis();
          buzzCount++;
        }
        if (buzzCount >= 3) {
          state = CLOSE;
          buzzed = true;
        }
      } else {
        buzzCount = 0;
      }
    }
    int getRight() {
      return rightPWM;
    }
    int getLeft() {
      return leftPWM;
    }
    bool getState() {
      return state;
    }
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
