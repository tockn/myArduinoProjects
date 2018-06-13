long escTimer = 0;
long loopTimer = 0;
bool escFlag = false;
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
//pinMode(13, OUTPUT);
DDRB |= B00100000;
DDRD |= B00100000;
  long pre = millis();
  while(1){
    PORTD |= B00100000;
    delayMicroseconds(1000);
    PORTD &= B00000000;
    delay(3);
    if(millis() - pre > 3000) break;
  }

}

void loop() {
  PORTB |= B00100000;

  while(micros() - loopTimer < 4000){}
  loopTimer = micros();
  PORTD |= B00100000;

  while(PORTD != 0){
    if(micros() - loopTimer > 1300){ 
      PORTD &= B00000000;
    }
  }
}
