
/*
 * LEDの足が長い方ーArduinoの13番ピン
 * LEDの足が短い方ーArduinoのGND
 */


void setup() {

  pinMode(13, OUTPUT);

}

void loop() {

  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);

}
