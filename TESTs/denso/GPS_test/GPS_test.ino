
/*
 * GPSの5v - Arduinoの5v
 * GPSのGND - ArduinoのGND
 * GPSのTX - Arduinoの0番ピン
 * 
 * プログラムを書き込む時は一度TX - 0を抜くように！
 * 
 */

void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    Serial.print((char)Serial.read());
  }
}

