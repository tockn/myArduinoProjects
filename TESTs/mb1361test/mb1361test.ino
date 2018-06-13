
HardwareSerial SonicSerial(2);

void setup() {
  //SonicSerial.begin(9600);
  Serial.begin(9600);
  pinMode(34, INPUT);
}

void loop() {
  
    while( SonicSerial.available() >= 4 ){
      Serial.println((char)SonicSerial.read());
    if( SonicSerial.read() == 'R'){
      Serial.print((char)SonicSerial.read());
      Serial.print((char)SonicSerial.read());
      Serial.print((char)SonicSerial.read());
      Serial.println(" inch from Serial");
      SonicSerial.flush();
    }
  }

  int a = 0;
  for (int i = 0; i < 1; i++) {
    a += pulseIn(34, HIGH);
  }

  Serial.println(a / 147 * 2.54);
  
}
