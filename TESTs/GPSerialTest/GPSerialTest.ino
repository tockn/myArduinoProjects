
HardwareSerial GPSerial(2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  GPSerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(GPSerial.available() > 0) {
    Serial.print((char)GPSerial.read());
  }
}
