
HardwareSerial Serial1(2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
}
int n = 0;
void loop() {
  if(Serial1.available() > 0){
    Serial.write(Serial1.read());
  }

  // put your main code here, to run repeatedly:

}
