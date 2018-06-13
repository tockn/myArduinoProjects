void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(5, 150);
  digitalWrite(13, LOW);
}
