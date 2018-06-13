int leftP   = 9;
int rightP  = 10;
void setup() {
  pinMode(leftP, OUTPUT);
  pinMode(rightP, OUTPUT);
}
void loop() {
  digitalWrite(rightP, LOW);
  digitalWrite(leftP, HIGH);
  delay(1000);
  digitalWrite(rightP, LOW);
  digitalWrite(leftP, LOW);
  delay(1000);
  digitalWrite(leftP, LOW);
  digitalWrite(rightP, HIGH);
  delay(1000);
  digitalWrite(rightP, LOW);
  digitalWrite(leftP, LOW);
  delay(1000);
}
