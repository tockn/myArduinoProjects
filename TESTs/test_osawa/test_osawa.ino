void setup() {
  // put your setup code here, to run once:
  pinMode (7,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(10,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(7,1);
 delay(200);
 digitalWrite(7,0);
 delay(200);
 digitalWrite(3,HIGH);
 delay(200);
 digitalWrite(3,LOW);
 delay(200);
digitalWrite(10,HIGH);
 delay(200);
 digitalWrite(10,LOW);
 delay(200);

}
