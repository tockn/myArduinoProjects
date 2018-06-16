
#define MOTER1 9
#define MOTER2 10

void setup() {
  Serial.begin(115200);
}

char recv_data[3];
int power = 0, mappedPower = 0;
void loop() {

  while(Serial.available() > 2){
    recv_data[0] = (char)Serial.read();
    recv_data[1] = (char)Serial.read();
    recv_data[2] = Serial.read();
    /*
    Serial.print(recv_data[0]);
    Serial.print(recv_data[1]);
    Serial.println((int)recv_data[2]);
    */
    if(recv_data[0] == 't' && recv_data[1] == 't'){
      power = recv_data[2];
    }
  }

  if(power > 50){
    mappedPower = map(power, 51, 100, 120, 255);
    analogWrite(MOTER1, mappedPower);
    analogWrite(MOTER2, 0);
  }else if(power == 50){
    mappedPower = 0;
    analogWrite(MOTER2, 0);
    analogWrite(MOTER1, 0);
  }else{
    mappedPower = map(power, 49, 0, 120, 255);
    analogWrite(MOTER2, mappedPower);
    analogWrite(MOTER1, 0);
  }

  Serial.println(mappedPower);
}
