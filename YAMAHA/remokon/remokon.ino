
int joyStick = 0;
int sendData = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {

  joyStick = joyStick * 0.9 + analogRead(0) * 0.1;
  
  sendData = map(joyStick, 0, 1023, 0, 255);
  if(sendData < 0)  sendData = 0;
  if(sendData > 255) sendData = 255;

  Serial.write(sendData);
  
}

