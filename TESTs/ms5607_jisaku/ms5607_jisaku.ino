#include<Wire.h>

#define MS5607_ADDR 0x76

uint16_t C_[6];
uint32_t raw_temp, raw_pas;
int32_t dT, TEMP;
int64_t offset, sens, P;
void setup() {
  // put your setup code here, to run once:
  readPROM();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  raw_pas = readMS(0x40);
  raw_temp = readMS(0x50);

  dT = raw_temp - ((uint32_t)C_[4] << 8);
  TEMP = 2000 + (dT * ((uint32_t)C_[5] >> 23));

  offset = ((int64_t)C_[1] << 17) + (((int64_t)C_[3] * dT) >> 6);
  sens = ((int64_t)C_[0] << 16) + (((int64_t)C_[2] * dT) >> 7);
  P = ((int64_t)raw_pas * (sens >> 21) - offset) >> 15;

  Serial.print((float)TEMP / 100);
  Serial.print("\t");
  Serial.println((float)P);
  
}

void readPROM(){ 
  for(int i=0; i<6; i++){
    Wire.beginTransmission(MS5607_ADDR);
    Wire.write(0xA2 + i*2); //PROMの最初 データシートでは0xA0に見えるが0xA2から   
    Wire.endTransmission();
 
    Wire.beginTransmission(MS5607_ADDR);
    Wire.requestFrom(MS5607_ADDR,2);
    if(Wire.available() >= 2){
      C_[i] = Wire.read() * 256 + Wire.read();
    }
  }
}

void writeMS(byte reg, byte data) {
  Wire.beginTransmission(0x76); //MPU6050、こんにちは～
  Wire.write(reg);  //住所 : reg に～
  Wire.write(data);  // "data" って書き込んでくれ～
  Wire.endTransmission();  //ありがとう、それじゃあまた明日～
}

byte readMS(byte reg) {
  Wire.beginTransmission(0x76);  //MPU6050、こんにちは～
  Wire.write(reg);  //読み込みたいから、、住所 : reg 開いといて～
  Wire.endTransmission(false);  //え？ちょっと待てって？了解、退席します～
  Wire.requestFrom(0x76, 1/*length*/, false); //用意できたかい！
  byte data =  Wire.read();  //よし、じゃあさっきの住所に入ってるデータ頂戴～
  Wire.endTransmission(true);  //ありがとう、それじゃあまた明日～
  return data;
}
