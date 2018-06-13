#include <Wire.h> //I2C Arduino Library
#define address 0x1E //0011110b, I2C 7bit address of HMC5883
 
void setup(){
  //GY-273
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}
 
void loop(){
     Serial.println(Angleread()) ;          // 方位角を返す
     delay(500) ;                            // 500ms後に繰り返す
}
int Angleread(){
  int x,y,z; //triple axis data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  return atan2((x + 20) , (y + 20)*(-1)) * RAD_TO_DEG + 180;//40と20は補正値
}
