#include<Servo.h>
#include<Wire.h>
#include<EEPROM.h>

//レジスタアドレス
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75


//============グローバル変数宣言=================================================

float dt = 0.01;  //単位s センサからの値読み取りの間隔。（積分、相補フィルターのため）
double offsetX = 1.6941, offsetY = -2.2198, offsetZ = -0.3398;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;

int low, high;

float raw_acc_x, raw_acc_y, raw_acc_z, acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

float goalAngleX = 0, goalAngleY = 0, goalAngleZ = 0;

void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR); //MPU6050、こんにちは～
  Wire.write(reg);  //住所 : reg に～
  Wire.write(data);  // "data" って書き込んでくれ～
  Wire.endTransmission();  //ありがとう、それじゃあまた明日～
}

byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);  //MPU6050、こんにちは～
  Wire.write(reg);  //読み込みたいから、、住所 : reg 開いといて～
  Wire.endTransmission(false);  //え？ちょっと待てって？了解、退席します～
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/, false); //用意できたかい！
  byte data =  Wire.read();  //よし、じゃあさっきの住所に入ってるデータ頂戴～
  Wire.endTransmission(true);  //ありがとう、それじゃあまた明日～
  return data;
}



void setup() {
  // put your setup code here, to run once:

  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial.println("start");
/*
  Serial.println(EEPROM.read(10));
  Serial.println(EEPROM.read(11));
  Serial.println(EEPROM.read(12));
  Serial.println(EEPROM.read(13));
  Serial.println(EEPROM.read(14));
  Serial.println(EEPROM.read(15));
  Serial.println(EEPROM.read(16));
  Serial.println(EEPROM.read(17));
  Serial.println(EEPROM.read(18));
*/
  
  Serial.println(EEPROM.read(0));
  Serial.println(EEPROM.read(1));
  Serial.println(EEPROM.read(2));
  Serial.println(EEPROM.read(3));
  Serial.println(EEPROM.read(4));
  
  Wire.begin();
  TWBR = 12;
  delay(100);
  //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }
  
  //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  delay(100);
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  delay(100);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  delay(100);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  delay(100);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  delay(100);
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  delay(100);
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  delay(100);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  delay(100);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  delay(100);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  delay(100);

  float aax, aay, aaz, ax, ay, az, counter = 0;

  //静止状態であることを確認
  do{
  
  aax = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
  aay = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
  aaz = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);

  ax = aax / 16384.0;
  ay = aay / 16384.0;
  az = aaz / 16384.0;
  counter += 1;
  delay(10);

  //百回読み込んでも静止状態と判断できなければ諦める。
  if(counter > 100){
    Serial.println("Can not confirm the stationary state.");
    while(true);
  }
  }  //三軸の加速度を全て足して約1Gならば静止状態と言える。
  while(abs(1 - (ax + ay + az)) > 0.3);

  if(counter <= 100){
    Serial.println("Confirm stationary state!");
    delay(1000);
    Serial.println("Calculate calibration...");
    Serial.println("Please wait... Don't touch sensor...");

    for(int i = 0; i < 3000; i++){
      float raw_gyro_x = (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
      float raw_gyro_y = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
      float raw_gyro_z = (readMPU6050(MPU6050_GYRO_ZOUT_H) << 8) | readMPU6050(MPU6050_GYRO_ZOUT_L);
      offsetX += raw_gyro_x / 65.5;
      offsetY += raw_gyro_y / 65.5;
      offsetZ += raw_gyro_z / 65.5;
    }
    offsetX /= 3000;
    offsetY /= 3000;
    offsetZ /= 3000;
    Serial.print("Success!  offset : ");
    Serial.print(offsetX,10);
    Serial.print("\t");
    Serial.print(offsetY,10);
    Serial.print("\t");
    Serial.print(offsetZ,10);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
