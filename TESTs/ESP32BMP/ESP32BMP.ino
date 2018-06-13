#include <Wire.h>
#include <Adafruit_BMP085.h>



#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b



double offsetX = -1.7349021434, offsetY = 0.1476538085, offsetZ = 0.21899728;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

float alt = 0, hPa, TEMP;
float P0 = 1006.3;
float raw_p = 0;
float raw_t = 0;

float offsetAlt = 0;

int logNum = 0;
char numStr[100];
char logNumDir[100] = "/";
char directory[100] ="/flight";
char dummy[2048];
char GPSDir[100] = "/GPS";
char RPYDir[100] = "/RPY";

char GPSRead[1024];

char data[2048];

Adafruit_BMP085 bmp;

void calcRotation();
void calcAltitude(void);

void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)2/*length*/); 
  byte data =  Wire.read();
  Wire.read();
  Serial.println(data);
  return data;
}

void setup() {

  pinMode(2, OUTPUT);
  
  Wire.begin(26, 25);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(9600);
  
  bmp.begin(26, 25);
  Serial.println("ok");
  delay(100);
  //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }
  else{
    Serial.println("oooooooo");
  }

  //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  delay(100);
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  delay(100);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  delay(100);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  delay(100);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  delay(100);

  
  if (!bmp.begin(26, 25)) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  while (1) {}
  }
  delay(1000);

}
float preAlt = 0, maxDi = 0, di;
int n = 0;
void loop() {
  // put your main code here, to run repeatedly:
  calcRotation();
  calcAltitude();
  di = abs(alt - preAlt) * 100;
  Serial.print(alt);
  Serial.print("  m   DI : ");
  Serial.println(di);
  preAlt = alt;
  
  if(n > 10){
    if(maxDi < di){
      maxDi = di;
    }
  }
  else{
    n++;
    Serial.println(n);
  }
  
}

float pre_acc_x, pre_acc_y, pre_acc_z;
uint8_t buf[14];
void calcRotation(){

  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);

  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
  
  //単位Gへ変換
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;
  
  //加速度センサーから角度を算出
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  dpsX = ((float)raw_gyro_x) / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
  dpsY = ((float)raw_gyro_y) / 65.5;
  dpsZ = ((float)raw_gyro_z) / 65.5;
  interval = millis() - preInterval;
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);
  preInterval = millis();

  angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
  angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
  angleZ = gyro_angle_z;
  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;
/*
  Serial.print(angleX);
  Serial.print("\t");
  Serial.print(angleY);
  Serial.print("\t");
  Serial.print(angleZ);
  Serial.print("\t");
  Serial.println(alt);
    */
}

void calcAltitude(){

  for(int i = 0; i < 1; i++){
    raw_p += bmp.readPressure();
    raw_t += bmp.readTemperature();
  }
  raw_p /= 1;
  raw_t /= 1;
  
  TEMP = raw_t;
  hPa = raw_p / 100;
  alt = 44330.0 * (1.0 - pow((hPa/P0), (1.0/5.255)));
  raw_p = 0;
  raw_t = 0; 
}

