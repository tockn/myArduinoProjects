#include <stdio.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>

#include <SD.h>
#include <SPI.h>

#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b


//============================global===============================================

SoftwareSerial gpsPort(3, 4);

float offsetX = -1.7349021434, offsetY = 0.1476538085, offsetZ = 0.21899728;
float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

float alt, hPa, TEMP;
float P0 = 1006.3;
float raw_p = 0;
float raw_t = 0;

float offsetAlt = 0;

uint8_t logNum = 0;
String directory ="flight/";
String GPSDir = "";
String RPYDir = "";

char fetchData[200];
uint8_t fetchDataLength;

String data = "";

File GPSFile;
File RPYFile;

Adafruit_BMP085 bmp;

void culcRotation();
void culcAltitude(void);
/*
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void readLogNum(fs::FS &fs, const char * path, char* logNum, int len);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);

HardwareSerial GPSerial(2);
*/
//===============================================================================

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

  gpsPort.begin(9600);
  
  pinMode(2, OUTPUT);
  
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(9600);
  
  bmp.begin();
  Serial.println("ok");
  delay(100);
  //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
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

  
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  while (1) {}
  }

  pinMode(10, OUTPUT);
  
  if(!SD.begin(4)){
     Serial.println("Card Mount Failed");
     while(true);
  }

  File logNumFile = SD.open("flight/lognum.txt");

  if(logNumFile){
    byte buffer[5];
    int len = logNumFile.available();
    logNumFile.read(buffer, len);
    logNum = atoi(buffer);
    Serial.println(logNum);
    logNum++;
    Serial.println(logNum);
    logNumFile.close();
  }
  else{
    Serial.println("read error");
    logNumFile.close();
    while(true);
  }
  logNumFile = SD.open("flight/lognum.txt", FILE_WRITE);
  if(logNumFile){
    logNumFile.seek(0);
    logNumFile.print(logNum);
    logNumFile.close();
  }
  else{
    Serial.println("write error");
    logNumFile.close();
    while(true);
  }
  directory += (String)logNum;
  SD.mkdir(directory);
  GPSDir = directory + "/GPS";
  RPYDir = directory + "/RPY";
  SD.mkdir(GPSDir);
  SD.mkdir(RPYDir);
  GPSDir += "/gps.txt";
  RPYDir += "/rpy.txt";

  
 /*
  createDir(SD, directory);
  sprintf(GPSDir, "%s/GPS", directory);
  sprintf(RPYDir, "%s/RPY", directory);
  createDir(SD, GPSDir);
  createDir(SD, RPYDir);
  sprintf(GPSDir, "%s/GPS/gps.txt", directory);
  sprintf(RPYDir, "%s/RPY/rpy.txt", directory);
  writeFile(SD, GPSDir, "\n");
  writeFile(SD, RPYDir, "\n");
*/
}



int n = 0, i = 0;

void loop() {
  
  if(gps_fetch()){
    // GPSデータ書き込み
    GPSFile = SD.open(GPSDir ,FILE_WRITE);
    GPSFile.print(fetchData);
    GPSFile.close();
    Serial.print(fetchData);
  }
  
  culcRotation();
  culcAltitude();
  data = String(millis()) + "," + String(angleX) + "," + String(angleY) + "," + String(angleZ) + "," + String(TEMP) + "," + String(alt);
  n++;
  if(n > 0){
    RPYFile = SD.open(RPYDir, FILE_WRITE);
    RPYFile.print(data);
    RPYFile.close();
    n = 0;
    //Serial.println(RPYDir);
    Serial.print(data);  
  } 
  
}


float pre_acc_x, pre_acc_y, pre_acc_z;
uint8_t buf[14];
void culcRotation(){

  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

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

  Serial.print(angleX);
  Serial.print("\t");
  Serial.print(angleY);
  Serial.print("\t");
  Serial.print(angleZ);
  Serial.print("\t");
  Serial.print(TEMP);
  Serial.print("\t");
  Serial.println(alt);
    
}

void culcAltitude(){

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

bool gps_fetch(void)
{
  if( gpsPort.available() ){
    unsigned long last_time = millis();
    uint8_t preamble = 0;
    uint8_t len = 0;
    fetchDataLength = 0;
    do {
      while( gpsPort.available() ){
        char c = gpsPort.read();
        if( c == '$' ){
          preamble++;
        } 
        if( preamble ){
          if( len < sizeof(fetchData) ){
            fetchData[len++] = c;
          }
          if( c == '\n'){
            fetchDataLength = len;
          }
        }
        last_time = millis();
      }
      delay(1);
    }
    while( millis() - last_time < 10 );
    return fetchDataLength > 0;
  }
  return false;
}
