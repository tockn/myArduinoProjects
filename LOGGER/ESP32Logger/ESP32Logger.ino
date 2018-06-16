#include <string>
#include <Wire.h>
#include <Adafruit_BMP085.h>


#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b


//============================global===============================================
//-2.1153638363  0.2630327940  0.4755737185
double offsetX = -2.1153638363, offsetY = 0.2630327940, offsetZ = 0.4755737185;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

float alt, hPa, TEMP;
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

bool LED = false;

Adafruit_BMP085 bmp;

void calcRotation();
void calcAltitude(void);

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void readLogNum(fs::FS &fs, const char * path, char* logNum, int len);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);

HardwareSerial GPSerial(2)/*Rx-16,Tx-17*/, BLESerial(1), SPKSerial(4);


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

  pinMode(2, OUTPUT);
  
  Wire.begin(26, 25);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(115200);
  GPSerial.begin(115200);
  BLESerial.begin(115200);
  SPKSerial.begin(9600);
  
  bmp.begin(26, 25);
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
  
  if (!bmp.begin(26, 25)) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  while (1) {}
  }
  /*
  if(!SD.begin()){
        Serial.println("Card Mount Failed");
        LED = true;
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

  listDir(SD,"/",0);
  
  readLogNum(SD,"/flight/lognum.txt", numStr, 100);
  logNum = atoi(numStr);
  logNum++;
  sprintf(logNumDir, "%s/lognum.txt", directory);
  sprintf(numStr, "%d", logNum);
  writeFile(SD, logNumDir, numStr);
  sprintf(directory, "%s/%d", directory, logNum);
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



  if(Serial.available() > 0){
    if(Serial.read() == 'a'){
    offsetAlt = alt;
    }
  }

  if(LED){
    digitalWrite(2, HIGH);
  }
  else{
    digitalWrite(2, LOW);
  }


/*
  while(GPSerial.available() > 0){
    GPSRead[i] = GPSerial.read();
    i++;
  }
  if(i >= 500){
    appendFile(SD, GPSDir, GPSRead);
    Serial.print("&");
    Serial.print(GPSRead);
    i = 0;
    sprintf(GPSRead,"");
  }
*/

  BLESerial.print("%");
  Serial.println("ok");

  while(GPSerial.available() > 0){
    BLESerial.print((char)GPSerial.read());
  }

  //Serial.printf("%.3f  [m]    %.3f  [hPa]\n", alt - offsetAlt, hPa);
  calcRotation();
  calcAltitude();
  
  sprintf(data, "%s%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",dummy,millis(),angleX,angleY,angleZ,TEMP,alt);
  sprintf(dummy, "%s", data);

  BLESerial.print("&");
  BLESerial.print(data);
  sprintf(data,"");
  sprintf(dummy,"");
  
/*
  n++;
  if(n > 5){
    appendFile(SD, RPYDir, data);
    n = 0;
    sprintf(data,"");
    sprintf(dummy,"");    
  }
*/ 
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

  angleX = (0.98 * gyro_angle_x) + (0.02 * acc_angle_x);
  angleY = (0.98 * gyro_angle_y) + (0.02 * acc_angle_y);
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


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }
    Serial.print("Read from file: ");
    while(file.available()){
        Serial.print(file.read());
    }
}

void readLogNum(fs::FS &fs, const char * path, char* logNum, int len){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }
    int i = 0;
    Serial.print("Read from file: ");
    while(file.available()){
        logNum[i] = file.read();
        i++;
        if(i >= len) return;
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
      digitalWrite(2, HIGH);
       // Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
      //LED = false;
     //   Serial.print("Message appended");
    } else {
      //LED = true;
    //    Serial.println("Append failed");
    }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

