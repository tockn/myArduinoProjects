#include <Wire.h>
#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include <MPU6050_tockn.h>
#include "SD_tockn.h"
#include "BMP180_tockn.h"

#define LED 2

HardwareSerial GPSerial(2)/*Rx-16 Tx-17*/, DispSerial(1)/* Rx-32 Tx-33*/;
MPU6050 mpu6050(Wire);
BMP180 bmp180;
SD_tockn sd;

char gpsBuf[1024];
int gpsIndex = 0;

char rpyBuf[1024];

char dispBuf[1024];

bool go = true;
bool ledHigh = false;

long preLED = 0;

float sonicAlt = 0;

TaskHandle_t th[1];

void ReadGPS() {
  while (GPSerial.available() > 0) {
    char data = (char)GPSerial.read();
    gpsBuf[gpsIndex] = data;
    gpsIndex++;
    if (gpsIndex == 1024) {
      sd.appendFile(sd.GPSDir, gpsBuf);
      gpsIndex = 0;
    }
  }
}

byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/, false);
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}


void SonicPulseIn(void *pvParameters){
  while(1){
  sonicAlt = pulseIn(34, HIGH) / 147 * 2.54;
  bmp180.update();
  }
}

void setup() {
  
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);

  if (!sd.begin()) {
    while(1){
      digitalWrite(LED, HIGH);  
    }
  }

  xTaskCreatePinnedToCore(SonicPulseIn, "SonicPulseIn", 4096, 
      NULL, 2, &th[0], 0);
  Wire.begin(26,25);
  Serial.begin(9600);
  GPSerial.begin(9600);
  DispSerial.begin(9600);

  if (!bmp180.begin(26, 25)) {
    while(1){
      digitalWrite(LED, HIGH);  
    }
  }
  
  mpu6050.begin();
  mpu6050.setGyroOffsets(1.45, -1.22, 0.21);
  if (readMPU6050(0x75) != 115) {
    while(1) {
      digitalWrite(LED, HIGH);
    }
  }
    pinMode(34, INPUT);
}

void loop() {

  mpu6050.update();
  sprintf(rpyBuf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
    mpu6050.getAngleX(), mpu6050.getAngleY(), mpu6050.getAngleZ(),
    bmp180.getAltitude(), bmp180.getTemperature(), sonicAlt);
  ReadGPS();
  
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.print(mpu6050.getAngleZ());
  Serial.print("\talt : ");
  Serial.print(bmp180.getAltitude());
  Serial.print("\ttemp : ");
  Serial.print(bmp180.getTemperature());
  Serial.print("\tsonicAlt : ");
  Serial.println(sonicAlt);
  


  sd.appendFile(sd.RPYDir, rpyBuf);
  sd.appendFile(sd.GPSDir, gpsBuf);
  gpsIndex = 0;

  DispSerial.print("rrr"); DispSerial.write((char)mpu6050.getAngleX());
  DispSerial.print("ppp"); DispSerial.write((char)mpu6050.getAngleY());
  DispSerial.print("yyy"); DispSerial.write((char)mpu6050.getAngleZ());
  
  if (sd.stack > 0 && !ledHigh && millis() - preLED > 20) {
    digitalWrite(LED, HIGH);
    ledHigh = true;
    preLED = millis();
    sd.stack--;
  }

  if (millis() - preLED > 20 && ledHigh) {
    digitalWrite(LED, LOW);
    ledHigh = false;
  }

}


