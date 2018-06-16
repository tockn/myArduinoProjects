#include <Wire.h>
#include "Adafruit_BMP085.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include <MPU6050_tockn.h>
#include "SD_tockn.h"

HardwareSerial GPSerial(2)/*Rx-16 Tx-17*/, DispSerial(1)/* Rx-32 Tx-33*/;
Adafruit_BMP085 bmp;
MPU6050 mpu6050(Wire, 0.2, 0.8);
SD_tockn sd;

char gpsBuf[1024];
int gpsIndex = 0;

char rpyBuf[1024];

char dispBuf[1024];

bool go = true;

float sonicAlt = 0;

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


void SonicPulseIn(void *pvParameters){
  while(1){
  sonicAlt = pulseIn(34, HIGH) / 58;
  }
}

void setup() {

  //xTaskCreatePinnedToCore(SonicPulseIn, "SonicPulseIn", 4096, NULL, 1, NULL, 1);
  
  Wire.begin(26,25);
  Serial.begin(9600);
  GPSerial.begin(9600);
  DispSerial.begin(9600);

  mpu6050.begin();
  mpu6050.setGyroOffsets(2.84, 1.93, 1.93);
  /*
  if (!bmp.begin(26, 25, 0)) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {}
  }
  */
  pinMode(34, INPUT);
  if(!SD.begin()) {
    Serial.println("unti");
  }
  if (!sd.begin()) {
    Serial.println("Fatal SD error");
    while(1){}
  }
  
}

void loop() {

  mpu6050.update();

  sprintf(rpyBuf, "%.2f,%.2f,%.2f\n", 
    mpu6050.getAngleX(), mpu6050.getAngleY(), mpu6050.getAngleZ());
  ReadGPS();
  sd.appendFile(sd.RPYDir, rpyBuf);
  sd.appendFile(sd.GPSDir, gpsBuf);
  gpsIndex = 0;

  DispSerial.print("rrr"); DispSerial.write((char)mpu6050.getAngleX());
  DispSerial.print("ppp"); DispSerial.write((char)mpu6050.getAngleY());
  DispSerial.print("yyy"); DispSerial.write((char)mpu6050.getAngleZ());
  
}
