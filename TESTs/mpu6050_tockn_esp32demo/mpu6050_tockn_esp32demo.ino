#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire, 0.3, 0.7);

void setup() {
  Wire.begin(26,25);
  Serial.begin(9600);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  Serial.print(mpu6050.getAngleX());
  Serial.print("\t");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\t");
  Serial.println(mpu6050.getAngleZ());
}
