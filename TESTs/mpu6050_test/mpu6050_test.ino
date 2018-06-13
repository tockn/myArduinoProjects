
#include <Wire.h>

#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42

double offsetX = 1.5568342208, offsetY = 1.6733970642, offsetZ = 1.1827219772;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angle_roll, angle_pitch, angle_yaw;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;
float start_yaw = 0;


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
  return data;
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);

    //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro

/*
  float x = 0, y = 0, z = 0;
  for(int i = 0; i < 5000; i++){
  
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU6050_ADDR, 14, (int)true);
  
    raw_acc_x = Wire.read() << 8 | Wire.read();
    raw_acc_y = Wire.read() << 8 | Wire.read();
    raw_acc_z = Wire.read() << 8 | Wire.read();
    raw_t = Wire.read() << 8 | Wire.read();
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();
      
    x += ((float)raw_gyro_x) / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
    y += ((float)raw_gyro_y) / 65.5;
    z += ((float)raw_gyro_z) / 65.5;
  }
  Serial.print(x / 5000, 10);
  Serial.print("\t");
  Serial.print(y / 5000, 10);
  Serial.print("\t");
  Serial.println(z / 5000, 10);
  delay(1000000);
  */
  
}

void loop() {
  calcRotation();
  Serial.print(angle_roll);
  Serial.print("\t");
  Serial.print(angle_pitch);
  Serial.print("\t");
  Serial.print(angle_yaw);
  Serial.println("\t");
}

void calcRotation() {

  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, 14, (int)true);

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
  dpsX -= offsetX;
  dpsY -= offsetY;
  dpsZ -= offsetZ;
  gyro_angle_x += dpsX * (interval * 0.001);
  gyro_angle_y += dpsY * (interval * 0.001);
  gyro_angle_z += dpsZ * (interval * 0.001);
  preInterval = millis();

  angle_pitch = (0.98 * gyro_angle_x) + (0.02 * acc_angle_x);
  angle_roll = (0.98 * gyro_angle_y) + (0.02 * acc_angle_y);
  angle_yaw = gyro_angle_z - start_yaw;
  gyro_angle_x = angle_pitch;
  gyro_angle_y = angle_roll;
  
  /*
    Serial.print(gyro_angle_x);
    Serial.print("\t");
    Serial.print(gyro_angle_y);
    Serial.print("\t");
    Serial.print(gyro_angle_z);
    Serial.print("\t");
    Serial.println(alt);
  */
}

