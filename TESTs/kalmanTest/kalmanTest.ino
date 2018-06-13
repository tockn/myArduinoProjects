// 倒立振子、加速度/ジャイロセンサー(MPU-6050)版
// 2016/3/4

#include <Wire.h>
#include "KalmanFilter.h"

// 加速度/ジャイロセンサーの制御定数。
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
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75

// 加速度/ジャイロセンサーの制御変数。
KalmanFilter gKfx, gKfy; // カルマンフィルタ。
float gCalibrateY; // 初期化時の角度。（＝静止角とみなす）
long gPrevMicros; // loop()間隔の計測用。

// 倒立振子の制御変数。
float gPowerP, gPowerI, gPowerD; // 現在出力値とPID成分。

// 加速度/ジャイロセンサーへのコマンド送信。
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// 加速度/ジャイロセンサーからのデータ読み込み。
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/, false);
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}

void setup() {
  Serial.begin(115200);

  // 加速度/ジャイロセンサーの初期化。
  Wire.begin();
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: ±250dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  delay(2000);

  // 重力加速度から求めた角度をカルマンフィルタの初期値とする。
  float ax = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
  float ay = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
  float az = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
  float degRoll  = atan2(ay, az) * RAD_TO_DEG;
  float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  gKfx.setAngle(degRoll);
  gKfy.setAngle(degPitch);
  gCalibrateY = degPitch;
  gPrevMicros = micros();
}

void loop() {

  // 重力加速度から角度を求める。
  float ax = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
  float ay = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
  float az = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
  float degRoll  = atan2(ay, az) * RAD_TO_DEG;
  float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // ジャイロで角速度を求める。
  float gx = (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
  float gy = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
  float gz = (readMPU6050(MPU6050_GYRO_ZOUT_H) << 8) | readMPU6050(MPU6050_GYRO_ZOUT_L);
  float dpsX = gx / 65.5; // LSB sensitivity: 131 LSB/dps @ ±250dps
  float dpsY = gy / 65.5;
  float dpsZ = gz / 65.5;

  // カルマンフィルタで角度(x,y)を計算する。
  long curMicros = micros();
  float dt = (float)(curMicros - gPrevMicros) / 1000000; // μsec -> sec
  gPrevMicros = curMicros;
  float degX = gKfx.calcAngle(degRoll, dpsX, dt);
  float degY = gKfy.calcAngle(degPitch, dpsY, dt);
  degY -= gCalibrateY;

  Serial.print(degX);
  Serial.print("\t");
  Serial.println(degY);

  delay(1);
}

