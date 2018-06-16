#include<Servo.h>
#include<Wire.h>

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


float dt = 0.01;  //単位s センサからの値読み取りの間隔。（積分、相補フィルターのため）
double offsetX = -1.8400697708, offsetY = 0.4478996992, offsetZ = 0.5767665505;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float anglePitch, angleRoll, angleYaw;
float interval;
unsigned long long preInterval;

float raw_acc_x, raw_acc_y, raw_acc_z, acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;
float calibraX = 0, calibraY = 0;


int upPulse = 0, downPulse = 0;

int recvData = 0;
int pulse = 0;
bool mode = false;
Servo upServo, downServo;

void calcRotation();

void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR); 
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
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

void setup() {
  Serial.begin(115200);

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
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  Serial.println("Launch program...");

  //角度の初期値を計算
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
  angleRoll = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  anglePitch = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  gyro_angle_x = anglePitch;
  gyro_angle_y = angleRoll;

  
  upServo.attach(7);
  downServo.attach(4);
}

void loop() {

  //delay(130);
  calcRotation();
  if(angleRoll > 20)  angleRoll = 20;
  if(angleRoll < -20) angleRoll = -20;

  upPulse = 5.8823 * angleRoll + 1450;
  
//  upPulse = map(angleRoll, -127, 127, 700, 2200);
  
  upServo.writeMicroseconds(upPulse);

  //Serial.println(upPulse);

/*
  upServo.write(angleRoll + 90);
  downServo.write(angleRoll + 95);
  */
  
}


void calcRotation() {

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
  interval = micros() - preInterval;
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.000001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.000001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.000001);
  preInterval = micros();

  anglePitch = (0.9995 * gyro_angle_x) + (0.0005 * acc_angle_x);
  angleRoll = (0.9995 * gyro_angle_y) + (0.0005 * acc_angle_y);
  angleYaw = gyro_angle_z;

  gyro_angle_x = anglePitch;
  gyro_angle_y = angleRoll;
  gyro_angle_z = angleYaw;

}
