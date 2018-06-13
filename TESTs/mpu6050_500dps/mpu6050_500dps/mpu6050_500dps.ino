#include<Wire.h>
#include<MsTimer2.h>

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
bool dataUpdate = false;  //間隔dt秒ごとに実行するためのスイッチ
float offsetZ = 0, offsetX = 0, offsetY = 0;
float comple_angle_x[2] = {0}, comple_angle_y[2] = {0}, comple_angle_z[2] = {0};
float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;

//===============================================================================

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

//MsTimerを使いこの関数をdt秒ごとに呼び出す！
void update_(){
  dataUpdate = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(2, INPUT);
  
  // 加速度/ジャイロセンサーの初期化。
  
  Wire.begin();
  TWBR = 12;
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
  delay(2000);

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

  //百回読み込んでも静止状態と判断できなければ諦める。
  if(counter > 100){
    Serial.println("Can not confirm the stationary state.");
    break;
  }
  }  //三軸の加速度を全て足して約1Gならば静止状態と言える。
  while(abs(1 - (ax + ay + az)) > 0.5);

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
    Serial.print(offsetX);
    Serial.print("\t");
    Serial.print(offsetY);
    Serial.print("\t");
    Serial.print(offsetZ);
  }
  Serial.println("Launch program...");
  delay(2000);

  MsTimer2::set(dt, update_);
  MsTimer2::start();
  
}

void loop() {
  if(dataUpdate){
    dataUpdate = false;

//================================温度=========================================
    //温度を読み込み
    float rawTemp = (readMPU6050(MPU6050_TEMP_H) << 8) | readMPU6050(MPU6050_TEMP_L);
    float temp = (rawTemp + 12412.0) / 340.0;
//=============================================================================

//=======================================加速度================================
    //加速度を読み込み
    float raw_acc_x = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
    float raw_acc_y = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
    float raw_acc_z = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
    //単位Gへ変換
    float acc_x = raw_acc_x / 16384.0;
    float acc_y = raw_acc_y / 16384.0;
    float acc_z = raw_acc_z / 16384.0;
    //加速度センサーから角度を算出
    float acc_angle_x = atan2(acc_x, acc_z) * 360 / 2.0 / PI;
    float acc_angle_y = atan2(acc_y, acc_z) * 360 / 2.0 / PI;
//=============================================================================


//============================ジャイロセンサー=================================
    //角速度を読み込み
    float raw_gyro_x = (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
    float raw_gyro_y = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    float raw_gyro_z = (readMPU6050(MPU6050_GYRO_ZOUT_H) << 8) | readMPU6050(MPU6050_GYRO_ZOUT_L);
    //単位dps (degree per second)へ変換
    float dpsX = raw_gyro_x / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
    float dpsY = raw_gyro_y / 65.5;
    float dpsZ = raw_gyro_z / 65.5;
    //ジャイロセンサーから角度を算出(数値積分)
    gyro_angle_x += (dpsX - offsetX) * dt;
    gyro_angle_y += (dpsY - offsetY) * dt;
    gyro_angle_z += (dpsZ - offsetZ) * dt;
    
//==============================================================================

//=========================相補フィルター=======================================
    //係数
    float k = 0.98;
  
      comple_angle_x[1] = k * (comple_angle_x[0] + dpsY * dt) + ((1 - k) * acc_angle_x * -1);
  
  
//        comple_angle_y[1] = k * (comple_angle_y[0] + dpsX * dt) + (1 - k) * acc_angle_y;
        comple_angle_z[1] = 0.3 * comple_angle_z[0] + 0.7 * (comple_angle_z[0] + dpsZ * dt);

//==============================================================================
comple_angle_y[1] = 0.7 * comple_angle_y[0] + 0.3 * gyro_angle_y;

gyro_angle_x = 0.9992 * gyro_angle_x + 0.0008 * acc_angle_y;
gyro_angle_y = 0.9992 * gyro_angle_y + 0.0008 * acc_angle_x;

//=========================シリアル通信=========================================
    Serial.print(temp);
    Serial.print("\t");
    Serial.print(floor(gyro_angle_x));
    Serial.print("\t");
    Serial.print(floor(gyro_angle_y));
    Serial.print("\t");
    Serial.println(floor(gyro_angle_z));
//==============================================================================

//========================相補フィルターその２==================================
    comple_angle_x[0] = comple_angle_x[1];
    comple_angle_y[0] = comple_angle_y[1];
    comple_angle_z[0] = comple_angle_z[1];
//==============================================================================
    
  }

}
