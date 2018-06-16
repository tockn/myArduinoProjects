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
double offsetX = -0.4937493324, offsetY = -1.3529599905, offsetZ = -0.0149898648;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval;
unsigned long long preInterval;

float raw_acc_x, raw_acc_y, raw_acc_z, acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;
float calibraX = 0, calibraY = 0;


int recvData = 0;
int pulse = 0;
bool mode = false;
Servo upServo, downServo;

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
  upServo.attach(7);
  downServo.attach(4);
/*
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // 失敗、何もしない
    while(1);
  }
  Serial.println(F("ok."));
  */
}

void loop() {
/*
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // もしファイルが開けたら値を書き込む
  if (dataFile) {
    int value = analogRead(0);
    dataFile.println(value);
    dataFile.close();
    // シリアルポートにも出力
    Serial.println(value);
  }
  // ファイルが開けなかったらエラーを出力
  else {
    Serial.println(F("error opening datalog.txt"));
  } 
  */
  if(Serial.available() > 0){
    recvData = Serial.read();
    /*
    if(pulse <= 0){
      mode = false;
    }
    if(pulse >= 2400){
      mode = true;
    }

    if(mode){
      pulse--;
    }else{
      pulse++;
    }
    */
  }

 // Serial.println(recvData);

  pulse = map(recvData, 0, 255, 560, 2400);
  
  upServo.writeMicroseconds(pulse);
  downServo.writeMicroseconds(pulse);
  
}
