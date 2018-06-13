#include<Servo.h>
#include<Wire.h>
#include<EEPROM.h>

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
double offsetX = 1.6941, offsetY = -2.2198, offsetZ = -0.3398;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;

int low, high;

float raw_acc_x, raw_acc_y, raw_acc_z, acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

float goalAngleX = 0, goalAngleY = 0, goalAngleZ = 0;

Servo esc1, esc2, esc3, esc4;

unsigned char recv_data = 0;
char recv_charData;
float leftHatY = 0;
float  esc1Power = 0, esc2Power = 0, esc3Power = 0, esc4Power = 0;
float escError_roll = 0;
float escError_pitch = 0;
float escError_yaw = 0;

float culcTimer = 0;
void culcRotation(void);
//1.6   2.1

float Kp_roll = 3.7, Ki_roll = 0.008, Kd_roll = 16.8;


float errorSum_roll = 0;
float errorSum_pitch = 0;
float errorSum_yaw = 0;
float preError_roll = 0;
float preError_pitch = 0;
float preError_yow = 0;

int INPUT_MODE = 0;
int CULC_MODE = 1;
int STATE = 0;

/*
PID esc2PID(&gyro_angle_y, &esc2Power, &angleY, 0.6, 8, 2, DIRECT);
PID esc3PID(&gyro_angle_x, &esc3Power, &angleX, 0.6, 8, 2, DIRECT);
PID esc4PID(&gyro_angle_y, &esc4Power, &angleY, 0.6, 8, 2, DIRECT);
*/

void culcPID(float &target, float now, float goal, float &errorSum, float &preError, float Kp, float Ki, float Kd);
bool checkBuffer(char str[8]);

char circle[8] = {'c', 'i', 'r', 'c', 'l', 'e', '0', '1'};
char square1[8] ={'s', 'q', 'u', 'a', 'r', 'e', '0', '1'};
char cross[8] = {'c', 'r', 'o', 's', 's', '0', '1', '2'};
char triangle[8] = {'t', 'r', 'i', 'a', 'n', 'g', 'l', 'e'};
char up[8] = {'u', 'p', '0', '1', '2', '3', '4', '5'};
char down[8] = {'d', 'o', 'w', 'n', '0', '1', '2', '3'};
char right[8] = {'r', 'i', 'g', 'h', 't', '0', '1', '2'};
char left[8] = {'l', 'e', 'f', 't', '0', '1', '2', '3'};



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
/*
//MsTimerを使いこの関数をdt秒ごとに呼び出す！
void update_(){
  dataUpdate = true;
}
*/
// 0.44   0.21

void clamp(float *n, float max_, float min_){
  if(*n < min_){
  *n = min_;
  }
if(*n > max_){
  *n = max_;
  }
}

void clampInt(int *n, int max_, int min_){
  if(*n < min_){
  *n = min_;
  }
if(*n > max_){
  *n = max_;
  }
}

void setup() {
  // put your setup code here, to run once:

  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial.println("start");
/*
  Serial.println(EEPROM.read(10));
  Serial.println(EEPROM.read(11));
  Serial.println(EEPROM.read(12));
  Serial.println(EEPROM.read(13));
  Serial.println(EEPROM.read(14));
  Serial.println(EEPROM.read(15));
  Serial.println(EEPROM.read(16));
  Serial.println(EEPROM.read(17));
  Serial.println(EEPROM.read(18));
*/
  
  Serial.println(EEPROM.read(0));
  Serial.println(EEPROM.read(1));
  Serial.println(EEPROM.read(2));
  Serial.println(EEPROM.read(3));
  Serial.println(EEPROM.read(4));
  
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
  delay(100);
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  delay(100);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  delay(100);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  delay(100);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  delay(100);
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  delay(100);
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  delay(100);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  delay(100);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  delay(100);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  delay(100);
/*
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
  delay(10);

  //百回読み込んでも静止状態と判断できなければ諦める。
  if(counter > 100){
    Serial.println("Can not confirm the stationary state.");
    while(true);
  }
  }  //三軸の加速度を全て足して約1Gならば静止状態と言える。
  while(abs(1 - (ax + ay + az)) > 0.3);

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
  
  int x = 0, y = 0, z = 0;
  if(offsetX < 0){
    x = 1;
  }
  if(offsetY < 0){
    y = 1;
  }
  if(offsetZ < 0){
    z = 1;
  }
  
  EEPROM.write(10, (int)(abs(offsetX)*100));
  EEPROM.write(11, (int)(abs(offsetX)*10000 - (((int)(abs(offsetX) * 100)) * 100)));
  
  EEPROM.write(12, (int)(abs(offsetY)*100));
  EEPROM.write(13, (int)(abs(offsetY)*10000 - (((int)(abs(offsetY) * 100)) * 100)));
  
  EEPROM.write(14, (int)(abs(offsetZ)*100));
  EEPROM.write(15, (int)(abs(offsetY)*10000 - (((int)(abs(offsetY) * 100)) * 100)));
  EEPROM.write(16, x);
  EEPROM.write(17, y);
  EEPROM.write(18, z);
*/
  Serial.println("Launch program...");
  

  delay(1000);

  esc1.attach(3);
  esc2.attach(6);
  esc3.attach(10);
  esc4.attach(9);
  esc1.writeMicroseconds(0);
  esc2.writeMicroseconds(0);
  esc3.writeMicroseconds(0);
  esc4.writeMicroseconds(0);
  
}
//1610～
int availa;
void loop() {

  culcTimer++;
  if(culcTimer > 20){
    STATE = CULC_MODE;
    culcRotation();
    culcTimer = 0;

  if(leftHatY > 600){
    culcPID(escError_roll, angleY, goalAngleY, errorSum_roll, preError_roll, Kp_roll, Ki_roll, Kd_roll);  
  }
  else{
    escError_roll = 0;
  }
  }  


  clamp(&leftHatY, 2000, 550);
  esc1Power = leftHatY - escError_roll;
  esc2Power = leftHatY + escError_roll;
  esc3Power = leftHatY - escError_roll;
  esc4Power = leftHatY + escError_roll;


  if(angleY < 0){
    //esc1Power = 600;
    //esc3Power = 600;
    digitalWrite(13, HIGH);
  }
  else{
    digitalWrite(13, LOW);
  }
  if(angleY > 0){
    //esc2Power = 600;
    //esc4Power = 600;
  }
  
  
  esc1.writeMicroseconds(esc1Power);
  esc2.writeMicroseconds(esc2Power);
  esc3.writeMicroseconds(esc3Power);
  esc4.writeMicroseconds(esc4Power);

  if(STATE == INPUT_MODE){

    availa = Serial.available();
    if(Serial.available() >= sizeof('y') + sizeof(unsigned char)){
    recv_charData = Serial.peek();
    if(recv_charData == 'X' || recv_charData == 'Y' || recv_charData == 'x' || recv_charData == 'y'){
      Serial.read();
      Serial.print(recv_charData);
      Serial.print("\t");
    
    // ヘッダの確認
    if(recv_charData == 'X'){
      recv_data = Serial.read();
      Serial.println(recv_data);
    }
    if(recv_charData == 'Y'){
      recv_data = Serial.read();
      Serial.println(recv_data);
    }
    if(recv_charData == 'x'){
      recv_data = Serial.read();
      Serial.println(recv_data);
    }
    if (recv_charData  == 'y' ) {
        recv_data = Serial.read();
        Serial.println(recv_data);

      if(recv_data >= 0 && recv_data <= 255){
        leftHatY += ((float)recv_data - 127) * -0.01;
      }
    }
    }
    }
    if(Serial.available() >= 8){
    if(checkBuffer(cross)){
      Serial.println("STOP!");
      EEPROM.write(2, Kp_roll * 10);
      EEPROM.write(3, Ki_roll * 1000);
      EEPROM.write(4, Kd_roll * 10);
      esc1.writeMicroseconds(0);
      esc2.writeMicroseconds(0);
      esc3.writeMicroseconds(0);
      esc4.writeMicroseconds(0);
      while(true);
    }
    else if(checkBuffer(triangle)){
      Kd_roll += 0.4;
      
    }
    else if(checkBuffer(square1)){
      Kd_roll -= 0.4;
      if(Kd_roll < 0){
        Kd_roll = 0;
      }
    }
    else if(checkBuffer(up)){
      Kp_roll += 0.2;
      Serial.println("UP!");
    }
    else if(checkBuffer(down)){
      Serial.println("DOWN!");
      Kp_roll -= 0.2;
      if(Kp_roll < 0){
        Kp_roll = 0;
      }
      }
    else if(checkBuffer(right)){
      Serial.println("RIGHT!");
      Ki_roll += 0.001;
    }
    else if(checkBuffer(left)){
      Serial.println("LEFT!");
      Ki_roll -= 0.001;
     if(Ki_roll < 0){
        Ki_roll = 0;
     }
    }
  }
  }
    else{
      STATE = INPUT_MODE;
    }
    while(Serial.available() > 40){
      Serial.read();
    }


}
float oneg = 0;
float pre_acc_x, pre_acc_y, pre_acc_z;
void culcRotation(){
  
      //加速度を読み込み
    raw_acc_x = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
    raw_acc_y = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
    raw_acc_z = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
    //単位Gへ変換
    acc_x = (raw_acc_x / 16384.0);
    acc_y = (raw_acc_y / 16384.0);
    acc_z = (raw_acc_z / 16384.0);
    acc_x = 0.8 * acc_x + 0.2 * pre_acc_x;
    acc_y = 0.8 * acc_y + 0.2 * pre_acc_y;
    acc_z = 0.8 * acc_z + 0.2 * pre_acc_z;
    //加速度センサーから角度を算出
    acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
    acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;
  
    
    // ジャイロで角速度を求める。
    gx = (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
    gy = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    gz = (readMPU6050(MPU6050_GYRO_ZOUT_H) << 8) | readMPU6050(MPU6050_GYRO_ZOUT_L);
    dpsX = gx / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
    dpsY = gy / 65.5;
    dpsZ = gz / 65.5;
    interval = millis() - preInterval;
    gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
    gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
    gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);
    preInterval = millis();

    if(acc_y == 0 && acc_z == 0){
      oneg = acc_x;
    }
    // Z = 0.81  Y = 0.95 X = 1.03

    float outPower = abs(acc_x) + abs(acc_y) + abs(acc_z);
/*
    if(acc_angle_x < 15){
      angleX = (0.65 * gyro_angle_x) + (0.35 * acc_angle_x);
      gyro_angle_x = angleX;
    }
    else{
      angleX = gyro_angle_x;
    }
    if(acc_angle_y < 15){
      angleY = (0.65 * gyro_angle_y) + (0.35 * acc_angle_y);
      gyro_angle_y = angleY;
    }
    else{
      angleY = gyro_angle_y;
    }
    */
    angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
    angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
    angleZ = gyro_angle_z;
    gyro_angle_x = angleX;
    gyro_angle_y = angleY;
    gyro_angle_z = angleZ;
  
    
/*
    if(outPower < 1.5){
      angleX = (0.65 * gyro_angle_x) + (0.35 * acc_angle_x);
      angleY = (0.65 * gyro_angle_y) + (0.35 * acc_angle_y);
      angleZ = gyro_angle_z;
      gyro_angle_x = angleX;
      gyro_angle_y = angleY;
    }
    else{
      angleX = gyro_angle_x;
      angleY = gyro_angle_y;
      angleZ = gyro_angle_z;
    }
    */
    
 /*
    Serial.print(gyro_angle_x);
    Serial.print("\t");
    Serial.print(gyro_angle_y);
    Serial.print("\t");
    Serial.print(gyro_angle_z);

    Serial.print("\t");
    Serial.print(acc_angle_x);
    Serial.print("\t");
    Serial.print(acc_angle_y); 
    */
    /*
    Serial.print("\t");
    Serial.print(acc_x);
    Serial.print("\t");
    Serial.print(acc_y);
    Serial.print("\t");
    Serial.print(acc_z);
*/


  Serial.print(errorSum_roll);
  Serial.print("\t");
  Serial.print(angleY - preError_roll);
  Serial.print("\t");
  Serial.print(esc1Power);
  Serial.print("\t");
  Serial.print(esc2Power);
  Serial.print("\t");
  Serial.print(esc3Power);
  Serial.print("\t");
  Serial.print(esc4Power);
  Serial.print("\t");
    Serial.print("\t");
    Serial.print(angleX);
    Serial.print("\t");
    Serial.print(angleY);
    Serial.print("\t");
    Serial.println(angleZ);  
  
 
  /*
    if(outPower  >= 1.5){
      Serial.print("\t");
      Serial.print("a;sdfjk;ajsdfj;kasdjfa;sdjf;jasdk;jf");
    }
 
    Serial.print("\t");
    Serial.print(outPower);
*/

}

/*PID計算関数。引数は　変える値、現在の値、目標値、対応する偏差の合計が入った変数、
 * 対応する前回の偏差が入った変数、Kp、Ki、Kdの順番
 */
void culcPID(float &target, float now, float goal, float &errorSum, float &preError, float Kp, float Ki, float Kd){
  float P, I, D;
  float error = now - goal;

  errorSum += error * Ki;

  if(errorSum > 300){
    errorSum = 300;
  }
  if(errorSum < -300){
    errorSum = -300;
  }
  
  P = error * Kp;
  I = errorSum;
  D = (error - preError) * Kd;
  
  target = P + I + D;
  preError = error;

  if(target > 400){
    target = 400;
  }
  if(target < -400){
    target = -400;
  }
  
}

bool checkBuffer(char str[8]){
  if(Serial.peek() == str[0]){
    Serial.read();
    if(Serial.peek() == str[1]){
      Serial.read();
      if(Serial.peek() == str[2]){
      Serial.read();
        if(Serial.peek() == str[3]){
        Serial.read();
          if(Serial.peek() == str[4]){
          Serial.read();
            if(Serial.peek() == str[5]){
            Serial.read();
              if(Serial.peek() == str[6]){
              Serial.read();
                if(Serial.peek() == str[7]){
                Serial.read();
                return true;
              }
            }
          }
        }
      }
    }
    } 
  }
  return false;
  
}


