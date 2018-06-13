#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <U8glib.h>
#include <Servo.h>

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


//======================GLOBAL=========================

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST);  // Dev 0, Fast I2C / TWI

int STATE = 0, preSTATE = 0, BARO = 0, DEG = 1;
int culcSTATE = 0;

double offsetX = -1.7349021434, offsetY = 0.1476538085, offsetZ = 0.21899728;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float raw_acc_x, raw_acc_y, raw_acc_z, acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

float alt, hPa, TEMP;
float P0 = 1016.1;
float raw_p = 0;
float raw_t = 0;

float offsetAlt = 0;

int btn = 1;

Adafruit_BMP085 bmp;
void culcRotation(void);
void culcAltitude(void);
void firstCulcRotation(void);

Servo servo;

//====================================================

void draw(float num, int width, int height) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(width, height); 
  // call procedure from base class, http://arduino.cc/en/Serial/Print
  u8g.print(num);
}

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

  
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  while (1) {}
  }

  //加速度を読み込み
  raw_acc_x = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
  raw_acc_y = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
  raw_acc_z = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
  //単位Gへ変換
  acc_x = (raw_acc_x / 16384.0);
  acc_y = (raw_acc_y / 16384.0);
  acc_z = (raw_acc_z / 16384.0);

  //加速度センサーから角度を算出
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  gyro_angle_x = acc_angle_x;
  gyro_angle_y = acc_angle_y;
  angleX = acc_angle_x;
  angleY = acc_angle_y;

  culcAltitude();
  offsetAlt = alt;
  servo.attach(3);
}



void loop() {

  servo.write(180 - (16 * abs(alt)));
  
  btn = digitalRead(12);
  if(btn == 0){
    delay(400);
    if(STATE == BARO){
      STATE = DEG;
      firstCulcRotation();
    }
    else{
      STATE = BARO;
    }
  }
  
  if(STATE == BARO){
    digitalWrite(13, HIGH);
    culcAltitude();
    u8g.firstPage();  
    do {
      draw(TEMP, 0, 20);
      u8g.drawStr(110, 20, "C");
      draw(hPa, 0, 40);
      u8g.drawStr(100, 40, "hPa");
      draw(alt, 0, 60);
      u8g.drawStr(110, 60, "m");
    } while( u8g.nextPage() );
    
  }
  else if(STATE == DEG){
    digitalWrite(13, LOW);
    culcRotation();
    u8g.firstPage();
    do {
      u8g.drawStr(0, 20, "Roll  :");
      draw(angleX, 70, 20);
      u8g.drawStr(0, 40, "Pitch :");
      draw(angleY, 70, 40);
      u8g.drawStr(0, 60, "Yaw   :");
      draw(angleZ, 70, 60);
    } while( u8g.nextPage() );
  }

  raw_p = 0;
  raw_t = 0;

}

void culcAltitude(){

  for(int i = 0; i < 3; i++){
    raw_p += bmp.readPressure();
    raw_t += bmp.readTemperature();
  }
  raw_p /= 3;
  raw_t /= 3;
  
  TEMP = raw_t;
  hPa = raw_p / 100;
  alt = 44330 * (1 - pow((hPa/P0), (1/5.255))) - offsetAlt;
  
}

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
    pre_acc_x = acc_x;
    pre_acc_y = acc_y;
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

    angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
    angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
    angleZ = gyro_angle_z;
    gyro_angle_x = angleX;
    gyro_angle_y = angleY;
    gyro_angle_z = angleZ;

    Serial.print(acc_angle_x);
    Serial.print("\t");
    Serial.println(acc_angle_y);
    
}

void firstCulcRotation(){
  
    //加速度を読み込み
    raw_acc_x = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
    raw_acc_y = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
    raw_acc_z = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
    //単位Gへ変換
    acc_x = (raw_acc_x / 16384.0);
    acc_y = (raw_acc_y / 16384.0);
    acc_z = (raw_acc_z / 16384.0);
    
    //加速度センサーから角度を算出
    acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
    acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;
    
    angleX = acc_angle_x;
    angleY = acc_angle_y;
    Serial.println(angleX);
    Serial.println(angleY);
    gyro_angle_x = angleX;
    gyro_angle_y = angleY;
    preInterval = millis();
}

