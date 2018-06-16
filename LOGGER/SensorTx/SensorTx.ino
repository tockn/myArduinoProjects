#include <Wire.h>
//#include <Adafruit_BMP085.h>

#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42


//============================global===============================================
//-2.1153638363  0.2630327940  0.4755737185
double offsetX = -1.8400697708, offsetY = 0.4478996992, offsetZ = 0.5767665505
;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angle_roll, angle_pitch, angle_yaw;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

float start_yaw = 0;

float presAlt, hPa, TEMP;
float P0 = 1006.3;
float raw_p = 0;
float raw_t = 0;

int changedFormatPresAlt, sendAlt;

int sonicAlt = 0, preSonicTimer = 0;

char sendData[10];

float offsetAlt = 0;

long offsetTime = 0;

char data[1024], dummy[1024];

bool LED = false;

//Adafruit_BMP085 bmp;

void calcRotation();
//void calcAltitude(void);
void sendIntData(int value);

HardwareSerial GPSerial(2)/*Rx-16 Tx-17*/, BLESerial(1)/* Rx-32 Tx-33*/, DispSerial(4);

//===============================================================================

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

void SonicPulseIn(void *pvParameters){
  while(1){
  sonicAlt = pulseIn(34, HIGH);
  }
}

void setup() {

  xTaskCreatePinnedToCore(SonicPulseIn,"SonicPulseIn", 4096, NULL, 1, NULL, 1);

  Serial.begin(9600);
  GPSerial.begin(115200);
  BLESerial.begin(115200);
  DispSerial.begin(115200);

  Serial.println("start");
    
  Wire.begin(26, 25);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.println("Wired");
/*
  delay(100);
  //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }

  //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
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
  if (!bmp.begin(26, 25, 0)) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {}
  }
  */
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
  pinMode(34, INPUT);
  pinMode(2, OUTPUT);
}

bool state = true;

int n = 0, i = 0;
long pre = 0;

void loop() {

  digitalWrite(2, HIGH);

  while (GPSerial.available() > 0) {
    BLESerial.write((char)GPSerial.read());
  }
  calcRotation();
  //calcAltitude();

/*
  if(millis() - preSonicTimer > 20){
    sonicAlt = pulseIn(34, HIGH);
    preSonicTimer = millis();
  }
  */
  
  sprintf(data, "t%d,r%.2f,p%.2f,y%.2f,d%.2f,h%.2f,a%.2f,s%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",millis() - offsetTime, angle_roll, angle_pitch, angle_yaw, TEMP, hPa, presAlt, sonicAlt / 147 * 2.54, acc_x, acc_y, acc_z, dpsX, dpsY, dpsZ);
  sprintf(dummy, "%s", data);
  BLESerial.write("&");
  BLESerial.write(data);
  BLESerial.write("%");
  
  sprintf(data, "");

  if(presAlt <= 5.5){
    sendAlt = sonicAlt / 147 * 2.54;
  }else{
    sendAlt = presAlt * 100;
  }

  sprintf(sendData, "%d", sendAlt);

  //Serial.print("aa");
//  Serial.print(sendData);
  sendIntData((short int)sendAlt);
  //Serial.println((short int)sendAlt);

  if(BLESerial.available() > 0){
    char recv_data = BLESerial.read();
    if(recv_data == 'r'){
      Serial.print("rrr");
      start_yaw += angle_yaw;
      offsetAlt += presAlt - 10;
      offsetTime = millis();
    }
  }  
}

void sendIntData(int value) {
  Serial.write(lowByte(value)); // 下位バイトの送信
  Serial.write(highByte(value)); // 上位バイトの送信
}

float pre_acc_x, pre_acc_y, pre_acc_z;
uint8_t buf[14];
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
  
  
    Serial.print(angle_pitch);
    Serial.print("\t");
    Serial.print(angle_roll);
    Serial.print("\t");
    Serial.println(angle_yaw);
    //Serial.print("\t");
    //Serial.println(alt);
  
}
/*
void calcAltitude() {

  for (int i = 0; i < 1; i++) {
    raw_p += bmp.readPressure();
    raw_t += bmp.readTemperature();
  }
  raw_p /= 1;
  raw_t /= 1;

  TEMP = raw_t;
  hPa = raw_p / 100;
  //メートル表記
  presAlt = 44330.0 * (1.0 - pow((hPa / P0), (1.0 / 5.255))) - offsetAlt;
  //超音波センサのフォーマットに合わせる
  changedFormatPresAlt = presAlt / 2.54 * 147 * 100;
  raw_p = 0;
  raw_t = 0;
}
*/
