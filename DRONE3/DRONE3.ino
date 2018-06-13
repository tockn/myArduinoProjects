#include<Wire.h>
#include<EEPROM.h>

//レジスタアドレス
#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
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
double offsetX = -0.4937493324, offsetY = -1.3529599905, offsetZ = -0.0149898648;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval;
unsigned long long preInterval;

int low, high;

float raw_acc_x, raw_acc_y, raw_acc_z, acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;
float calibraX = 0, calibraY = 0;

Servo esc1, esc2, esc3, esc4;

unsigned char recv_data = 0;
char recv_charData;
float leftHatY = 0;
float  esc1Power = 0, esc2Power = 0, esc3Power = 0, esc4Power = 0;
float escError_roll = 0;
float escError_pitch = 0;
float escError_yaw = 0;

unsigned long loopTime;

float calcTimer = 0;
void calcRotation(void);
//1.6   2.1

float Kp_roll = 0.31/*0.174*/, Ki_roll = 0 /*0.000106*/, Kd_roll = 0.845;/*0.945/*40*/
float Kp_pitch = 0.24/*0.46*/, Ki_pitch = 0/*0.0076*/, Kd_pitch = 0.845;/*0.945/*35*/
float Kp_yaw = 0.3, Ki_yaw = 0.001, Kd_yaw = 0;

float rate_Kp_roll = 0, rate_Kp_pitch = 0, rate_Kp_yaw = 0;
float setRateX = 0, setRateY = 0, setRateZ = 0;

float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

float errorSum_roll = 0;
float errorSum_pitch = 0;
float errorSum_yaw = 0;
float preError_roll = 0;
float preError_pitch = 0;
float preError_yaw = 0;

float setPointX = 0, setPointY = 0, setPointZ = 0;

float btnValueLeft = 0, btnValueRight = 0;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, counter_channel_5, counter_channel_6, counter_channel_7, loop_counter;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, timer_channel_6, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;
unsigned long loop_timer;

float input_gyro_roll, input_gyro_pitch, input_gyro_yaw;

void calcPID(float &target, float now, float setPoint, float &errorSum, float &preError, float Kp, float Ki, float Kd);

float angleX_correction, angleY_correction;

int aileron, elevator, throttle, rudder, btnRight, btnLeft;

float offsetAngleX = 9, offsetAngleY = -4.2;


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
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/, false);
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}
/*
  //MsTimerを使いこの関数をdt秒ごとに呼び出す！
  void update_(){
  dataUpdate = true;
  }
*/
// 0.44   0.21

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
  Serial.println("start");

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
  angleY = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  angleX = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  gyro_angle_x = angleX;
  gyro_angle_y = angleY;

  DDRD |= B00111100;
  long pre = millis();
  while(1){
    PORTD |= B00111100;
    delayMicroseconds(1000);
    PORTD &= B00000000;
    delay(3);
    if(millis() - pre > 3000) break;
  }

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);

  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT23);

}

//1610～
int availa;
int num = 1;
bool store = true;
float diffSensor = 0;
float cycle = 0;
void loop() {

  calcRotation();

  input_gyro_roll = input_gyro_roll * 0.7 + (dpsY - offsetY) * 0.3;
  input_gyro_pitch = input_gyro_pitch * 0.7 + (dpsX - offsetX) * 0.3;
  input_gyro_yaw = input_gyro_yaw * 0.7 + (dpsZ - offsetZ) * 0.3;

  angleX_correction = angleX * 15;
  angleY_correction = angleY * 15;

  setPointX = 0;
  if(elevator > 1510) setPointX = (long)elevator - 1510;
  else if(elevator < 1490) setPointX = (long)elevator - 1490;
  setPointX -= angleX_correction;
  setPointX /= 3;

  setPointY = 0;
  if(aileron > 1510) setPointY = (long)aileron - 1510;
  else if(aileron < 1490) setPointY = (long)aileron - 1490;
  setPointY -= angleY_correction;
  setPointY /= 3;

  setPointZ = 0;
  if(rudder > 1510) setPointZ = (long)rudder - 1510;
  else if(rudder < 1490) setPointZ = (long)rudder - 1490;
  setPointZ /= 3;

  btnValueLeft = map(btnLeft, 1082, 1926, 0, 5000);
  btnValueLeft /= 10000;
  Kp_roll = btnValueLeft;

  btnValueRight = map(btnRight, 1100, 1926, 0, 5000);
  if (btnValueRight < 0) btnValueRight = 0;
  btnValueRight /= 10000;
  Kp_pitch = btnValueRight;

  if (Kp_roll < 0) Kp_roll = 0;
  if (Kd_roll < 0) Kd_roll = 0;
  if (Kp_pitch < 0) Kp_pitch = 0;
  if (Kd_pitch < 0) Kd_pitch = 0;
  if (Kp_roll < 0) Kp_roll = 0;

  if (throttle > 1150) {
    calcPID(escError_roll, input_gyro_roll, setPointY, errorSum_roll, preError_roll, Kp_roll, Ki_roll, Kd_roll);
    calcPID(escError_pitch, input_gyro_pitch, setPointX, errorSum_pitch, preError_pitch, Kp_pitch, Ki_pitch, Kd_pitch);
    calcPID(escError_yaw, input_gyro_yaw, setPointZ, errorSum_yaw, preError_yaw, Kp_yaw, Ki_yaw, Kd_yaw);
   
    esc1Power = throttle + escError_roll + (escError_pitch) - escError_yaw;
    esc2Power = throttle - escError_roll + (escError_pitch) + escError_yaw;
    esc3Power = throttle + escError_roll - escError_pitch + escError_yaw;
    esc4Power = throttle - escError_roll - escError_pitch - escError_yaw;
  }
  else {
    esc1Power = 1000;
    esc2Power = 1000;
    esc3Power = 1000;
    esc4Power = 1000;

    angleX = acc_angle_x;
    angleY = acc_angle_y;

    gyro_angle_x = acc_angle_x;
    gyro_angle_y = acc_angle_y;
    
    errorSum_roll = 0;
    errorSum_pitch = 0;
    errorSum_yaw = 0;
    preError_roll = 0;
    preError_pitch = 0;
    preError_yaw = 0;
    angleZ = 0;
    gyro_angle_z = 0;
  }

  while (micros() - loopTime < 4000);
  loopTime = micros();
  PORTD |= B00111100;

  while(PORTD != 0){
    if(micros() - loopTime > esc1Power){
      PORTD &= B00111000;
    }
    if(micros() - loopTime > esc2Power){
      PORTD &= B00110100;
    }
    if(micros() - loopTime > esc3Power){
      PORTD &= B00101100;
    }
    if(micros() - loopTime > esc4Power){
      PORTD &= B00011100;
    }
  }

}

float oneg = 0;
float pre_acc_x, pre_acc_y, pre_acc_z;
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

  acc_angle_y += offsetAngleY;
  acc_angle_x += offsetAngleX;

  dpsX = ((float)raw_gyro_x) / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
  dpsY = ((float)raw_gyro_y) / 65.5;
  dpsZ = ((float)raw_gyro_z) / 65.5;
  interval = micros() - preInterval;
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.000001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.000001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.000001);
  preInterval = micros();

  angleX = (0.9995 * gyro_angle_x) + (0.0005 * acc_angle_x);
  angleY = (0.9995 * gyro_angle_y) + (0.0005 * acc_angle_y);
  angleZ = gyro_angle_z;
  
  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;

}

/*PID計算関数。引数は　変える値、現在の値、目標値、対応する偏差の合計が入った変数、
   対応する前回の偏差が入った変数、Kp、Ki、Kdの順番
*/
void calcPID(float &target, float now, float setPoint, float &errorSum, float &preError, float Kp, float Ki, float Kd) {
  float P, I, D, diff, interval;
  float error = now - setPoint;

  errorSum += error * Ki;
  diff = error - preError;

  P = error * Kp;
  I = errorSum;
  D = diff * Kd;

  target = P + I + D;
  preError = error;

  if (target > 500) {
    target = 500;
  }
  if (target < -500) {
    target = -500;
  }
}

//プロポからの入力値を読み込む
ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001) {                                                   //Is input 8 high?
    if (last_channel_1 == 0) {                                              //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1) {                                           //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    aileron = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if (PINB & B00000010 ) {                                                  //Is input 9 high?
    if (last_channel_2 == 0) {                                              //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1) {                                           //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    elevator = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if (PINB & B00000100 ) {                                                  //Is input 10 high?
    if (last_channel_3 == 0) {                                              //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1) {                                           //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    throttle = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if (PINB & B00001000 ) {                                                  //Is input 11 high?
    if (last_channel_4 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1) {                                           //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    rudder = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  if (PINB & B00010000 ) {                                                  //Is input 11 high?
    if (last_channel_5 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_5 == 1) {
    last_channel_5 = 0;
    btnRight = current_time - timer_5;
  }
}

ISR(PCINT2_vect) {
  current_time = micros();
  if (PIND & B10000000) {
    if (last_channel_6 == 0) {
      last_channel_6 = 1;
      timer_6 = current_time;
    }
  }
  else if (last_channel_6 == 1) {
    last_channel_6 = 0;
    btnLeft = current_time - timer_6;
  }
}


