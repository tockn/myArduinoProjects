/*
 Example sketch for the RFCOMM/SPP Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or 
 send me an e-mail:  kristianl@tkjelectronics.com
 */




//   x -> rotateX -> y -> rotateY -> z -> rotateZ -> p ->  -> i -> I -

#include <Wire.h>
#include <SPP.h>
#include <usbhub.h>


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

double offsetX = 0, offsetY = 0, offsetZ = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float raw_acc_x, raw_acc_y, raw_acc_z, acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;
float interval, preInterval;





USB Usb;
USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
SPP SerialBT(&Btd); // This will set the name to the defaults: "Arduino" and the pin to "1234"
//SPP SerialBT(&Btd, "Lauszus's Arduino","0000"); // You can also set the name and pin like so

boolean firstMessage = true;


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
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while(1); //halt
  }
  Serial.print(F("\r\nSPP Bluetooth Library Started"));

    
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
  delay(10);

  //百回読み込んでも静止状態と判断できなければ諦める。
  if(counter > 100){
    Serial.println("Can not confirm the stationary state.");
    while(true);
  }
  }  //三軸の加速度を全て足して約1Gならば静止状態と言える。
  while(abs(1 - (ax + ay + az)) > 0.3);

  if(counter <= 100){

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


  
}
}
float k = 0;
int n = 0;
int p = 0;
int s = 0;
void loop() {
  if(s > 100){
  culcRotation();
  s=0;
  }
  s++;
  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well
  
  if(SerialBT.connected) {
    if(firstMessage) {
      firstMessage = false;
      SerialBT.println(F("Hello from Arduino")); // Send welcome message
    }

    n++;
    p++;
    if(p>1000){
      k+=0.1;
      p=0;
    }
    
    if(n>500){
    SerialBT.print('x');
    SerialBT.print(angleX);
    SerialBT.print('y');
    SerialBT.print(angleY);
    SerialBT.print('z');
    SerialBT.print(angleZ);
    SerialBT.print('e');
    Serial.print(angleX);
    Serial.print("\t");
    Serial.print(angleY);
    Serial.print("\t");
    Serial.println(angleZ);
    n=0;
    }


    if(Serial.available())
      SerialBT.write(Serial.read());
    if(SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '1') {
        digitalWrite(13, HIGH);
      } else {
        digitalWrite(13, LOW);
      }
      Serial.write(c);
    }
  } 
  else 
    firstMessage = true;
}






void culcRotation(){
  
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

    
    // Z = 0.81  Y = 0.95 X = 1.03

    float outPower = abs(acc_x) + abs(acc_y) + abs(acc_z);

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
    
    angleZ = gyro_angle_z;    
    
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
    angleX = acc_angle_x;
    angleY = acc_angle_y;
    
    
    //相補フィルターっぽいの
 //   gyro_angle_x = 0.9992 * gyro_angle_x + 0.0008 * acc_angle_y;
   // gyro_angle_y = 0.9992 * gyro_angle_y + -0.0008 * acc_angle_x;
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
/*
    Serial.print("\t");
    Serial.print(angleX);
    Serial.print("\t");
    Serial.print(angleY);
    Serial.print("\t");
    Serial.println(angleZ);  
    */
 
  /*
    if(outPower  >= 1.5){
      Serial.print("\t");
      Serial.print("a;sdfjk;ajsdfj;kasdjfa;sdjf;jasdk;jf");
    }
 
    Serial.print("\t");
    Serial.print(outPower);
*/

}

