#include<Servo.h>

Servo esc;

int recv_data; // 受信データ

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  esc.attach(9);
  esc.writeMicroseconds(0);
}
double leftHatY = 1000;
void loop() { 
/*
  // 受信バッファに３バイト（ヘッダ＋int）以上のデータが着ているか確認
  if ( Serial.available() >= sizeof('x') + sizeof(int) ) {
    // ヘッダの確認
    if ( Serial.read() == 'x' ) {
      int low = Serial.read(); // 下位バイトの読み取り
      int high = Serial.read(); // 上位バイトの読み取り
      recv_data = makeWord(high,low); // 上位バイトと下位バイトを合体させてint型データを復元
      Serial.print("LeftHatX : ");
      Serial.println(recv_data);
      
    }
  }
  */
  if ( Serial.available() >= sizeof('y') + sizeof(int) ) {
    // ヘッダの確認
    if ( Serial.read() == 'y' ) {
      int low = Serial.read(); // 下位バイトの読み取り
      int high = Serial.read(); // 上位バイトの読み取り
      recv_data = makeWord(high,low); // 上位バイトと下位バイトを合体させてint型データを復元
      if(recv_data >= 0 && recv_data <= 255){
        Serial.print("LeftHatY : ");
        Serial.println((recv_data - 127) * -1);
        leftHatY += (recv_data - 127) * -0.003;
        if(leftHatY < 1000){
          leftHatY = 1000;
        }
        if(leftHatY > 1800){
          leftHatY = 1800;
        }
        Serial.println(leftHatY);
  
      }
    }
  }
  else{
  Serial.print("LeftHatY : ");
  Serial.println(0);
  }

  esc.writeMicroseconds(leftHatY);

}
