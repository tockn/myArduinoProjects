#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <string>

//SDカードにいくつログフォルダがあるかが格納される変数
int logNum = 0;

//lognum.txtに書き込む数字(文字列)が入る変数
char numStr[100];

//lognum.txtのパスが入る
char logNumDir[100] = "/";

//最初にいろいろと使うパス
char directory[100] ="/flight";

//GPSのデータが入るGPS.txtのパス
char GPSDir[100] = "/GPS";

//GPS以外のセンサーの情報が入るRPY.txtのパス
char RPYDir[100] = "/RPY";

//取得したGPSのデータを格納するバッファ
char GPSdata0[2048];
char GPSdata1[2048];
//配列GPSdataのインデックス
int g0 = 0, g1 = 0;
int GPSdataMode = 0;

//取得したGPS以外のセンサーのデータを格納するバッファ
char RPYdata0[2048];
char RPYdata1[2048];
//配列RPYdataのインデックス
int d0 = 0, d1 = 0;
int RPYdataMode = 0;

//GPSdataに書き込むかRPYdataに書き込むか
int MODE = 0;
const int GPS = 0, RPY = 1;

bool written = false;

//データ分割機能で、Androidから送られてきた分割文字列が入る
char sepa[500];

//記録エラー時に点灯するLEDの点灯させるか否かの変数
bool errorLED = false;

//記録時に点滅させるLEDの間隔
long LEDtimer = 0;

//バッファからかSDカードへ書き込みを行うか否か
bool GPSWrite0 = false, GPSWrite1 = false, RPYWrite0 = false, RPYWrite1 = false;

//Bluetoothモジュールとシリアル通信
HardwareSerial BLESerial(1), ANDROSerial(2); //1--Rx33 Tx32  2--Rx17 Tx16

//SDカード操作関数
void createDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void readLogNum(fs::FS &fs, const char * path, char* logNum, int len);
bool writeFile(fs::FS &fs, const char * path, const char * message);
bool appendFile(fs::FS &fs, const char * path, const char * message);

//デュアルコア！並列実行する関数。
void WriteSD(void *pvParameters){
  while(1){
 
  if(errorLED){
    digitalWrite(2, HIGH);
  }
  else{
    digitalWrite(2, LOW);
  }
/*
  if(LEDtimer > millis() || LEDtimer + 200 < millis()){
    digitalWrite(4, HIGH);
  }else{
    digitalWrite(4, LOW);
  }
    */

  if(written){
    digitalWrite(4, HIGH);
  }else{
    digitalWrite(4, LOW);
  }
  if(GPSWrite0){
    written = true;
    appendFile(SD, GPSDir, GPSdata0);
    g0 = 0;
    GPSWrite0 = false;
  }
  if(GPSWrite1){
    written = true;
    appendFile(SD, GPSDir, GPSdata1);
    g1 = 0;
    GPSWrite1 = false;
  }
  
  if(RPYWrite0){
    written = false;
    appendFile(SD, RPYDir, RPYdata0);
    d0 = 0;
    RPYWrite0 = false;
  }
  if(RPYWrite1){
    written = false;
    appendFile(SD, RPYDir, RPYdata1);
    d1 = 0;
    RPYWrite1 = false;
  }
  }
}

void setup() {

  xTaskCreatePinnedToCore(WriteSD, "WriteSD", 4096, NULL, 1, NULL, 1);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  BLESerial.begin(115200);
  ANDROSerial.begin(115200);

  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  
  
  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      errorLED = true;
      while(1){
        digitalWrite(2,HIGH);
        delay(100);
      }
    }
    
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

  
  readLogNum(SD,"/flight/lognum.txt", numStr, 100);
  logNum = atoi(numStr);
  logNum++;
  sprintf(logNumDir, "%s/lognum.txt", directory);
  sprintf(numStr, "%d", logNum);
  writeFile(SD, logNumDir, numStr);
  sprintf(directory, "%s/%d", directory, logNum);
  createDir(SD, directory);
  /*
  sprintf(GPSDir, "%s/GPS", directory);
  sprintf(RPYDir, "%s/RPY", directory);
  createDir(SD, GPSDir);
  createDir(SD, RPYDir);
  */
  sprintf(GPSDir, "%s/gps.txt", directory);
  sprintf(RPYDir, "%s/rpy.txt", directory);
  while(!writeFile(SD, GPSDir, "\n")){digitalWrite(2,HIGH);}
  while(!writeFile(SD, RPYDir, "\n")){digitalWrite(2,HIGH);}
}

char st[1000];
int s = 0;

void loop() {
  // put your main code here, to run repeatedly:

    /*Bluetoothからのデータ読み込み。
   * ％を読んだらGPSのデータが送られてくるので、GPSdataへ書き込む。。
   * ＆を読んだらそれ以外のデータが送られてくるので、RPYdataへ書き込む。
   * RPY.TXTは、
   * 時間、ロール、ピッチ、ヨー、温度、気圧、気圧からの高度、超音波からの高度、ｘ加速度、ｙ加速度、ｚ加速度、ｘ角速度、ｙ角速度、ｚ角速度
   * のフォーマットになっている。
    */
    while(BLESerial.available() > 0){
      char recv_data = (char)BLESerial.read();
      
      if(recv_data == '&')  MODE = RPY;
      else if(recv_data == '%')  MODE = GPS;
      else if(MODE == GPS){
        if(GPSdataMode == 0){
          GPSdata0[g0] = recv_data;
          g0++;
        }else{
          GPSdata1[g1] = recv_data;
          g1++;
        }
        if(g0 >= 200){
          GPSdataMode = 1;
          GPSWrite0 = true;
        }else if(g1 >= 200){
          GPSdataMode = 0;
          GPSWrite1 = true;
        }
      }
      else if(MODE == RPY){
        ANDROSerial.print(recv_data);
        if(GPSdataMode == 0){
          RPYdata0[d0] = recv_data;
          d0++;          
        }else{
          RPYdata1[d1] = recv_data;
          d1++;
        }
        if(d0 >= 200){
          RPYdataMode = 1;
          RPYWrite0 = true;
        }else if(d1 >= 200){
          RPYdataMode = 0;
          RPYWrite1 = true;
        }
      }
    }

    if(ANDROSerial.available() > 0){
      char recv_data = ANDROSerial.read();
      if(recv_data == 'r')  BLESerial.print(recv_data);
      if(recv_data == 's'){
        if(ANDROSerial.read() == 's' && ANDROSerial.read() == 's'){
          appendFile(SD, GPSDir, "\n\n\n\n////////////////////////////////////////////////\n\n\n\n");
          appendFile(SD, RPYDir, "\n\n\n\n////////////////////////////////////////////////\n\n\n\n");
          char recv[1];
          while(ANDROSerial.available() > 0){
            recv[0] = ANDROSerial.read();
            appendFile(SD, GPSDir, recv);
            appendFile(SD, RPYDir, recv);
          }
          appendFile(SD, GPSDir, "\n\n\n\n////////////////////////////////////////////////\n\n\n\n");
          appendFile(SD, RPYDir, "\n\n\n\n////////////////////////////////////////////////\n\n\n\n");
        }
      }
    }
   

}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }
    Serial.print("Read from file: ");
    while(file.available()){
        Serial.print(file.read());
    }
}

void readLogNum(fs::FS &fs, const char * path, char* logNum, int len){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }
    int i = 0;
    Serial.print("Read from file: ");
    while(file.available()){
        logNum[i] = file.read();
        i++;
        if(i >= len) return;
    }
}

bool writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return false;
    }
    if(file.print(message)){
        Serial.println("File written");
        return true;
    } else {
        Serial.println("Write failed");
        return false;
    }
}

bool appendFile(fs::FS &fs, const char * path, const char * message){
  //  Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
      digitalWrite(2, HIGH);
        Serial.println("Failed to open file for appending");
        return false;
    }
    if(file.print(message)){
      errorLED = false;
      Serial.print("append  :  ");
      Serial.println(path);
      if(LEDtimer < millis() - 25){
        LEDtimer = millis() + 25;
      }
      return true;
     //   Serial.print("Message appended");
    } else {
      errorLED = true;
      Serial.printf("Appending to file: %s\n", path);
      Serial.println("Append failed");
      return false;
    }
}

