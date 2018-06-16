#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <string>

int logNum = 0;
char numStr[100];
char logNumDir[100] = "/";
char directory[100] ="/flight";
char dummy[2048];
char GPSDir[100] = "/GPS";
char RPYDir[100] = "/RPY";

char GPSdata[2048];
int g = 0;
char RPYdata[2048], androSendData[1024];
char sepa[500];
int d = 0, androCount = 0;
bool errorLED = false;
bool written = false;
long LEDtimer = 0;

bool GPSWrite = false, RPYWrite = false;

HardwareSerial BLESerial(1), ANDROSerial(2); //1--Rx33 Tx32  2--Rx17 Tx16

int MODE = 0;
const int GPS = 0, RPY = 1;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void readLogNum(fs::FS &fs, const char * path, char* logNum, int len);
bool writeFile(fs::FS &fs, const char * path, const char * message);
bool appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);

void WriteSD(void *pvParameters){
  while(1){

    /*Bluetoothからのデータ読み込み。
   * ％を読んだらGPSのデータが送られてくるので、GPS.TXTへ書き込む。。
   * ＆を読んだらそれ以外のデータが送られてくるので、RPY.TXTへ書き込む。
   * RPY.TXTは、
   * 時間、ロール、ピッチ、ヨー、温度、気圧、気圧からの高度、超音波からの高度、ｘ加速度、ｙ加速度、ｚ加速度、ｘ角速度、ｙ角速度、ｚ角速度
   * のフォーマットになっている。
    */
    while(BLESerial.available() > 0){
      char recv_data = (char)BLESerial.read();
      
      if(recv_data == '&')  MODE = RPY;
      else if(recv_data == '%')  MODE = GPS;
      else if(MODE == GPS){
        GPSdata[g] = recv_data;
        g++;
        if(g >= 200){
          GPSWrite = true;
        }
      }
      else if(MODE == RPY){
        ANDROSerial.print(recv_data);
        RPYdata[d] = recv_data;
        d++;
        if(d >= 200){
          RPYWrite = true;
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
      while(0){
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

  listDir(SD,"/",0);
  
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

  if(errorLED){
    digitalWrite(2, HIGH);
  }
  else{
    digitalWrite(2, LOW);
  }

  if(LEDtimer > millis() || LEDtimer + 200 < millis()){
    digitalWrite(4, HIGH);
  }else{
    digitalWrite(4, LOW);
  }

  
/*
  if(written && LEDtimer < millis()){
    LEDtimer = millis() + 200;
  }

  if(LEDtimer > millis()){
    digitalWrite(4, HIGH);
  }else{
   digitalWrite(4, LOW); 
  }

  if(!written){
    digitalWrite(4, HIGH);
  }
  */

      
    if(GPSWrite){
      appendFile(SD, GPSDir, GPSdata);
      g = 0;
      GPSWrite = false;
    }
    if(RPYWrite){
      appendFile(SD, RPYDir, RPYdata);
      d = 0;
      RPYWrite = false;
    }

  written = false;
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
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

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
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
      if(LEDtimer < millis() - 25){
        written = true;
        LEDtimer = millis() + 25;
      }
      return true;
     //   Serial.print("Message appended");
    } else {
      errorLED = true;
      written = false;
      Serial.printf("Appending to file: %s\n", path);
      Serial.println("Append failed");
      return false;
    }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}
