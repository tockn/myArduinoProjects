#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

// GPS制御端子
const int gpsRxPin = 4;
const int gpsTxPin = 2;
const int gpsPowerPin = 8;

// LED端子
const int ledRedPin = 5;
const int ledGrnPin = 6;

File gp;

// GPS通信
SoftwareSerial gpsPort(gpsRxPin, gpsTxPin);

// GPS受信データ
uint8_t fetchDataLength;
char fetchData[200];

void setup()
{

  pinMode(ledRedPin, OUTPUT);
  pinMode(ledGrnPin, OUTPUT);

  pinMode(gpsPowerPin, OUTPUT);
  digitalWrite(gpsPowerPin, HIGH);

  // ソフトウェアシリアルを初期化
  gpsPort.begin(9600);
  Serial.begin(9600);
  Serial.println("start");

  SD.begin();
  
}

void loop()
{
  // GPSデータ受信
  if(gps_fetch()){
    digitalWrite(ledGrnPin, HIGH);
    // GPSデータ書き込み
    gp = SD.open("gpstest.txt",FILE_WRITE);
    gp.print(fetchData);
    gp.close();
    Serial.print(fetchData);
    digitalWrite(ledGrnPin, LOW);
  }
}

bool gps_fetch(void)
{
  if( gpsPort.available() ){
    unsigned long last_time = millis();
    uint8_t preamble = 0;
    uint8_t len = 0;
    fetchDataLength = 0;
    do {
      while( gpsPort.available() ){
        char c = gpsPort.read();
        if( c == '$' ){
          preamble++;
        } 
        if( preamble ){
          if( len < sizeof(fetchData) ){
            fetchData[len++] = c;
          }
          if( c == '\n'){
            fetchDataLength = len;
          }
        }
        last_time = millis();
      }
      delay(1);
    }
    while( millis() - last_time < 10 );
    return fetchDataLength > 0;
  }
  return false;
}

