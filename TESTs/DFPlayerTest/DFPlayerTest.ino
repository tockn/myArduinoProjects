#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>
 
SoftwareSerial mySerial(2, 3); // RX, TX
 
//
void setup () {
  Serial.begin (9600);
  mySerial.begin (9600);
  mp3_set_serial (mySerial);  //set softwareSerial for DFPlayer-mini mp3 module 
  mp3_set_volume (20);
}
 
//
void loop () {        
  mp3_play (1);
  Serial.println("play");
  delay(6000);
  
}
