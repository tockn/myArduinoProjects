#include <SoftwareSerial.h>

SoftwareSerial logger(10, 11);

char recv;

char r, p, y;

void setup() {
  logger.begin(9600);
  Serial.begin(9600);
  Serial.println("launch");
}

void loop() {
  while (logger.available() >= 12) {
    if(header('r')) r = logger.read();
    else  continue;
    if(header('p')) p = logger.read();
    else  continue;
    if(header('y')) y = logger.read();
    else  continue;
    Serial.print((int)r); Serial.print("\t");
    Serial.print((int)p); Serial.print("\t");
    Serial.print((int)y); Serial.println();
  }
} 

bool header(char c) {
  char cc;
  cc = (char)logger.read();
  if (cc == c) {
    cc = (char)logger.read();
    if (cc == c) {
      cc = (char)logger.read();
      if (cc == c) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
  return false;
}

