#include "BMP180_tockn.h"

BMP180 bmp(1006.3, 0);

void setup() {
  Serial.begin(9600);
  bmp.begin(26, 25);
}

void loop() {
  bmp.update();
  Serial.println(bmp.getAltitude());
}


