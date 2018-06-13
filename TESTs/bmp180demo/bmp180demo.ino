#include "BMP180_tockn.h"

BMP180 bmp;

void setup() {
  Serial.begin(9600);
  bmp.begin(26, 25);
}

void loop() {
  bmp.update();
  Serial.println(bmp.getAltitude());
}
