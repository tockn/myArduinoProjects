#include "BMP180_tockn.h"

void BMP180::update() {
  raw_t = 0;
  raw_p = 0;
  for (int i = 0; i < filterTimes; i++) {
    raw_p += bmp.readPressure();
    raw_t += bmp.readTemperature();
  }
  raw_p /= filterTimes;
  raw_t /= filterTimes;
  temp = raw_t;
  pres = raw_p / 100;
  alt = 44330.0 * (1.0 - pow((pres / P0), (1.0 / 5.255))) - offsetAlt;
}
