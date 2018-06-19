#ifndef _BMP180_TOCKN_
#define _BMP180_TOCKN_

#include "Adafruit_BMP085.h"

class BMP180 {
  private:
    float raw_p;
    float raw_t;
    float P0 = 1006.3;
    float alt;
    float pres;
    float temp;
    float offsetAlt;
    int filterTimes = 1;
    Adafruit_BMP085 bmp;
  public:
    void update();
    bool begin(int SDA, int SCL) { return bmp.begin(SDA, SCL); }
    void setFilterTimes(int t) { filterTimes = t; };
    void setOffsetAlt(float a) { offsetAlt = a; }
    float getAltitude() { return alt; }
    float getPressure() { return pres; }
    float getTemperature() { return temp; }
};

#endif
