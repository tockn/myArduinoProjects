#ifndef BMP180_TOCKN_H
#define BMP180_TOCKN_H

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BMP085_tockn.h"

class BMP180 {
public:
	BMP180(float P0 = 1006.3, float offsetAlt = 0);

	void begin(int SDA, int SCL);

	void update();

	float getPressure() { return pressure; }
	float getTemperature() { return temperature; }
	float getAltitude() { return altitude; }

private:
	TwoWire *wire;
	float P0;
	float raw_p, raw_t;
	float temperature;
	float pressure;
	float altitude;
	float offsetAlt;
	Adafruit_BMP085 bmp;
}

#endif
