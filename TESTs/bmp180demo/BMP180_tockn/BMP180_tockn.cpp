#include "BMP180_tockn.h"
#include "Arduino.h"

BMP180::BMP180() {
	this->P0 = 1006.3;
}

BMP180::BMP180(float P0) {
	this->P0 = P0;
}

void BMP180::begin(int SDA, int SCL) {
	bmp.begin(SDA, SCL);
}

void BMP180::update() {
	raw_p = bmp.readPressure();
	raw_t = bmp.readTemperature();

	temperature = raw_t;
	pressure = raw_p / 100;
	altitude = 44330.0 * (1.0 - pow((pressure / P0), (1.0 / 5.255))) - offsetAlt;
}
