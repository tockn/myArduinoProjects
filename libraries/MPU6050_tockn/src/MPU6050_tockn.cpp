#include "MPU6050_tockn.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w){
	wire = &w;
	accCoef = 0.02f;
	gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
	wire = &w;
	accCoef = aC;
	gyroCoef = gC;
}

void MPU6050::begin(){
	writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
	writeMPU6050(MPU6050_CONFIG, 0x00);
	writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
	writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
	writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
	this->update();
	angleGyroX = this->getAccAngleX();
	angleGyroY = this->getAccAngleY();
	preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(reg);
	wire->write(data);
	wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(reg);
	wire->endTransmission(true);
	wire->requestFrom((uint8_t)MPU6050_ADDR, (size_t)2/*length*/);
	byte data =  wire->read();
	wire->read();
	return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

	if(console){
		Serial.println("calculate gyro offsets");
		Serial.print("DO NOT MOVE A MPU6050");
	}
	for(int i = 0; i < 5000; i++){
		if(console && i % 1000 == 0){
			Serial.print(".");
		}
		wire->beginTransmission(MPU6050_ADDR);
		wire->write(0x3B);
		wire->endTransmission(false);
		wire->requestFrom((int)MPU6050_ADDR, 14, (int)true);

		wire->read() << 8 | wire->read();
		wire->read() << 8 | wire->read();
		wire->read() << 8 | wire->read();
		wire->read() << 8 | wire->read();
		rx = wire->read() << 8 | wire->read();
		ry = wire->read() << 8 | wire->read();
		rz = wire->read() << 8 | wire->read();

		x += ((float)rx) / 65.5;
		y += ((float)ry) / 65.5;
		z += ((float)rz) / 65.5;
	}
	gyroXoffset = x / 5000;
	gyroYoffset = y / 5000;
	gyroZoffset = z / 5000;

	if(console){
		Serial.println("Done!!!");
		Serial.print("X : ");Serial.println(gyroXoffset);
		Serial.print("Y : ");Serial.println(gyroYoffset);
		Serial.print("Z : ");Serial.println(gyroYoffset);
		Serial.print("Program will start after 3 seconds");
		delay(3000);
	}
}

void MPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14, (int)true);

	rawAccX = wire->read() << 8 | wire->read();
	rawAccY = wire->read() << 8 | wire->read();
	rawAccZ = wire->read() << 8 | wire->read();
	rawTemp = wire->read() << 8 | wire->read();
	rawGyroX = wire->read() << 8 | wire->read();
	rawGyroY = wire->read() << 8 | wire->read();
	rawGyroZ = wire->read() << 8 | wire->read();

	temp = (rawTemp + 12412.0) / 340.0;

	accX = ((float)rawAccX) / 16384.0;
	accY = ((float)rawAccY) / 16384.0;
	accZ = ((float)rawAccZ) / 16384.0;

	angleAccX = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 360 / 2.0 / PI;
	angleAccY = atan2(accX, sqrt(accY * accY + accZ * accZ)) * 360 / -2.0 / PI;

	gyroX = ((float)rawGyroX) / 65.5;
	gyroY = ((float)rawGyroY) / 65.5;
	gyroZ = ((float)rawGyroZ) / 65.5;

	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	interval = millis() - preInterval;

	angleGyroX += gyroX * (interval * 0.001);
	angleGyroY += gyroY * (interval * 0.001);
	angleGyroZ += gyroZ * (interval * 0.001);

	preInterval = millis();

	if (angleGyroX < 70 && angleGyroX > -70) {
		angleX = (gyroCoef * angleGyroX) + (accCoef * angleAccX);
	} else {
		angleX = angleAccX;
	}
	if (angleGyroY < 70 && angleGyroY > -70) {
		angleY = (gyroCoef * angleGyroY) + (accCoef * angleAccY);
	} else {
		angleY = angleAccY;
	}
	angleZ = angleGyroZ;


	if(angleX > ANGLE_X_MAX){
		angleX = ANGLE_X_MAX;
	}
	if(angleX < ANGLE_X_MIN){
		angleX = ANGLE_X_MIN;
	}

	if(angleY > ANGLE_Y_MAX){
		angleY = ANGLE_Y_MAX;
	}
	if(angleY < ANGLE_Y_MIN){
		angleY = ANGLE_Y_MIN;
	}
}

