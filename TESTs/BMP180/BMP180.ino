#include <Wire.h>

#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  while (1) {}
  }
}

float P0 = 1016.1;
float raw_p = 0;
float raw_t = 0;
void loop() {

  for(int i = 0; i < 10; i++){
    raw_p += bmp.readPressure();
    raw_t += bmp.readTemperature();
  }
  raw_p /= 10;
  raw_t /= 10;
  
  float TEMP = raw_t;
  float hPa = raw_p / 100;
  float alt = 44330 * (1 - pow((hPa/P0), (1/5.255)));
    /*
    Serial.print("Temperature = ");
    Serial.print(TEMP);
    Serial.print(" *C");
    Serial.print("\t");
    Serial.print("Pressure = ");
    Serial.print(hPa);
    Serial.print(" hPa");
    Serial.print("\t");
    Serial.print("Altitude = ");
    Serial.print(alt);
    Serial.println(" m");
    */

  raw_p = 0;
  raw_t = 0;

}
