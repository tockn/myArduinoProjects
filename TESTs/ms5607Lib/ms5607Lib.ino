#include <math.h> // pow関数を使うので一応
 
#include <Wire.h>
#include <MS5xxx.h>
 
MS5xxx sensor(&Wire);
float cal = 0;
void setup() {
  Serial.begin(9600);
  if (sensor.connect() > 0) {
    Serial.println("Error connecting...");
    delay(500);
    setup();
  }
  sensor.ReadProm();
  sensor.Readout();
 
  float temp = sensor.GetTemp() / 100;
  float pres = sensor.GetPres() / 100;
  for(int i = 0; i<10; i++){
    cal += getHeight(temp, pres);
  }
  cal/=10;
}
 float pres=0, temp=0;
void loop() {
  pres= 0;
  temp = 0;
   sensor.ReadProm();
  sensor.Readout();
 
  for(int i = 0; i < 30; i++){
 
  temp += sensor.GetTemp() / 100;
  pres += sensor.GetPres() / 100;
  }
  temp/=30;
  pres/=30;
  Serial.print("Pressure [hPa]: ");
  Serial.print(pres);
  Serial.print("\t");
  Serial.print("Height [m]: ");
  Serial.println(getHeight(temp, pres) - cal);
  
}
 
float getHeight(float temperature, float pressure) {
  // http://keisan.casio.jp/exec/system/1257609530 と そのコメント13 より
  // P0: 海面気圧, P: 気圧, T: 気温(摂氏) 
  // h = (1013.25/P0)^(1/5.256) * ((P0/P)^(1/5.256)-1) * (T+273.15) / 0.0065
 
  return pow(1013.25/1013.25, 1/5.256) * ( pow(1013.25/pressure, 1/5.256) - 1.0 ) * (temperature + 273.15) / 0.0065;
}
