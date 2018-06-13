// http://code.google.com/p/ardroneme/
// From the file NavData2.zip 

/* Updated by MAPGPS based on Richi's code:
 * http://ulrichard.is-a-geek.net/?p=23
 */

//Update by boulderjoe 12/30/11 to work with arduino 1.0 

#ifndef INTERSEMA_BARO_H
#define INTERSEMA_BARO_H

#include <Wire.h>
#include <util/delay.h>
#include "Arduino.h"

namespace Intersema
{
  
class BaroPressure
{
public:
    virtual void init() = 0;   
   
    int32_t getPressurePascals(void) {
      return AcquireAveragedSamplePascal(1);
    }
      
    int32_t getHeightCentiMeters(void) {
        return AcquireAveragedSampleCm(1);
    }
    
    int32_t getTemperature(void) {
      return AcquireAveragedTempSample(1);
    }
    
protected:
    virtual int32_t AcquireAveragedTempSample(const uint8_t nSamples) = 0;
    virtual int32_t AcquireAveragedSamplePascal(const uint8_t nSamples) = 0;
    virtual int32_t AcquireAveragedSampleCm(const uint8_t nSamples) = 0;
    virtual uint32_t ConvertPressureTemperature(uint32_t pressure, uint32_t temperature) = 0;
    
    static const uint8_t NUM_SAMP_FOR_AVG = 4;

    // Values of the calibration coefficients, in order.
    unsigned int coefficients_[6];
    
    int32_t PascalToCentimeter(const int32_t pressurePa)
    {
        // Lookup table converting pressure in Pa to altitude in cm.
        // Each LUT entry is the altitude in cm corresponding to an implicit
        // pressure value, calculated as [PA_INIT - 1024*index] in Pa.
        // The table is calculated for a nominal sea-level pressure  = 101325 Pa.
        static const int32_t PZLUT_ENTRIES = 77;
        static const int32_t PA_INIT       = 104908;
        static const int32_t PA_DELTA      = 1024;

        // Consider using PROGMEM? http://arduino.cc/en/Reference/PROGMEM http://www.atmel.com/images/doc8453.pdf
        static const int32_t lookupTable[PZLUT_ENTRIES] = {
	    -29408, -21087, -12700,  -4244,   4279,
	     12874,  21541,  30281,  39095,  47986,
	     56953,  66000,  75126,  84335,  93628,
	    103006, 112472, 122026, 131672, 141410,
	    151244, 161174, 171204, 181335, 191570,
	    201911, 212361, 222922, 233597, 244388,
	    255300, 266334, 277494, 288782, 300204,
	    311761, 323457, 335297, 347285, 359424,
	    371719, 384174, 396795, 409586, 422552,
	    435700, 449033, 462560, 476285, 490216,
	    504360, 518724, 533316, 548144, 563216,
	    578543, 594134, 609999, 626149, 642595,
	    659352, 676431, 693847, 711615, 729752,
	    748275, 767202, 786555, 806356, 826627,
	    847395, 868688, 890537, 912974, 936037,
	    959766, 984206};

        if(pressurePa > PA_INIT) 
             return lookupTable[0];
        else 
        {
           const int32_t inx = (PA_INIT - pressurePa) >> 10;      
           if(inx >= PZLUT_ENTRIES - 1) 
               return lookupTable[PZLUT_ENTRIES - 1];
           else 
           {
                const int32_t pa1 = PA_INIT - (inx << 10);
                const int32_t z1 = lookupTable[inx];
                const int32_t z2 = lookupTable[inx+1];
                return (z1 + (((pa1 - pressurePa) * (z2 - z1)) >> 10));
            }
        }
    }
};

class BaroPressure_MS5607B : public BaroPressure
{
public:
    // @param CSB  i2c address select
    // 0xEC is aka DEVICE_ADDR
    BaroPressure_MS5607B(bool CSB = false) : i2cAddr_((CSB ? 0xEC : 0xEE) >> 1) { }
    
    void init()
    {    
        ResetSensor();
        ReadCoefficients();
    }
    
private:
    static const uint8_t DEVICE_READ      = 0x01; // got this from the Propellor code but not sure what it does
    
    
    const uint8_t i2cAddr_;
    static const uint8_t cmdReset_   = 0x1E;
    static const uint8_t cmdAdcRead_ = 0x00;
    static const uint8_t cmdAdcConv_ = 0x40;
    static const uint8_t cmdPromRd_  = 0xA0;
    
    // unadjusted pressure (add to cmdAdcConv_ to select D1)
    static const uint8_t cmdAdcD1_   = 0x00;
    // unadjusted temperature (add to cmdAdcConv_ to select D2)
    static const uint8_t cmdAdcD2_   = 0x10;
    
    // Oversampling Ratio (OSR) selectors (added to cmdAdcConv_ and D1 or D2 to select the Oversampling Ratio)
    // which change the Resolution RMS (root mean square).
    // Typical resolution RMS in Celsius at VDD = 3V, T = 25 Celsius in comments:
    static const uint8_t cmdAdc256_  = 0x00; //0.012
    static const uint8_t cmdAdc512_  = 0x02; //0.008
    static const uint8_t cmdAdc1024_ = 0x04; //0.005
    static const uint8_t cmdAdc2048_ = 0x06; //0.003
    static const uint8_t cmdAdc4096_ = 0x08; //0.002

    void ResetSensor()
    {
	Wire.begin();
        Wire.beginTransmission(i2cAddr_);
        Wire.write(cmdReset_);   
        Wire.endTransmission(); 
        delay(3);
    }

    /*
    This populates the calibration coefficient values and stores them in a member variable.
    
    My chip shows coefficient values:
    1: 43215
    2: 38230
    3: 26771
    4: 24522
    5: 32706
    6: 27652
    */
    void ReadCoefficients(void)
    {
        for(uint8_t i=0; i<6; ++i)
        {
            coefficients_[i] = ReadCoefficient(i + 1);
        }
        
#ifdef DEBUG
	for(uint8_t i=0; i<6; ++i)
        {
            Serial.print("Coefficient ");
            Serial.print(i + 1, DEC);
            Serial.print(" : ");
            Serial.println(coefficients_[i], DEC);
        }
        Serial.println(ConvertPressureTemperature(6074082, 8574974));
        Serial.println(ConvertPressureTemperature(6074082, 8574984));
#endif
    }

    /*
    coefNum: a number from 0 to 6 selecting a coefficient value to return
    */
    uint16_t ReadCoefficient(const uint8_t coefNum)
    {
        uint16_t result = 0;
    
        Wire.beginTransmission(i2cAddr_);
        
        // send PROM READ command
        // The address of the PROM is embedded inside of the PROM read command using the Ad2, Ad1 and Ad0 bits,
        // those being the 2nd, 3rd, and 4th bits. Thus we multiply the address by 2 to make it start at the 2nd bit.
        Wire.write(cmdPromRd_ + coefNum * 2);
        Wire.endTransmission(); 
    
        // PROM reads return two bytes
        Wire.requestFrom(i2cAddr_, static_cast<uint8_t>(2));

        if(Wire.available() >= 2)
        {
            // Concatenate the first & second byte to create a 16 bit uint.
            // read MSB and acknowledge
            uint8_t tmp = Wire.read(); //was previously uint16_t... but just a byte should be ok?
            uint16_t result  = tmp << 8;
            // read LSB and not acknowledge
            tmp = Wire.read();
            result  = result + tmp;
            return result;
        }
#ifdef DEBUG
        else
        {
	    Serial.println("No data available in ReadCoefficient()");
        }
#endif 
    
        return 0;
    }

    //this method by SC. eventually, unify this with the pressure accumulated samples, because it does the same work of reading the temperature
    virtual int32_t AcquireAveragedTempSample(const uint8_t nSamples) {
      int64_t tempAccum = 0;
      
      for(size_t n = nSamples; n; n--) {
        const uint32_t temperature = ReadAdc(cmdAdcD2_ | cmdAdc4096_); // digital temperature value : typical 8077636
        const int32_t tempConv = convertTemperature(temperature);
        tempAccum += tempConv;
      }
      
      const int32_t tempAvg = tempAccum / nSamples;
      
      return tempAvg;
    }
    virtual int32_t AcquireAveragedSampleCm(const uint8_t nSamples)
    {
        const int32_t pressAvg = AcquireAveragedSamplePascal(nSamples);        
        const int32_t AltCm = PascalToCentimeter(pressAvg);
	
        return AltCm;	
    }
    virtual int32_t AcquireAveragedSamplePascal(const uint8_t nSamples)
    {
        int64_t pressAccum = 0;

        for(size_t n = nSamples; n; n--) 
        {
            const uint32_t temperature = ReadAdc(cmdAdcD2_ | cmdAdc4096_); // digital temperature value : typical 8077636
            const uint32_t pressure    = ReadAdc(cmdAdcD1_ | cmdAdc4096_); // digital pressure value : typical 6465444
            const uint32_t pressConv   = ConvertPressureTemperature(pressure, temperature);                 
            pressAccum += pressConv;
/*
            //MAPGPS
            Serial.print("pressure: ");
            Serial.print(pressure, DEC);
            Serial.print(", pressConv: ");
            Serial.print(pressConv, DEC);
            Serial.print(", temperature: ");
            Serial.println(temperature, DEC);
*/
        }

        const int32_t pressAvg = pressAccum / nSamples;
	
        return pressAvg;	
    }
    
    int32_t ReadAdc(const uint8_t cmd)
    {             
        Wire.beginTransmission(i2cAddr_);
        Wire.write(cmdAdcConv_ | cmd); // send conversion command
        Wire.endTransmission(); 

        // wait necessary conversion time
        

		switch(cmd & 0x0f) 
        {
        case cmdAdc256_: 
            delay(1);
            break;
        case cmdAdc512_: 
            delay(3);
            break;
        case cmdAdc1024_: 
            delay(4);
            break;
        case cmdAdc2048_: 
            delay(6);
            break;
        case cmdAdc4096_: 
            delay(10); 
            break;
        }

        Wire.beginTransmission(i2cAddr_);
        Wire.write(cmdAdcRead_);
        Wire.endTransmission();
    
        Wire.requestFrom(i2cAddr_, static_cast<uint8_t>(3));

        if(Wire.available() >= 3)
        {
            uint16_t ret  = Wire.read(); // read MSB and acknowledge
            uint32_t temp = 65536 * ret;
            ret  = Wire.read();      // read byte and acknowledge
            temp = temp + 256 * ret;
            ret  = Wire.read();  // read LSB and not acknowledge
            temp = temp + ret;
                
            return temp;
        }
#ifdef DEBUG
        else
        {
	    Serial.println("No data available in cmdAdc()");
        }
#endif 
        
        return 0;
    }

    /*
    Parameters: unadjusted pressure and temperature from D1 and D2.
    Returns temperature compensated pressure according to the 1st-order algorithm (inaccurate for temperatures below 20 C)
    */
    uint32_t ConvertPressureTemperature(uint32_t pressure, uint32_t temperature) {
        // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
        const int32_t dT    = temperature - coefficients_[4] * 256;                     // difference between actual and reference temperature
        //const int32_t temp  = (2000 + (dT * coefficients_[5]) / pow(2, 23)) ; // / 100;       // actual temperature

        //const int64_t OFF   = static_cast<int64_t>(coefficients_[1]) * pow(2, 17) + dT * coefficients_[3] / pow(2, 6); // offset at actual temperature
        //const int64_t SENS  = static_cast<int64_t>(coefficients_[0]) * pow(2, 16) + dT * coefficients_[2] / pow(2, 7); // sensitivity at actual temperature
        //const int32_t press = ((pressure * SENS / pow(2, 21) - OFF) / pow(2, 15)); // / 100;      // temperature compensated pressure
         
        //MAPGPS: adapt formulas to avoid overflow
        const int32_t OFF   = coefficients_[1] * 4 + ((float)dT / 2048) * ((float)coefficients_[3] / 1024);
        const int32_t SENS  = coefficients_[0] * 2 + ((float)dT / 4096) * ((float)coefficients_[2] / 1024);
        const int32_t press =  ((float)pressure / 2048) * ((float)SENS / 1024) - OFF;

/*
        Serial.println();
        Serial.println(dT, DEC);
        Serial.println(OFF, DEC);
        Serial.println(SENS, DEC);
        Serial.println();
*/
        return press; 
    }
    
    /*
    Converts d2 to temperature.
    
    A d2 of zero should return -58243.
    A d2 of 
    
    d2: the digital temperature value
    returns: temperature in cents of Celsius (divide by 100 to get C)
    */
    int32_t convertTemperature(uint32_t d2) {
      // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
      // For my chip: dT min = -8,372,736 and dT max = 8,404,480 (32 bit signed)
      // However, based on min & max TEMP, dT min = -1,820,181 and dT max = 1,971,862 (still 32 bit signed)
      // difference between actual and reference temperature
      const int32_t dT = d2 - coefficients_[4] * static_cast<int32_t>(256);
      const int32_t firstOrderTemperature = (2000 + static_cast<int32_t>((((float) dT) * coefficients_[5]) / pow(2, 23)));
      if (firstOrderTemperature < 2000) {
        const float t2 = (float) dT * (float) dT / pow(2, 31);
        const int32_t temp = firstOrderTemperature - t2;
        return temp;
      } else {
        return firstOrderTemperature;
      }
    }
};
} // namespace Intersema
#endif


