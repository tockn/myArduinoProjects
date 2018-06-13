/*******************************************************************************
*  skHMC5883L.h - 方位センサHMC5883L用関数ライブラリのインクルードファイル     *
*                                                                              *
* ============================================================================ *
* ============================================================================ *
*   VERSION  DATE        BY             CHANGE/COMMENT                         *
* ---------------------------------------------------------------------------- *
*   1.00     2015-05-17  きむ茶工房     Create                                 *
* ============================================================================ *
*   Arduino IDE 1.7.3    (Ardino Duemilanove 328/UNO)(Arduino Zero Pro)        *
*******************************************************************************/
#ifndef SKHMC5883L_h
#define SKHMC5883L_h

#include "arduino.h"

#define ADJUST_X_ANGLE     -252         // Ｘ軸を調整する値
#define ADJUST_Y_ANGLE     +102         // Ｙ軸を調整する値
#define DIRECTION_DIVISION 16           // 方位を何分割するか指定する

// デバイスのレジスタアドレス
#define CONF_REGa_ADRS     0x00         // コンフィギュレーションＡの設定レジスタアドレス
#define CONF_REGb_ADRS     0x01         // コンフィギュレーションＢの設定レジスタアドレス
#define MODE_REG_ADRS      0x02         // モードの設定レジスタアドレス
#define DATA_START_ADRS    0x03         // 読み出すデータの先頭レジスタアドレス

// コンフィギュレーションＡの設定
#define CONF_REGa_DATA     0b01110000   // B6-5=AVG(8) B4-2=Rate(15Hz) B1-0=normal measurement
// コンフィギュレーションＢの設定
// ゲイン(検出レンジ)を変える場合は、H43_GAIN_SCALEも変えましょう
#define CONF_REGb_DATA     0b00100000   // B7-5=Gain(1.3Ga) B4-0=(00000)
#define H43_GAIN_SCALE     (float)0.92  // 1.3Gauss->Digital Resolution=0.92
// モードの設定
#define H43_MODE_IDLE      0b00000011   // B7-2=(00000) B4-0=Mode(Idle Mode)
#define H43_MODE_SINGLE    0b00000001   // B7-2=(00000) B4-0=Mode(Single-Measurement Mode)
#define H43_MODE_CONTINUE  0b00000000   // B7-2=(00000) B4-0=Mode(Continuous-Measurement Mode)


extern int16_t RowX ;                   // 読み出したセンサー値の生のＸ値
extern int16_t RowY ;                   // 読み出したセンサー値の生のＹ値
extern int16_t RowZ ;                   // 読み出したセンサー値の生のＺ値

/*******************************************************************************
*	クラスの定義                                                              *
*******************************************************************************/
class skHMC5883L
{
  private:
    uint8_t Sensor_adrs ;
    float   CalcDegree(float dec) ;

  public:
            skHMC5883L(uint8_t address) ;
    uint8_t Begin(void) ;
    uint8_t ActionMode(uint8_t mode) ;
    uint8_t Receive(char reg_adrs,unsigned char *data,char kosu) ;
    uint8_t Send(char reg_adrs,unsigned char *data,char kosu) ;
    uint8_t ContinueRead(float *deg,float dec) ;
    uint8_t SingleRead(float *deg,float dec) ;
    uint8_t GetOrientation(float deg) ;
} ;

#endif
