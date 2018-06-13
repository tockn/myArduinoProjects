/*******************************************************************************
*  skHMC5883L - 方位センサHMC5883L用関数ライブラリ                             *
*                                                                              *
*    skHMC5883L     - この関数ライブラリを生成する時の初期化処理               *
*    Begin          - 方位センサの初期化を行う処理                             *
*    ActionMode     - 方位センサの動作モードを設定する処理                     *
*    SingleRead     - 単発読込みを行い方位の方向を角度で返す処理               *
*    ContinueRead   - 連続読込みを行い方位の方向を角度で返す処理               *
*    Receive        - 方位センサから指定個数のデータを受信する処理             *
*    Send           - 方位センサに指定個数のデータを送信する処理               *
*    GetOrientation - 方位の角度を１６分割する処理                             *
*    CalcDegree     - 方位の計算を行い方向を角度に変換する処理                 *
* ============================================================================ *
*   VERSION  DATE        BY             CHANGE/COMMENT                         *
* ---------------------------------------------------------------------------- *
*   1.00     2015-05-17  きむ茶工房     Create                                 *
* ============================================================================ *
*   Arduino IDE 1.7.3    (Ardino Duemilanove 328/UNO)(Arduino Zero Pro)        *
*******************************************************************************/
#include <Wire.h>
#include "arduino.h"
#include "skHMC5883L.h"


int16_t RowX ;               // 読み出したセンサー値の生のＸ値
int16_t RowY ;               // 読み出したセンサー値の生のＹ値
int16_t RowZ ;               // 読み出したセンサー値の生のＺ値

/*******************************************************************************
*  skHMC5883L(address)                                                         *
*    この関数ライブラリを生成する時の初期化処理(コンストラクタ)                *
*    address : 方位センサ(スレーブ)のI2Cアドレスを指定します                   *
*******************************************************************************/
skHMC5883L::skHMC5883L(uint8_t address)
{
     Sensor_adrs = address ;
}
/*******************************************************************************
*  ans = Begin()                                                               *
*    方位センサの初期化を行う処理                                              *
*    設定のデータは、データのサンプル数は８回で設定、サンプルレートは15Hzです  *
*    通常の”測定モード”に設定し、ゲイン(検出レンジ)は1.3Gaussで設定。        *
*    初期デバイスの動作モードは"アイドル"でスタートです。                      *
*    ans  : 戻り値、0=正常終了　それ以外I2C通信エラー下記                      *
*                   1=送ろうとしたデータが送信バッファのサイズを超えた         *
*                   2=スレーブ・アドレスを送信し、NACKを受信した               *
*                   3=データ・バイトを送信し、NACKを受信した                   *
*                   4=その他のエラー                                           *
*                   5=データ受信エラー                                         *
*******************************************************************************/
uint8_t skHMC5883L::Begin()
{
     uint8_t  ans ;
     unsigned char data[4] ;

     // 方位センサを初期化する処理
     data[0] = CONF_REGa_DATA ;    // コンフィギュレーションAの設定(AVG=8 Rate=15Hz normal measurement)
     data[1] = CONF_REGb_DATA ;    // コンフィギュレーションBの設定(Gain=1.3Ga)
     data[2] = H43_MODE_IDLE ;     // モードの設定(Idle Mode)
     ans = Send(CONF_REGa_ADRS,&data[0],3) ;
     return ans ;
}
/*******************************************************************************
*  ans = ActionMode(mode)                                                      *
*  方位センサの動作モード(測定モード)を設定する処理                            *
*    mode : 動作モードを指定する                                               *
*           H43_MODE_IDLE／H43_MODE_SINGLE／H43_MODE_CONTINUE                  *
*    ans  : 戻り値、0=正常終了　それ以外Begin()のans値を参照                   *
*******************************************************************************/
uint8_t skHMC5883L::ActionMode(uint8_t mode)
{
     uint8_t  ans ;
     unsigned char data[2] ;

     data[0] = mode ;
     ans = Send(MODE_REG_ADRS,&data[0],1) ;
     delay(6) ;     // データ準備完了まで待つ
     return ans ;
}
/*******************************************************************************
*  ans = Receive(reg_adrs,*data,kosu)                                          *
*  方位センサから指定個数のデータを受信する処理                                *
*    reg_adrs : 読出すデータのレジスターアドレスを指定する                     *
*               連続的に読出す場合は、読出すレジスターの先頭アドレスを指定     *
*    *data    : 読出したデータの格納先を指定する                               *
*    kosu     : 読出すデータのバイト数を指定する                               *
*    ans      : 戻り値、0=正常終了　それ以外Begin()のans値を参照               *
*******************************************************************************/
uint8_t skHMC5883L::Receive(char reg_adrs,unsigned char *data,char kosu)
{
     uint8_t  ans , i ;

     Wire.beginTransmission(Sensor_adrs) ;             // 通信の開始
     Wire.write(reg_adrs) ;                            // レジスタアドレスを指定
     ans = Wire.endTransmission() ;                    // データの送信と通信の終了
     if (ans == 0) {
          ans = Wire.requestFrom((int)Sensor_adrs,(int)kosu) ;   // データの受信を行う
          if (ans == kosu) {
               for (i=0 ; i<kosu ; i++) {
                    *data = Wire.read() ;
                    data++ ;
               }
               ans = 0 ;
          } else ans = 5 ;                             // 受信バイト数がおかしい？
     }
     return ans ;
}
/*******************************************************************************
*  ans = Send(reg_adrs,*data,kosu)                                             *
*  方位センサに指定個数のデータを送信する処理                                  *
*    reg_adrs : 書出すデータのレジスターアドレスを指定する                     *
*               連続的に書出す場合は、書出すレジスターの先頭アドレスを指定     *
*    *data    : 書出すデータの格納先を指定する                                 *
*    kosu     : 書出すデータのバイト数を指定する                               *
*    ans      : 戻り値、0=正常終了　それ以外Begin()のans値を参照               *
*******************************************************************************/
uint8_t skHMC5883L::Send(char reg_adrs,unsigned char *data,char kosu)
{
     uint8_t  ans , i ;

     Wire.beginTransmission(Sensor_adrs) ;        // 通信の開始
     Wire.write(reg_adrs) ;                       // レジスタのアドレスを通知
     for (i=0 ; i<kosu ; i++) {
          Wire.write(*data) ;                     // データを通知
          data++ ;
     }
     ans = Wire.endTransmission() ;               // データの送信と通信の終了
     return ans ;
}
/*******************************************************************************
*  ans = GetOrientation(deg)                                                   *
*  方位の角度を１６分割する処理                                                *
*    deg  : 方位の角度を指定(0.0-360.0度)                                      *
*    ans  : 角度に対応する分割の位置を返す(16分割なら0-15を返す)               *
*******************************************************************************/
uint8_t skHMC5883L::GetOrientation(float deg)
{
     uint8_t ans ;
     float d , val ;

     val = 360.0 / DIRECTION_DIVISION ; // 16分割(変更可能)
     d = deg + (val/2) ;
     d -= (uint8_t)(d / 360.0) * 360.0 ;
     ans = (uint8_t)(d / val) ;
     return ans ;
}
/*******************************************************************************
*  ans = CalcDegree(dec)                                                       *
*  方位の計算を行い方向を角度に変換する処理                                    *
*    dec  : 磁気偏角を度で指定、8°30′の場合8.5°(8+30/60)を指定              *
*    ans  : 0.0-360.0度の値を返す                                              *
*******************************************************************************/
float skHMC5883L::CalcDegree(float dec)
{
     float X , Y , ans ;

     X = (RowX * H43_GAIN_SCALE) + (ADJUST_X_ANGLE) ;
     Y = (RowY * H43_GAIN_SCALE) + (ADJUST_Y_ANGLE) ;
     ans = atan2(Y, X) ;
     if (ans < 0) ans += 2*PI ;
     if (ans > 2*PI) ans -= 2*PI;
     ans = ans * 180/M_PI ;
     // 西偏(日本)の場合で磁気偏角を調整する
     ans = ans + dec ;
     if (ans > 360.0) ans = ans - 360.0 ;
     return ans ;
}
/*******************************************************************************
*  ans = ContinueRead(deg,dec)                                                 *
*  連続読込みを行い方位の方向を角度で返す処理                                  *
*  方位センサより連続読込みを行い、方位の計算をして度数を"deg"にセットする。   *
*  ActionMode(H43_MODE_CONTINUE)を初期化で実行させて置く必要が有ります。       *
*    deg  : 方位の計算結果(0-360)を返すのでfloat変数のアドレスを指定           *
*    dec  : 磁気偏角を度で指定、8°30′の場合8.5°(8+30/60)を指定              *
*    ans  : 戻り値、0=正常終了　それ以外Begin()のans値を参照                   *
*******************************************************************************/
uint8_t skHMC5883L::ContinueRead(float *deg,float dec)
{
     uint8_t  ans ;
     unsigned char data[6] ;

     ans = Receive(DATA_START_ADRS,&data[0],6) ;
     if (ans == 0) {
          RowX = (data[0] << 8) | data[1];
          RowZ = (data[2] << 8) | data[3];
          RowY = (data[4] << 8) | data[5];
          *deg = CalcDegree(dec);    // 方位の角度を計算する
     }
     return ans ;
}
/*******************************************************************************
*  ans = SingleRead(deg,dec)                                                   *
*  単発読込みを行い方位の方向を角度で返す処理                                  *
*  方位センサよりシングル読込みを行い、方位の計算をして度数を"deg"にセットする *
*  ６msの待ち時間が発生します。                                                *
*    deg  : 方位の計算結果(0-360)を返すのでfloat変数のアドレスを指定           *
*    dec  : 磁気偏角を度で指定、8°30′の場合8.5°(8+30/60)を指定              *
*    ans  : 戻り値、0=正常終了　それ以外Begin()のans値を参照                   *
*******************************************************************************/
uint8_t skHMC5883L::SingleRead(float *deg,float dec)
{
     uint8_t  ans ;
     unsigned char data[6] ;

     // シングル測定モードを設定する
     ans = ActionMode(H43_MODE_SINGLE) ;
     if (ans == 0) {
          ans = Receive(DATA_START_ADRS,&data[0],6) ;
          if (ans == 0) {
               RowX = (data[0] << 8) | data[1];
               RowZ = (data[2] << 8) | data[3];
               RowY = (data[4] << 8) | data[5];
               *deg = CalcDegree(dec);    // 方位の角度を計算する
          }
     }
     return ans ;
}
