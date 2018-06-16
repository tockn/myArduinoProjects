
#include<SoftwareSerial.h>
#include<DFPlayer_Mini_Mp3.h>

//音声を再生する間隔
unsigned long preTimer = 0;

//取得した高度の値をmp3ファイル名へ変換する時の変数
int one, ten, hund, thou;

//再生する音声のボリューム
int volume = 35;

//取得した高度の値が入る変数
int intAlt;

//再生する音声の間隔
const int voiceInterval = 700;

//DFPlayerとシリアル通信するポート
SoftwareSerial DFSerial(2, 3); //RX, TX

void setup() {
  
  //HardwareSerial、センサー基盤と9600bpsでシリアル通信を開始
  Serial.begin(9600);

  //DFPlayerと9600bpsでシリアル通信を開始
  DFSerial.begin(9600);
  //DFPlayerを起動
  mp3_set_serial(DFSerial);  //set softwareSerial for DFPlayer-mini mp3 module 
  //DFPlayerの初期ボリュームを設定。音割れしない最大値が３５？
  mp3_set_volume(35);  //max : 35 ?

  //ボリュームを設定するボタン
  pinMode(4, INPUT);
  //正しく起動中であることを示すLED
  pinMode(13, OUTPUT);
}

void loop() {

  //LEDを光らせる
  digitalWrite(13, HIGH);

  //音声再生時間になったら
  if (millis() - preTimer > voiceInterval) {
    //関数playを実行
    play();
    //音声再生時間を更新
    preTimer = millis();
  }

  //ボリューム変更ボタンが押されている時
  if (digitalRead(4) == HIGH) {
    //ボリュームつまみ(可変抵抗)の抵抗値を取得
    volume = analogRead(0);
    //抵抗値は0から1023の1024段階で取得される。それを0から50の51段階に変換
    volume = map(volume, 0, 1023, 0, 50);
    //取得した値にボリュームをセット
    mp3_set_volume(volume);
  }

  //センサー基盤から4バイト以上データが送られてきた時（送られてくるデータが" a -> a -> 高度(short型)の下位１バイト -> 高度の上位１バイト" の計４バイトで１セットなので）
  if (Serial.available() >= 4) {
    //データを１バイト取得
    char recv_data = (char)Serial.read();

    //先頭２バイトが a -> a だったら
    if (recv_data == 'a' && (char)Serial.read() == 'a') {
        //高度の下位１バイトを取得
        int low = Serial.read();
        //上位１バイトを取得
        int high = Serial.read();
        //関数makeWordを実行（上位１バイト下位１バイトをくっつける関数）
        intAlt = makeWord(high, low);
      }
    }
    //先頭１バイトが r だったら
    else if (recv_data == 'r') {
      //つぎに r -> r が送られてくるけど、いらないので消化～～
      Serial.read();
      Serial.read();
    }
  }
}

/*
 * 関数名：DFPlay
 * 引数：再生するmp3ファイル名(番号)が入る num
 * 説明：引数numのファイルをDFPlayerで再生する。
 */
void DFPlay(int num) {
  mp3_play(num);
}

/*
 * 関数名：play
 * 引数：なし
 * 説明：取得した高度の値から、対応するmp3ファイル名を取得。再生する。
 * 
 */
void play() {

  //取得した高度の値から、１の位、１０の位、１００の位、１０００の位を取得
  thou = intAlt / 1000;
  hund = (intAlt - thou * 1000) / 100;
  ten = (intAlt - thou * 1000 - hund * 100) / 10;
  one = (intAlt - thou * 1000 - hund * 100 - ten * 10);

  //各位から、mp3ファイル名に変換する関数changeValueを実行
  changeValue();
  //各位の音声を再生する間隔
  const int voiceNumInterval = 600;

  //各位の値に応じて音声を再生
  if (thou != 0) {
    DFPlay(thou);
    delay(voiceNumInterval);
  }
  if (hund != 0) {
    DFPlay(hund);
    delay(voiceNumInterval);
  }
  if (ten != 0) {
    DFPlay(ten);
    delay(voiceNumInterval);
  }
  if (one != 0) {
    DFPlay(one);
    delay(voiceNumInterval);
  }
}

/*
 * 関数名：changeValue
 * 引数：なし
 * 説明：高度の値の各位から対応するmp3ファイル名を取得
 */
void changeValue() {

  if (ten == 1)  ten = 10;
  else if (ten == 2) ten = 11;
  else if (ten == 3) ten = 12;
  else if (ten == 4) ten = 13;
  else if (ten == 5) ten = 14;
  else if (ten == 6) ten = 15;
  else if (ten == 7) ten = 16;
  else if (ten == 8) ten = 17;
  else if (ten == 9) ten = 18;

  if (hund == 1) hund = 19;
  else if (hund == 2) hund = 20;
  else if (hund == 3) hund = 21;
  else if (hund == 4) hund = 22;
  else if (hund == 5) hund = 23;
  else if (hund == 6) hund = 24;
  else if (hund == 7) hund = 25;
  else if (hund == 8) hund = 26;
  else if (hund == 9) hund = 27;

  if (thou == 1) thou = 28;
  else if (thou == 2) thou = 29;
  else if (thou == 3) thou = 30;

}

