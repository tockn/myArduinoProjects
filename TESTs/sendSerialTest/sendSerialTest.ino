void setup() {
  Serial.begin(115200);
}

void loop() {
  sendIntData(12345); // int型データの送信
  delay(500);
  
  sendIntData(-12345); // int型データの送信
  delay(500); 
}

// int型のデータを送信する関数
void sendIntData(int value) {
  Serial.write('H'); // ヘッダの送信
  Serial.write(lowByte(value)); // 下位バイトの送信
  Serial.write(highByte(value)); // 上位バイトの送信
}
