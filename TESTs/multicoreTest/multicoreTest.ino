float pi = 3.14159; //円周率
float radA = (2 * pi) / 360; //ラジアンを角度(0-360度)単位にする
 
int mainAngle = 0; //メインloop のサイン波の初期角度
float mainSin = 0; //メインloop のサイン波の値
 
portTickType Delay1000 = 1000 / portTICK_RATE_MS; //freeRTOS 用の遅延時間定義
TaskHandle_t th[2];
 
void Task1(void *pvParameters) {
  int task1Angle = 45; //Task1 のサイン波の初期角度
  float task1Sin = 0; //Task1 のサイン波の値
   
  while(1) {
    Serial.printf("Task1 coreID = %d, Task1 priority = %d, task1Sin = %F\r\n", xPortGetCoreID(), uxTaskPriorityGet(th[0]), task1Sin);
    task1Sin = float(sin(radA * task1Angle)) * 1000; //角度をラジアンに変換しsin値を得て、1000倍にして表示しやすいようにする
    task1Angle++;
    if(task1Angle >= 360) task1Angle = 0; //角度が360度になったらゼロ
    vTaskDelay(Delay1000); //freeRTOS用のディレイタイム実行
  }
}
 
void Task2(void *pvParameters) {
  int task2Angle = 90; //Task2 のサイン波の初期角度
  float task2Sin = 0; //Task2 のサイン波の値
   
  while(1) {
    Serial.printf("Task2 coreID = %d, Task2 priority = %d, task2Sin = %F\r\n", xPortGetCoreID(), uxTaskPriorityGet(th[1]), task2Sin);
    task2Sin = float(sin(radA * task2Angle)) * 1000; //角度をラジアンに変換しsin値を得て、1000倍にして表示しやすいようにする
    task2Angle++;
    if(task2Angle >= 360) task2Angle = 0; //角度が360度になったらゼロ
    vTaskDelay(Delay1000); //freeRTOS用のディレイタイム実行
  }
}
 
void setup() {
  Serial.begin(115200);
  Serial.println();
 
  xTaskCreatePinnedToCore(Task1,"Task1", 4096, NULL, 3, &th[0], 0); //Task1実行
  xTaskCreatePinnedToCore(Task2,"Task2", 4096, NULL, 5, &th[1], 1); //Task2実行
}
 
void loop() {
  Serial.printf("loop  coreID = %d, loop  priority = %d, mainSin = %F\r\n", xPortGetCoreID(), uxTaskPriorityGet(NULL), mainSin);
  mainSin = float(sin(radA * mainAngle)) * 1000; //角度をラジアンに変換しsin値を得て、1000倍にして表示しやすいようにする
  mainAngle++;
  if(mainAngle >= 360) mainAngle = 0; //角度が360度になったらゼロ
  delay(1000);
}
