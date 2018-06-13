void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(7, INPUT);
  pinMode(10, INPUT);
  Serial.begin(115200);
}
int n = 0, sayu, zengo, jouge, kaiten;
void loop() {
  // put your main code here, to run repeatedly:
  if(n == 0)sayu = pulseIn(2, HIGH);
  if(n == 1)zengo = pulseIn(3, HIGH);
  if(n == 2)jouge = pulseIn(7, HIGH);
  if(n == 3)kaiten = pulseIn(10, HIGH);
  
  Serial.print("SAYU : ");
  Serial.print(sayu);
  Serial.print("\tZENGO : ");
  Serial.print(zengo);
  Serial.print("\tJOUGE : ");
  Serial.print(jouge);
  Serial.print("\tKAITEN : ");
  Serial.println(kaiten);
  
  n++;
  //Serial.println(n);
  if(n == 4) n = 0;
}
