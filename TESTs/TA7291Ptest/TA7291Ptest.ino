int MOTER1PIN = 9;
int MOTER2PIN = 10;

void setup() 
{ 
  pinMode(MOTER1PIN, OUTPUT);
  pinMode(MOTER2PIN, OUTPUT);
  Serial.begin(115200);
}

int n = 0, diff = 1;

void loop() 
{ 
  analogWrite(MOTER1PIN,255);
  analogWrite(MOTER2PIN,0);
  Serial.println(n);

  n += diff;
  if (n >= 255){
    diff = -1;
  }
  if (n <= 0){
    diff = 1;
  }

  delay(50);
} 
