void setup() {
  Serial.begin(115200);

  Serial.println("start");

 

  pinMode(28, OUTPUT);
  digitalWrite(28, HIGH);
  
  Serial1.begin(460800);


}

void loop() {
  Serial.println("hello");
  Serial1.println("hello1");
  delay(1000);
}