void setup() {
  pinMode(8, OUTPUT);
}

void loop() {
  digitalWrite(8, HIGH);
  delay(1000);
  for (int i=0; i<5; i++) {
    digitalWrite(8, LOW);
    delay(100);
    digitalWrite(8, HIGH);
    delay(100);
  }
  digitalWrite(8, LOW);
  while(1){
    
  }
}
