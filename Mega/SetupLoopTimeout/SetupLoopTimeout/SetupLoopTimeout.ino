void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(1){
    Serial.println("In Setup Loop");
    delay(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("In Main Loop");
}