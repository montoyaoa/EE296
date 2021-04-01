
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int baseline = 105;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  Serial.print("Raw value: ");
  Serial.print(sensorValue);
  Serial.print(" ");
  Serial.print(sensorValue * 0.00488281250);
  Serial.print("V\tWeight: ");
  Serial.print((sensorValue - baseline) * 0.06426735219);
  Serial.print(" lb ");
  Serial.print("Pressure: ");
  Serial.print(((sensorValue - baseline) * 0.06103515625)/0.04382512987);
  Serial.print(" psi ");
  Serial.print((((sensorValue - baseline) * 0.06103515625)/0.04382512987)/14.69595);
  Serial.print(" atm ");
  Serial.print(((((sensorValue - baseline) * 0.06103515625)/0.04382512987)/14.69595)*33);
  Serial.println(" ft");

  delay(100);
}
