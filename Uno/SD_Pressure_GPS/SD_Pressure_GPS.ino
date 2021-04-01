#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

//GPS TX on Digital 8
//GPS RX on Digital 7
SoftwareSerial mySerial(9, 8);
Adafruit_GPS GPS(&mySerial);
const int chipSelect = 10;
int sensorPin = A0;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  if (!SD.begin(chipSelect)) {
    while (true);
  }
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Date, Time, Analog Sensor, Sample Text");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void loop() {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    //printTimestamp(dt, dataFile);
    dataFile.print(analogRead(sensorPin));
    dataFile.println(", Hello World!");
    dataFile.close();
    // print to the serial port too:
    Serial.print(analogRead(sensorPin));
    Serial.println(", Hello World written to SD card.");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  delay(500);
}

void printTimestamp(){
//  outputFile.print(currentTime.month);
//  outputFile.print("/");
//  outputFile.print(currentTime.day);
//  outputFile.print("/");
//  outputFile.print(currentTime.year);
//  outputFile.print(", ");
//  
//  outputFile.print(currentTime.hour);
//  outputFile.print(":");
//  outputFile.print(currentTime.minute);
//  outputFile.print(":");
//  outputFile.print(currentTime.second);
//  outputFile.print(", ");
//  
//
//  Serial.print(dt.month);   Serial.print("/");
//  Serial.print(dt.day);  Serial.print("/");
//  Serial.print(dt.year);    Serial.print(", ");
//  Serial.print(dt.hour);   Serial.print(":");
//  Serial.print(dt.minute); Serial.print(":");
//  Serial.print(dt.second); Serial.print(", ");


}
