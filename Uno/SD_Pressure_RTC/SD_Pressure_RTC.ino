/*
  SD card datalogger
  This example shows how to log data from three analog sensors
  to an SD card using the SD library. Pin numbers reflect the default
  SPI pins for Uno and Nano models
  The circuit:
   analog sensors on analog ins 0, 1, and 2
   SD card attached to SPI bus as follows:
 ** SDO - pin 11
 ** SDI - pin 12
 ** CLK - pin 13
 ** CS - depends on your SD card shield or module.
        Pin 10 used here for consistency with other Arduino examples
    (for MKRZero SD: SDCARD_SS_PIN)
  created  24 Nov 2010
  modified  24 July 2020
  by Tom Igoe
  This example code is in the public domain.
*/
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <DS3231.h>

DS3231 clock;
RTCDateTime dt;
const int chipSelect = 10;
int sensorPin = A0;
int baseline = 105;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.println("Initialize RTC module");
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);  
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  while (!Serial);
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
    while (true);
  }
  Serial.println("initialization done.");
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
  dt = clock.getDateTime();
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    printTimestamp(dt, dataFile);
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

void printTimestamp(RTCDateTime currentTime, File outputFile){
  outputFile.print(currentTime.month);
  outputFile.print("/");
  outputFile.print(currentTime.day);
  outputFile.print("/");
  outputFile.print(currentTime.year);
  outputFile.print(", ");
  
  outputFile.print(currentTime.hour);
  outputFile.print(":");
  outputFile.print(currentTime.minute);
  outputFile.print(":");
  outputFile.print(currentTime.second);
  outputFile.print(", ");
  

  Serial.print(dt.month);   Serial.print("/");
  Serial.print(dt.day);  Serial.print("/");
  Serial.print(dt.year);    Serial.print(", ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.print(", ");


}
