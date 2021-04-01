
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;

//For the GPS module:
//TX is connected to RX3 (Digital 15)
//RX is connected to TX3 (Digital 14)
#define GPSSerial Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();

String outputString = "";
String filename;

const int pressurePin = A0;
const int waterPin = A1;


void setup() {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

  initializeGPS();
  initializeSD();

  //timer = millis();

}

void loop() {
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    gpsString();
    outputString.concat(", ");
    outputString.concat(analogRead(pressurePin));
    outputString.concat(", ");
    outputString.concat(analogRead(waterPin));
    outputString.concat(", ");
    outputString.concat(GPS.fix);
    Serial.println(outputString);
    if(GPS.fix){
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open(filename, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(outputString);
      Serial.print("Printed to SD: ");
      Serial.println(filename);
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
    }
    outputString = "";
    
  }
}

void gpsString() {
  if(filename == NULL && GPS.fix){
    
  }
  outputString.concat(GPS.day);
  outputString.concat("/");
  outputString.concat(GPS.month);
  outputString.concat("/20");
  outputString.concat(GPS.year);
  outputString.concat(", ");
  if (GPS.hour < 10) {
    outputString.concat("0");
  }
  outputString.concat(GPS.hour);
  outputString.concat(":");
  if (GPS.minute < 10) {
    outputString.concat("0");
  }
  outputString.concat(GPS.minute);
  outputString.concat(":");
  if (GPS.seconds < 10) {
    outputString.concat("0");
  }
  outputString.concat(GPS.seconds);
  outputString.concat(", ");

  if (GPS.fix) {
    outputString.concat(String(GPS.latitudeDegrees, 5));
    outputString.concat(", ");
    outputString.concat(String(GPS.longitudeDegrees, 5));
  }
  else {
    outputString.concat("0");
    outputString.concat(", ");
    outputString.concat("0");
  }



}

void initializeGPS(){
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
}

void initializeSD(){
    Serial.println("Initializing SD...");
    SD.begin(chipSelect);
    Serial.println("SD Initialized. Awaiting GPS Fix...");
    timer = millis();
    while(GPS.fix == 0){
      char x = GPS.read();
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          Serial.print("Failed to parse NMEA value\t");
          Serial.println(timer);
          timer = millis();
          //break; // we can fail to parse a sentence in which case we should just wait for another
      }
      delay(1000);
      Serial.print("Awaiting GPS Fix...\t");
      Serial.println(GPS.fix);
    }
    Serial.println("GPS Fix found.");
    Serial.println(GPS.fix);
    if(GPS.month < 10) { filename.concat("0"); }
    filename.concat(GPS.month);
    if(GPS.day < 10) { filename.concat("0"); }
    filename.concat(GPS.day);
    if(GPS.hour < 10) { filename.concat("0"); }
    filename.concat(GPS.hour);
    if(GPS.minute < 10) { filename.concat("0"); }
    filename.concat(GPS.minute);
    filename.concat(".csv");
    Serial.print("Filename ");
    Serial.print(filename);
    Serial.println(" created.");

    File dataFile = SD.open(filename, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println("Date, Time, Latitude, Longitude, Force, Water Level");
      dataFile.close();
    }
    Serial.print("Header data written to ");
    Serial.println(filename);
}