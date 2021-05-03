//PIN ASSIGNMENTS FOR MEGA:
//GPS
//RX -> 14 (TX3)
//TX -> 15 (RX3)
//
//SD
//CS -> 53
//DI -> 51
//DO -> 50
//CLK -> 52
//
//PRESSURE
//SIGNAL -> A0
//
//WATER LEVEL
//SIGNAL -> A1
//
//LEDS
//ANODE -> ARDUINO, CATHODE -> 220 OHM -> GND
//WHITE (LOW BATTERY) -> 6
//RED (GPS LOCK) -> 8
//YELLOW (SD AVAILABLE) -> 9
//GREEN (ORIENTATION CALIBRATION) -> 10
//BLUE (WATER INTRUSION) -> 11
//
//POWERBOOST
//LB (LOW BATTERY) -> 7

//libraries needed (Tools -> Manage Libraries, search these names):
//Adafruit BMP085 Library
//Adafruit BNO055
//Adafruit GPS Library
//Adafruit MPL3115A2 Library
//Adafruit Unified Sensor
//RTClib
//SD
#include <Adafruit_BMP085.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include "RTClib.h"
#include <SD.h>
//Built-in libraries used:
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#define BAUD_RATE 115200
#define BNO055_SAMPLERATE_DELAY_MS 100

//set this to true to ignore the need to have a GPS fix to continue initialization
#define IGNOREGPSFIX true

//set this to true to write to the Serial IO for debugging
#define SERIALLOGGING false

//define pin assignments
#define GPSSerial Serial3
#define PRESSUREPIN A0
#define WATERPIN A1
#define CHIPSELECT 53
#define WHITELED 6
#define REDLED 8
#define YELLOWLED 9
#define GREENLED 10
#define BLUELED 11
#define LOWBATTERY 7


//define global variables
uint32_t timer = millis();
String outputString = "";
String filename = "";

//Initialize the hardware objects:
//GPS
Adafruit_GPS GPS(&GPSSerial);
//Orientation
Adafruit_BNO055 bno = Adafruit_BNO055();
//Barometric Pressure (depending on sensor)
Adafruit_BMP085 bmp = Adafruit_BMP085();
Adafruit_MPL3115A2 mpl = Adafruit_MPL3115A2();
//RTC
RTC_DS3231 rtc;

//setup is called once, on startup
void setup() {
  //connect to the serial monitor
  if(SERIALLOGGING) { Serial.begin(BAUD_RATE); }

  //initialize LED outputs
  pinMode(REDLED, OUTPUT);
  pinMode(YELLOWLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  pinMode(WHITELED, OUTPUT);

  //initialize Low battery input
  pinMode(LOWBATTERY, INPUT);
  
  //initialize orientation sensor
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);

  //initialize mplmetric pressure sensor
  bmp.begin();

  //initialize real time clock
  rtc.begin();

  initializeGPS();

  //only perform the GPS to RTC DateTime alignment once, after initial fix
  alignDateTime(rtc.now());
  
  initializeSD();
}

//loop is called continuously, after setup()
void loop() {
  //call the read and parse GPS function multiple times per second
  readAndParseGPS();
  
  //approximately every second or so, print out the current stats
  //the minimum interval between printing out stats should be 1.1s
  //any lower, and the GPS serial will not be called enough times 
  //to read a full NMEA sentence before the next one arrives
  if (millis() - timer > 1100) {
    //reset the timer
    timer = millis();
    
    //add sensor data to a single output string
    timeString(rtc.now());
    gpsString();
    pressureString();
    waterIntrusionString();
    calibrationString();
    quatString();
    eulerString();
    baroString();
    
    //write the output string to the SD card
    writeToSD();
    
    //clear the output string for the next readings
    outputString = "";

      //check if the battery breakout board reports the battery level is low
      if(digitalRead(LOWBATTERY) == LOW) {
        //if it is, trap into an infinite loop, flashing the low battery LED
        //stop cycling through the loop, to avoid drawing extra power
        while(1){
          digitalWrite(WHITELED, HIGH);
          delay(100);
          digitalWrite(WHITELED, LOW);
        }
      }
    }
  }
}

//called once in setup() to initialize GPS and get a fix
void initializeGPS() {
  if(SERIALLOGGING) { Serial.println("Initializing GPS..."); }
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  //only use basic position data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //update the GPS data once per second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  //ignore the need for a GPS fix if not necessary
  if(!IGNOREGPSFIX){
    //wait until first fix to continue initialization
    if(SERIALLOGGING) { Serial.print("Awaiting GPS fix... "); }
    //continually read from the GPS sensor until fix is found
    while (GPS.fix == 0) {
      readAndParseGPS();
    }
    if(SERIALLOGGING) { Serial.println("GPS fix found."); }
  }
}

//called once in setup() to initialize SD and create a filename
void initializeSD() {
  //initialize SD
  if(SERIALLOGGING) { Serial.println("Initializing SD..."); }
  SD.begin(CHIPSELECT);
  if(SERIALLOGGING) { Serial.println("SD Initialized"); }
  
  //set a dummy filename if GPS fix is being ignored
  if(IGNOREGPSFIX) { filename = "test"; }
  
  //generate a filename based on the date and time
  //syntax is: MMDDHHMM.csv
  if(SERIALLOGGING) { Serial.println("Generating filename from GPS date/time data..."); }
  while (filename == "00000000" || filename == "") {
    filename = "";
    if (GPS.month < 10) { filename.concat("0"); }
    filename.concat(GPS.month);
    if (GPS.day < 10) { filename.concat("0"); }
    filename.concat(GPS.day);
    if (GPS.hour < 10) { filename.concat("0"); }
    filename.concat(GPS.hour);
    if (GPS.minute < 10) { filename.concat("0"); }
    filename.concat(GPS.minute);
  }
  filename.concat(".csv");
  
  if(SERIALLOGGING){ 
    Serial.print("Filename ");
    Serial.print(filename);
    Serial.println(" created");
  }
  
  //write header data to SD
  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Date, Time, Latitude, Longitude, GPS Fix, Force, Water Level, Accel Calibration, Gyro Calibration, Magne Calibration, Sys Calibration, Quat W, Quat X, Quat Y, Quat Z, Euler X, Euler Y, Euler Z, Pressure (Pa), Altitude (m), Internal Temperature (C)");
    if(SERIALLOGGING) {
      Serial.print("Header data written to ");
      Serial.println(filename);
    }
  }
  else {
    if(SERIALLOGGING) {
      Serial.print("Unable to open ");
      Serial.println(filename);
    }
  }
  dataFile.close();
}

//called continously in loop()
//recieves one character from the GPS serial link and parses the full sentence when it is recieved
void readAndParseGPS() {
   //read data from the GPS in the 'main loop'
  char c = GPS.read();
   //if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

//function called in loop() to write the data string to SD card
void writeToSD() {
  // open the file
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(outputString);
    if(SERIALLOGGING) {
      Serial.print("Printed to ");
      Serial.print(filename);
      Serial.print(" : ");
      Serial.println(outputString);
    }
    digitalWrite(YELLOWLED, HIGH);
  }
  // if the file isn't open, pop up an error:
  else {
    if(SERIALLOGGING) {
      Serial.print("error opening ");
      Serial.println(filename);
    }
    digitalWrite(YELLOWLED, LOW);
  }
  dataFile.close();
}

//called once in setup() to align RTC time to GPS time
void alignDateTime(DateTime currentTime) {
 if(GPS.fix == 1) { rtc.adjust(DateTime(GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds)); }
}

//function called in loop() to append the RTC date and time
//format is: MM/DD/YYYY, HH:MM:SS, 
void timeString(DateTime currentTime) {
  outputString.concat(currentTime.month());
  outputString.concat("/");
  outputString.concat(currentTime.day());
  outputString.concat("/");
  outputString.concat(currentTime.year());
  outputString.concat(", ");
  if (currentTime.hour() < 10) { outputString.concat("0"); }
  outputString.concat(currentTime.hour());
  outputString.concat(":");
  if (currentTime.minute() < 10) { outputString.concat("0"); }
  outputString.concat(currentTime.minute());
  outputString.concat(":");
  if (currentTime.second() < 10) { outputString.concat("0"); }
  outputString.concat(currentTime.second());
  outputString.concat(", ");
}

//function called in loop() to append the GPS coordinates and whether there is a GPS fix
//format is: (Latitude decimal degrees) XXX.XXXXX, (Longitude decimal degrees) XXX.XXXXX, (BOOL) X, 
void gpsString() {
  if(GPS.fix) {
    outputString.concat(String(GPS.latitudeDegrees, 5));
    outputString.concat(", ");
    outputString.concat(String(GPS.longitudeDegrees, 5));
    digitalWrite(REDLED, HIGH);
  }
  //append dummy data if there is no GPS fix
  else {
    outputString.concat("0, 0");
    digitalWrite(REDLED, LOW);
  }

  outputString.concat(", ");
  outputString.concat(GPS.fix);
}

//function called in loop() to append the analog reading from the force sensor
void pressureString() {
  outputString.concat(", ");
  outputString.concat(analogRead(PRESSUREPIN));
}

//function called in loop() to append the analog reading from the water intrustion sensor
void waterIntrusionString() {
  int waterLevel = analogRead(WATERPIN);
  
  //turn on/off the LED as appropriate
  if(waterLevel > 0) { digitalWrite(BLUELED, HIGH); }
  else { digitalWrite(BLUELED, LOW); }
  
  outputString.concat(", ");
  outputString.concat(waterLevel);
}

//function called in loop() to append the calibration status from the orientation sensors
//format is: (accelerometer) 0/1/2/3, (gyroscope) 0/1/2/3, (magnetometer) 0/1/2/3, (overall) 0/1/2/3, 
//full calibration is when all are 3
void calibrationString() {
  uint8_t accel, gyro, magne, sys = 0;
  bno.getCalibration(&sys, &gyro, &accel, &magne);
  
  outputString.concat(", ");
  outputString.concat(int(accel));
  outputString.concat(", ");
  outputString.concat(int(gyro));
  outputString.concat(", ");
  outputString.concat(int(magne));
  outputString.concat(", ");
  outputString.concat(int(sys));
  outputString.concat(", ");

  //turn on/off the LED as appropriate
  if((int(accel) == 3) && (int(gyro) == 3) && (int(magne) == 3)) { digitalWrite(GREENLED, HIGH); }
  else { digitalWrite(GREENLED, LOW); }
}

//function called in loop() to append the orientation as a quaternion (a+bi+cj+dk)
//format is: a, b, c, d, 
void quatString() {
  imu::Quaternion quat = bno.getQuat();
  outputString.concat(quat.w());
  outputString.concat(", ");
  outputString.concat(quat.x());
  outputString.concat(", ");
  outputString.concat(quat.y());
  outputString.concat(", ");
  outputString.concat(quat.z());
  outputString.concat(", ");
}

//function called in loop() to append the orientation as euler angles (in degrees)
//format is: x, y, z, 
void eulerString() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  outputString.concat(euler.x());
  outputString.concat(", ");
  outputString.concat(euler.y());
  outputString.concat(", ");
  outputString.concat(euler.z());
  outputString.concat(", ");
}

//function called in loop() to append the barometric pressure readings, calculated altitude, and internal temperature
//format is: pressure, altitude, temperature 
void baroString() {
  if(!mpl.begin()) { outputString.concat("0, 0, 0"); }
  else {
    outputString.concat(mpl.getPressure());
    outputString.concat(", ");
    outputString.concat(mpl.getAltitude());
    outputString.concat(", ");
    outputString.concat(mpl.getTemperature());
  }
}
