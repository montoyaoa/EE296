#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//GPS TX on Digital 8
//GPS RX on Digital 7
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial link established.");

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);

}

//milliseconds since the program started running.
//start timer after initialization
uint32_t timer = millis();

void loop() {
char c = GPS.read();

// if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }


  
  //this only updates every one second
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.print(GPS.milliseconds);
    Serial.print(", ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.print(GPS.year, DEC);

    Serial.print(", ");
    //GPS.latitude or GPS.longitude give the numerical value
    //GPS.lat or GPS.long gives the N/S/E/W letter
    //GPS.latitudeDegrees and GPS.longitudeDegrees gives the degrees in a signed float
    Serial.print(GPS.latitudeDegrees, 5);
    Serial.print(", ");
    Serial.println(GPS.longitudeDegrees, 5);

  }
}
