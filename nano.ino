#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <math.h>
 
#define BAUD_RATE 115200
#define BNO055_SAMPLERATE_DELAY_MS 100
 
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP085 bmp = Adafruit_BMP085();

int lineCounter = 0;

void printCalibrationVal() {
  uint8_t accel, gyro, mag, sys = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(accel);
  Serial.print(',');
  Serial.print(gyro);
  Serial.print(',');
  Serial.print(mag);
  Serial.print(',');
  Serial.print(sys);
  Serial.print(',');
}

void printQuaternion() {
  imu::Quaternion quat = bno.getQuat();
  Serial.print(quat.w(), 4);
  Serial.print(',');
  Serial.print(quat.x(), 4);
  Serial.print(',');
  Serial.print(quat.y(), 4);
  Serial.print(',');
  Serial.print(quat.z(), 4);
  Serial.print(',');
}

void printEulerAngle() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(euler.x());
  Serial.print(',');
  Serial.print(euler.y());
  Serial.print(',');
  // Serial.println(euler.z());
  Serial.print(euler.z());
  Serial.print(',');
}

void printPressure() {
  Serial.print(bmp.readPressure()); //(Pa)
  // Serial.print(bmp.readAltitude(bmp.readPressure())); //(m)
  Serial.print(',');
}

void printLineNum() {
  lineCounter++;
  Serial.println(lineCounter);
}
 
void setup() {
  Serial.begin(BAUD_RATE);
  bno.begin();
  bmp.begin();
  delay(1000);
  //int8_t temp=bno.getTemp();
  bno.setExtCrystalUse(true);
}
 
void loop() {
  printCalibrationVal();
  printQuaternion();
  printEulerAngle();
  printPressure();
  printLineNum();
   
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
