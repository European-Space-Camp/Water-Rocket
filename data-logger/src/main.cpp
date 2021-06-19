
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"
#include "MPU9250.h"
#include <SD.h>

#define SERIAL_CONNECT false

#define sampleRate 0   // ms

#if SERIAL_CONNECT
  #define SERIAL_PRINT(x) Serial.println(x)
#else
  #define SERIAL_PRINT(x)
#endif

MPU9250 mpu;
Adafruit_BMP280 bme;

double groundPressure; //hPa
char filename[16];


void writeToFile(const String& dataString);
  
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);

  #if SERIAL_CONNECT
    Serial.begin(9600);
    while(!Serial);
    Serial.println("SETTING UP\n");
  #endif
  
  if (!bme.begin()) {  
    SERIAL_PRINT("Could not find a valid BMP280 sensor, check adress!");
    while (1);
  }
  if (!mpu.begin()){
    SERIAL_PRINT("Could not find a valid IMU9250 sensor, check adress!");
    while (1);
  }

  mpu.set_accel_range(RANGE_2G);
  mpu.set_gyro_range(RANGE_GYRO_250);

  groundPressure = bme.readPressure()/100;

  if (!SD.begin(SS1)) {
    SERIAL_PRINT("Card failed, or not present");
    while(1);
  }
  SERIAL_PRINT("card initialized.");

  int n = 0;
  snprintf(filename, sizeof(filename), "data%03d.txt", n);
  while (SD.exists(filename)) {
    n++;
    snprintf(filename, sizeof(filename), "data%03d.txt", n);
  }

  writeToFile("Pressure(Pa) Alt(m) Accel(G)-X-Y-Z Gyro(º/s)-X-Y-Z Temperature(*C)-bme-imu");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  String dataString;
  dataString += String(millis()/1000.0);
  dataString += " ";
  dataString += String(bme.readPressure());
  dataString += " ";
  dataString += String(bme.readAltitude(groundPressure));
  dataString += " ";
  
  mpu.get_accel_g();
  dataString += String(mpu.x_g,2);
  dataString += " ";
  dataString += String(mpu.y_g,2);
  dataString += " ";
  dataString += String(mpu.z_g,2);
  dataString += " ";

  mpu.get_gyro_d();
  dataString += String(mpu.gx_d,2);
  dataString += " ";
  dataString += String(mpu.gy_d,2);
  dataString += " ";
  dataString += String(mpu.gz_d,2);
  dataString += " ";
  dataString += String(bme.readTemperature());
  dataString += " ";
  dataString += String(mpu.get_temp() / 333.87 + 21.0,1);

  writeToFile(dataString);

  delay(sampleRate);
}

void writeToFile(const String& dataString){
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    SERIAL_PRINT(dataString);
  }
  else {
    SERIAL_PRINT("error opening datalog.txt");
    digitalWrite(LED_BUILTIN, HIGH);
  }
} 