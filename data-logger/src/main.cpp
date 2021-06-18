
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include <SD.h>

MPU9250 mpu;
Adafruit_BMP280 bme;

double groundPressure; //hPa
char filename[16];

void writeToFile(const String& dataString);
  
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);

  // Serial.begin(9600);
  // while(!Serial);
  // Serial.println("SETTING UP\n");
  
  if (!bme.begin()) {  
    // Serial.println("Could not find a valid BMP280 sensor, check adress!");
    while (1);
  }
  if (!mpu.begin()){
    // Serial.println("Could not find a valid IMU9250 sensor, check adress!");
    while (1);
  }

  mpu.set_accel_range(RANGE_2G);
  mpu.set_gyro_range(RANGE_GYRO_250);

  groundPressure = bme.readPressure()/100;

  if (!SD.begin(SS1)) {
    // Serial.println("Card failed, or not present");
    while(1);
  }
  // Serial.println("card initialized.");

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

  delay(1000);
}

void writeToFile(const String& dataString){
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    // Serial.println(dataString);
  }
  else {
    // Serial.println("error opening datalog.txt");
  }
}

  
    //Mag not working

    // mpu.set_mag_scale(SCALE_14_BITS);
    // mpu.set_mag_speed(MAG_8_Hz);
    // if(!mpu.get_mag()){
    //   Serial.print("MX: ");  Serial.print(mpu.mx); 
    //   Serial.print(" MY: "); Serial.print(mpu.my); 
    //   Serial.print(" MZ: "); Serial.print(mpu.mz);

    //   mpu.get_mag_t();
    //   Serial.print(" MX_t: "); Serial.print(mpu.mx_t,2); 
    //   Serial.print(" MY_t: "); Serial.print(mpu.my_t,2); 
    //   Serial.print(" MZ_t: "); Serial.print(mpu.mz_t,2); Serial.println(" uT");
    // }
    // else{
    //   // |X|+|Y|+|Z| must be < 4912μT to sensor measure correctly 
    //   Serial.println("Overflow no magnetometro.");
    // }    