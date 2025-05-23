#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////
// BMP-280
////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BMP280_ADDRESS 0x76

float Temperature, Pressure, Altitude;

Adafruit_BMP280 bmp; // I2C

void setupBMPregisters()
{
  Serial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void readBMPdata()
{
  Temperature = bmp.readTemperature();
  Pressure = bmp.readPressure();
  Altitude = bmp.readAltitude(1013.25);
}

void printBMPdata()
{
  Serial.print(Temperature);
  Serial.print(",");
  Serial.print(Pressure);
  Serial.print(",");
  Serial.print(Altitude);
  Serial.print(",");
  Serial.println();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// MPU-6050
////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MPU6050_ADDRESS 0x68

const float acc_scale_factor = 4096; // From MPU6050 datasheet [g]
const float gyro_scale_factor = 65.5; // From MPU6050 datasheet [deg/s]
float AccX, AccY, AccZ, mpu_temperature, GyroX, GyroY, GyroZ; // Raw MPU data

void setupMPUregisters()
{
  // Activate the MPU-6050
  Wire.beginTransmission(MPU6050_ADDRESS); // Start communicating with the MPU-6050
  Wire.write(0x6B); // Send the requested starting register
  Wire.write(0x00); // Set the requested starting register
  Wire.endTransmission(); // End the transmission
  //Configure the accelerometer (+/- 8g)
  Wire.beginTransmission(MPU6050_ADDRESS); // Start communicating with the MPU-6050
  Wire.write(0x1C); // Send the requested starting register
  Wire.write(0x10); // Set the requested starting register
  Wire.endTransmission(); // End the transmission
  // Configure the gyroscope (500 deg/s full scale)
  Wire.beginTransmission(MPU6050_ADDRESS); // Start communicating with the MPU-6050
  Wire.write(0x1B); // Send the requested starting register
  Wire.write(0x08); // Set the requested starting register
  Wire.endTransmission(); // End the transmission
}

void readMPUdata()
{
  // Read raw gyroscope and accelerometer data
  Wire.beginTransmission(MPU6050_ADDRESS); // Start communicating with the MPU-6050
  Wire.write(0x3B); // Send the requested starting register
  Wire.endTransmission(); // End the transmission
  Wire.requestFrom(MPU6050_ADDRESS, 14, true); // Request 14 bytes from the MPU-6050
  AccX = (Wire.read() << 8 | Wire.read());
  AccY = (Wire.read() << 8 | Wire.read());
  AccZ = (Wire.read() << 8 | Wire.read());
  mpu_temperature = Wire.read() << 8 | Wire.read();
  GyroX = (Wire.read() << 8 | Wire.read());
  GyroY = (Wire.read() << 8 | Wire.read());
  GyroZ = (Wire.read() << 8 | Wire.read());
}

void printMPUdata()
{
  Serial.print(AccX);
  Serial.print(",");
  Serial.print(AccY);
  Serial.print(",");
  Serial.print(AccZ);
  Serial.print(",");
  Serial.print(GyroX);
  Serial.print(",");
  Serial.print(GyroY);
  Serial.print(",");
  Serial.print(GyroZ);
  Serial.print(",");
  Serial.println();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPS Neo 6M
////////////////////////////////////////////////////////////////////////////////////////////////////////

int RX_pin = 3;
int TX_pin = 4;

bool gpsAvailable = false;
byte gpsData;

TinyGPSPlus gps;

SoftwareSerial gpsSerial(TX_pin, RX_pin);

void setupGPSmodule() {
  gpsSerial.begin(9600);
}

void printGPSdata() {
  gpsSerial.available() > 0 ? gpsAvailable = true : gpsAvailable = false;
  while (gpsSerial.available() > 0){
    // get the byte data from the GPS
    // gpsData = gpsSerial.read();
    // Serial.write(gpsData);
    // Serial.print(gpsData);


    // Using TinyGPSPlus
    gps.encode(gpsSerial.read());

    if (gps.time.isUpdated()) {
      Serial.print("Time: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.print(gps.time.second());
      Serial.print(".");
      Serial.println(gps.time.centisecond());
    }
  }
}

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100); // wait for native usb

  // setupBMPregisters();
  // setupMPUregisters();
  setupGPSmodule();

  // Serial.print("Temperature");
  // Serial.print(",");
  // Serial.print("Pressure");
  // Serial.print(",");
  // Serial.print("Altitude");
  // Serial.print(",");
  // Serial.print("AccX");
  // Serial.print(",");
  // Serial.print("AccY");
  // Serial.print(",");
  // Serial.print("AccZ");
  // Serial.print(",");
  // Serial.print("GyroX");
  // Serial.print(",");
  // Serial.print("GyroY");
  // Serial.print(",");
  // Serial.print("GyroZ");
  // Serial.print(",");
  
  Serial.println();
}

void loop() {
  // readBMPdata();
  // readMPUdata();

  // printBMPdata();
  // printMPUdata();
  Serial.print(gpsAvailable);
  Serial.println();
  printGPSdata();

  Serial.println();
  
  delay(2000);
}
