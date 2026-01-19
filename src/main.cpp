#include <Arduino.h>
#include <Wire.h> // i2c communication
#include "ICM20948.h" // imu driver

static ICM20948 imu; // imu object, talk to sensor over i2c

void print(string text);

void setup()
{
  Serial.begin(115200);
  delay(300);

  print("IMU test starting");
  Wire.being(21, 22); // starts i2c, gpio 21 = sda, gpio 22 = scl
  imu.initialize(); //
  if (!imu.testConnection)
  {
    print("IMU not found");
    while (true); // freeze program
  }
  print("IMU connected")
}

void loop()
{
  // ...
}

void print(string text)
{
  Serial.println(text);
}