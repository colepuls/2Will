#include "imu.h"

static const int IMU_ADR = 0x68;
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
static const int S_WIDTH = 128;
static const int S_HEIGHT = 64;
static const int OLED_ADR = 0x3C;

ICM20948_WE imu = ICM20948_WE(IMU_ADR);
Adafruit_SSD1306 oled(S_WIDTH, S_HEIGHT, &Wire, -1);

void imSetup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN); // sda, scl
  Wire.setClock(400000); // faster i2c

  // check oled
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADR)) {
    while (1) {Serial.println("OLED failed"); delay(2000);}
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println("OLED ready");
  oled.display();

  // check imu
  if (!imu.init()) {
    while (1) {Serial.println("IMU failed"); delay(2000);}
  }
  oled.setCursor(0, 10);
  oled.println("IMU ready");
  oled.display();
}


void imuLoop()
{
  imu.readSensor(); // update

  xyzFloat acc, gyr;
  imu.getGValues(&acc);
  imu.getGyrValues(&gyr);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.printf("Ax %.2f\nAy %.2f\nAz %.2f\n", acc.x, acc.y, acc.z);
  oled.printf("Gx %.2f\nGy %.2f\nGz %.2f\n", gyr.x, gyr.y, gyr.z);
  oled.display();

  delay(100);
}