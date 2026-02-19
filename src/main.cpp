#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// PINS
static const int AIN1_PIN = 25; // forward
static const int AIN2_PIN = 26; // reverse
static const int PWMA_PIN = 27; // speed
static const int STBY_PIN = 14; // enable driver
static const int MC1_S_A_PIN = 32;
static const int MC1_S_B_PIN = 33;
static const int MC2_S_A_PIN = 16; // RX2
static const int MC2_S_B_PIN = 17; // TX2
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
static const int SERVO1_PIN = 18;
static const int SERVO2_PIN = 19;

// ADDRESSES
static const int IMU_ADR = 104;
static const int OLED_ADR = 60;

// PWM SETTINGS
static const int PWM_CH = 0;
static const int PWM_FREQ = 2000;
static const int PWM_RES = 8; // 8-bit (0-255), speed vals

ICM20948_WE imu = ICM20948_WE(IMU_ADR);
Adafruit_SSD1306 oled(128, 64, &Wire, -1); // width, height

// MOTOR FUNCTIONS
void motorStop() {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
    ledcWrite(PWM_CH, 0); // set speed
}

void motorForward(uint8_t speed) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    ledcWrite(PWM_CH, speed); // set speed
}

void motorReverse(uint8_t speed) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    ledcWrite(PWM_CH, speed); // set speed
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // faster i2c

    // check oled
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADR)) {
        while (1) { Serial.println("OLED failed"); delay(2000); }
    }

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.println("OLED ready");
    oled.display();

    // check imu
    if (!imu.init()) {
        while (1) { Serial.println("IMU failed"); delay(2000); }
    }

    oled.setCursor(0, 10);
    oled.println("IMU ready");
    oled.display();

    // setup servos
    Servo servo1; servo1.attach(SERVO1_PIN); servo1.write(0); delay(500);
    Servo servo2; servo2.attach(SERVO2_PIN); servo2.write(0); delay(500);

    // setup motors
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWMA_PIN, PWM_CH);
    digitalWrite(STBY_PIN, HIGH);
    motorStop();

    // need to setup 2nd motor...
}

void loop() {

}


