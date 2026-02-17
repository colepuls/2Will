#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_SSD1306.h>

// PINS
static const int AIN1_PIN = 25; // forward
static const int AIN2_PIN = 26; // reverse
static const int PWMA_PIN = 27; // speed
static const int STBY_PIN = 14; // enable driver
//static const int MC_S_A_PIN = 32;
//static const int MC_S_B_PIN = 33;
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
// Servo: 18, 19

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

    // setup motor
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWMA_PIN, PWM_CH);
    digitalWrite(STBY_PIN, HIGH);
    motorStop();
}

void loop() {
/*
  imu.readSensor();


  xyzFloat acc, gyr;
  imu.getGValues(&acc);
  imu.getGyrValues(&gyr);


  static uint32_t lastMicros = micros();
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;


  float pitchAcc = atan2(acc.x, acc.z) * 180.0f / PI;

  static float pitch = 0.0f;
  float pitchRate = gyr.y;              

  const float alpha = 0.98f;
  pitch = alpha * (pitch + pitchRate * dt) + (1.0f - alpha) * pitchAcc;

  float target = 0.0f;
  float error = target - pitch;

  float Kp = 20.0f;                    
  float u = Kp * error;                 

  if (fabs(pitch) > 35.0f) {
    motorStop();
  } else {
    int speed = constrain((int)fabs(u), 0, 255);

    if (speed < 25) {
      motorStop();
    } else {
      if (u > 0) motorForward(speed);
      else       motorReverse(speed);
    }
  }

  oled.clearDisplay();
  oled.setCursor(0, 0);

  oled.printf("Ax %.2f Ay %.2f\n", acc.x, acc.y);
  oled.printf("Az %.2f\n", acc.z);
  oled.printf("Gy %.2f dt %.3f\n", pitchRate, dt);

  oled.printf("pAcc %.1f\n", pitchAcc);
  oled.printf("p %.1f\n", pitch);
  oled.printf("err %.1f\n", error);
  oled.printf("u %.1f\n", u);

  oled.display();

  delay(20);
*/
  motorForward(255);  // 0-255
  delay(1000);

  motorStop();
  delay(1000);

  motorReverse(255); // 0-255
  delay(1000);

  motorStop();
  delay(1000);
}


