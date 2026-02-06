#include "dcmotor.h"

// pin setup
static const int AIN1_PIN = 25; // forward
static const int AIN2_PIN = 26; // reverse
static const int PWMA_PIN = 27; // speed
static const int STBY_PIN = 14; // enable driver

// PWM settings
static const int PWM_CH = 0; // channel 0
static const int PWM_FREQ = 20000; // 20 khz
static const int PWM_RES = 8; // 8-bit: 0-255, speed values

void motorStop() {
  digitalWrite(AIN1_PIN, LOW); // 0
  digitalWrite(AIN2_PIN, LOW); // 0
  ledcWrite(PWM_CH, 0); // motor, 0 (speed)
}

void motorForward(uint8_t speed) {
  digitalWrite(AIN1_PIN, HIGH); // 1 - forward
  digitalWrite(AIN2_PIN, LOW); // 0
  ledcWrite(PWM_CH, speed); // motor, speed
}

void motorReverse(uint8_t speed) {
  digitalWrite(AIN1_PIN, LOW); // 0
  digitalWrite(AIN2_PIN, HIGH); // 1 - reverse
  ledcWrite(PWM_CH, speed);
}

void motorSetup() {
  // set pins
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  // setup PWM on PWMA
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA_PIN, PWM_CH);

  digitalWrite(STBY_PIN, HIGH); // enable motor driver
  motorStop();
}

void motorLoop() {
  motorForward(255);  // 0-255
  delay(1000);

  motorStop();
  delay(1000);

  motorReverse(255); // 0-255
  delay(1000);

  motorStop();
  delay(1000);
}

