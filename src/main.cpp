#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDR 0x3C
#define SDA 21
#define SCL 22

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // oled object

const int TRIG = 19;
const int ECHO = 18;

float readDistanceCm() 
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 25000UL);
  if (duration == 0) return 9999;
  return ((duration * 0.0343f) / 2.0f) / 2.54f; // return distance
}

void setup() {

  pinMode(TRIG, OUTPUT); // sender
  pinMode(ECHO, INPUT); // reciever
  digitalWrite(TRIG, LOW);

  Wire.begin(SDA, SCL);
  oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR);
  oled.setTextSize(3);
  oled.setTextColor(SSD1306_WHITE);
  oled.clearDisplay();

  while (true) {
    float cm = readDistanceCm();
    oled.setCursor(0, 0);
    oled.print(cm);
    oled.display();
    delay(50);
    oled.clearDisplay();
  }
}   

void loop() {}