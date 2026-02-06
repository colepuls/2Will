#include "servo.h"

static const int SERVO_PIN = 4;
Servo servo;

void servoSetup()
{
    Serial.begin(115200);
    servo.attach(SERVO_PIN);
}

void servoLoop()
{
    for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        servo.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }

    for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        servo.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }    
}