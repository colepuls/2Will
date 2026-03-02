#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include <Bluepad32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>


// Adresses
static const uint8_t bnoAdr = 0x28;
static const uint8_t oledAdr = 0x3C;


// Pins

// bno & oled
static const int sdaPin = 21;
static const int sclPin = 22;

// motor controller
static const int ain1Pin = 25; // forward
static const int ain2Pin = 26; // reverse
static const int pwmPin = 27;
static const int stbyPin = 14;
// A01 - red wire left motor 
// A02 - black wire left motor 


// Pwm settings
static const int pwmCh = 0;
static const int pwmFreq = 20000;
static const int pwmRes = 8; // 0 - 255


// Objects

// bno
Adafruit_BNO055 bno(55, bnoAdr);

// oled
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// xbox controller
GamepadPtr gp;
void onConnectedGamepad(GamepadPtr g) { gp = g; }
void onDisconnectedGamepad(GamepadPtr g) { gp = nullptr; }
struct ControllerState {
    // sticks
    int16_t leftX, leftY;
    int16_t rightX, rightY;

    // triggers
    int16_t leftTrigger;
    int16_t rightTrigger;

    // dpad
    bool dpadUp, dpadDown, dpadLeft, dpadRight;

    // buttons
    bool a, b, x, y;
    bool lb, rb;
    bool thumbL, thumbR;
    bool start, select, home;
};
bool readController(ControllerState &s) {
    if (!gp || !gp->isConnected()) {
        return false;
    }

    // sticks
    s.leftX = gp->axisX();
    s.leftY = gp->axisY();
    s.rightX = gp->axisRX();
    s.rightY = gp->axisRY();

    // triggers
    s.leftTrigger = gp->brake();
    s.rightTrigger = gp->throttle();

    // dpad
    uint8_t dpad = gp->dpad();
    s.dpadUp = dpad & DPAD_UP;
    s.dpadDown = dpad & DPAD_DOWN;
    s.dpadLeft = dpad & DPAD_LEFT;
    s.dpadRight = dpad & DPAD_RIGHT;

    // buttons
    uint8_t btn = gp->buttons();
    s.a = btn & BUTTON_A;
    s.b = btn & BUTTON_B;
    s.x = btn & BUTTON_X;
    s.y = btn & BUTTON_Y;
    s.lb = btn & BUTTON_SHOULDER_L;
    s.rb = btn & BUTTON_SHOULDER_R;
    s.thumbL = btn & BUTTON_THUMB_L;
    s.thumbR = btn & BUTTON_THUMB_R;

    // misc buttons
    uint8_t misc = gp->miscButtons();
    s.select = misc & MISC_BUTTON_SELECT;
    s.start = misc & MISC_BUTTON_START;
    s.home = misc & MISC_BUTTON_SYSTEM;

    
    return true;
}
ControllerState ctrl;


// Motor
void motorStop() {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, LOW);
    ledcWrite(pwmCh, 0); // set speed
}
void motorForward(uint8_t speed) {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(ain2Pin, LOW);
    ledcWrite(pwmCh, speed); // set speed    
}
void motorReverse(uint8_t speed) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH);
    ledcWrite(pwmCh, speed); // set speed    
}

void setup() {
    // serial
    Serial.begin(115200);
    delay(500);

    // i2c
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);

    // bno setup
    if (!bno.begin()) {
        Serial.println("BNO not found.\n");
        while (true);
    }
    bno.setExtCrystalUse(true);
    Serial.println("BNO ready.\n");

    // oled
    if (!oled.begin(SSD1306_SWITCHCAPVCC, oledAdr)) {
        Serial.println("OLED not found.\n");
        while (true);
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    Serial.println("OLED ready.\n");

    // motor
    pinMode(ain1Pin, OUTPUT);
    pinMode(ain2Pin, OUTPUT);
    pinMode(stbyPin, OUTPUT);
    ledcSetup(pwmCh, pwmFreq, pwmRes);
    ledcAttachPin(pwmPin, pwmCh);
    digitalWrite(stbyPin, HIGH);
    motorStop();
    Serial.println("Motor ready.\n");

    // xbox controller
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
}


void loop () {
    BP32.update();
    bool hasCtrl = readController(ctrl);
    bool ctrlActive = hasCtrl && (ctrl.leftTrigger  > 200 ||ctrl.rightTrigger > 200);

    if (ctrlActive) {

        oled.clearDisplay();

        if (ctrl.leftTrigger > 200) {
            // reverse
            oled.setCursor(0, 0);
            oled.println("Controller reverse.");
            oled.display();
            motorReverse(255);
        }
        
        else if (ctrl.rightTrigger > 200) {
            // forward
            oled.setCursor(0, 0);
            oled.println("Controller forward.");
            oled.display();
            motorForward(255);
        }

        else {
            oled.setCursor(0, 0);
            oled.println("Motor stop.");
            oled.display();
            motorStop();
        }
    }
    else {
        // imu code
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        float pitch = euler.z();
        Serial.printf("Pitch: %.2f\n", pitch);

        oled.clearDisplay();

        if (pitch <= 10 && pitch >= -10) {
            motorStop();
            oled.setCursor(0, 0);
            oled.println("Motor stop.");
            oled.display();
            delay(50);
        }
        else if (pitch <= 20 && pitch > 10) {
            oled.setCursor(0, 0);
            oled.println("Imu forward.");
            oled.display();
            motorForward(100);
            delay(50);
        }
        else if (pitch <= 30 && pitch > 20) {
            oled.setCursor(0, 0);
            oled.println("Imu forward.");
            oled.display();
            motorForward(150);
            delay(50);
        }
        else if (pitch <= 40 && pitch > 30) {
            oled.setCursor(0, 0);
            oled.println("Imu forward.");
            oled.display();
            motorForward(200);
            delay(50);
        }
        else if (pitch > 40) {
            oled.setCursor(0, 0);
            oled.println("Imu forward.");
            oled.display();
            motorForward(255);   // max speed
            delay(50);
        }
        else if (pitch >= -20 && pitch < -10) {
            oled.setCursor(0, 0);
            oled.println("Imu reverse.");
            oled.display();
            motorReverse(100);
            delay(50);
        }
        else if (pitch >= -30 && pitch < -20) {
            oled.setCursor(0, 0);
            oled.println("Imu reverse.");
            oled.display();
            motorReverse(150);
            delay(50);
        }
        else if (pitch >= -40 && pitch < -30) {
            oled.setCursor(0, 0);
            oled.println("Imu reverse.");
            oled.display();
            motorReverse(200);
            delay(50);
        }
        else if (pitch < -40) {
            oled.setCursor(0, 0);
            oled.println("Imu reverse.");
            oled.display();
            motorReverse(255);  // max speed
            delay(50);
        }
    }
}




