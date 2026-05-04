#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include <stdint.h>
#include <math.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// ================= PINS =================
static const u16 SLPIN = 18;
static const u16 SRPIN = 19;
static const u16 SDAPIN = 21;
static const u16 SCLPIN = 22;

static const u16 AIN1PIN = 25;
static const u16 AIN2PIN = 26;
static const u16 PWMAPIN = 27;

static const u16 BIN1PIN = 13;
static const u16 BIN2PIN = 15;
static const u16 PWMBPIN = 5;

static const u16 STBYPIN = 14;

// ================= SERVO POSITIONS =================
static const int slCrouch = 180;
static const int slStand  = 150;
static const int srCrouch = 10;
static const int srStand  = 50;

// ================= PWM =================
static const u8 pwmaCh = 6;
static const u8 pwmbCh = 7;
static const u32 pwmFreq = 20000;
static const u8 pwmRes = 8;

// ================= IMU =================
static const u8 bnoAdr = 0x28;
static const u16 bnoModel = 55;

// ================= OBJECTS =================
Servo sl;
Servo sr;
Adafruit_BNO055 bno(bnoModel, bnoAdr);
GamepadPtr gamepad;

// ================= CONTROLLER =================
struct ControllerState {
  bool a, b, x, y;
};

ControllerState xboxCtrl;

// ================= EDGE DETECTION =================
bool lastA = false;
bool lastB = false;
bool lastX = false;
bool lastY = false;

bool leftLegState = false;
bool rightLegState = false;

bool posState = true; // true: stand, false: crouch

// =====================================================
//                    PID LOOP
// =====================================================
//
static float standingTargetPitch = 4.85f;
static float crouchingTargetPitch = 1.3f;
static float targetPitchDeg = standingTargetPitch; // starts in standing target

static const float motorSign = -1.0f; // wheel spin direction

static float standingKp = 38.0f;
static float standingKd = 0.75f;

static float crouchingKp = 32.0f;
static float crouchingKd = 0.70f;


// main tuning values
static float balanceKp = standingKp; // looks at error angle only, main correction strength
static float balanceKi = 0.0f; // long term bias
static float balanceKd = standingKd; // looks at how fast error is changing, smoothing, prevent overshooting, damping

void setPidVals(float Kp, float Kd) {
  balanceKp = Kp;
  balanceKd = Kd;
}

// ================= MOTOR OUTPUT SHAPING =================
static const float outDeadband = 5.0f; // ignored tiny pid outputs
static const int minBalancePwm = 25; // minimum motor power once the PID decides the motors should move
static const int maxBalancePwm = 255; // maximum motor power

// safety cutoff
static const float fallAngle = 38.0f;

// ================= PID STATE =================
static float balanceIntegral = 0.0f;
static float balancePrevErr = 0.0f;

// ================= IMU STATE =================
static float pitchFiltered = 0.0f;

// ================= TIMING =================
static u32 lastUs = 0;
static u32 lastPrintMs = 0;

// ================= HELPERS =================
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline int clampi(int x, int lo, int hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static int makeMotorPwm(float motorOut) {
  if (fabsf(motorOut) < outDeadband) {
    return 0;
  }

  int pwm = (int)fabsf(motorOut);

  if (pwm > 0 && pwm < minBalancePwm) {
    pwm = minBalancePwm;
  }

  return clampi(pwm, 0, maxBalancePwm);
}

static void resetPid() {
  balanceIntegral = 0.0f;
  balancePrevErr = 0.0f;
}

// ================= MOTOR =================
void motorStop(int m) {
  if (m == 0) {
    digitalWrite(AIN1PIN, LOW);
    digitalWrite(AIN2PIN, LOW);
    ledcWrite(pwmaCh, 0);
  } else {
    digitalWrite(BIN1PIN, LOW);
    digitalWrite(BIN2PIN, LOW);
    ledcWrite(pwmbCh, 0);
  }
}

void motorForward(int m, u8 speed) {
  if (m == 0) {
    digitalWrite(AIN1PIN, HIGH);
    digitalWrite(AIN2PIN, LOW);
    ledcWrite(pwmaCh, speed);
  } else {
    digitalWrite(BIN1PIN, LOW);
    digitalWrite(BIN2PIN, HIGH);
    ledcWrite(pwmbCh, speed);
  }
}

void motorReverse(int m, u8 speed) {
  if (m == 0) {
    digitalWrite(AIN1PIN, LOW);
    digitalWrite(AIN2PIN, HIGH);
    ledcWrite(pwmaCh, speed);
  } else {
    digitalWrite(BIN1PIN, HIGH);
    digitalWrite(BIN2PIN, LOW);
    ledcWrite(pwmbCh, speed);
  }
}

void stopBothMotors() {
  motorStop(0);
  motorStop(1);
}

// ================= CONTROLLER =================
void onConnected(GamepadPtr gp) {
  gamepad = gp;
}

void onDisconnected(GamepadPtr gp) {
  gamepad = nullptr;
}

bool readController(ControllerState& s) {
  if (!gamepad || !gamepad->isConnected()) {
    return false;
  }

  const uint8_t btn = gamepad->buttons();

  s.a = btn & BUTTON_A;
  s.b = btn & BUTTON_B;
  s.x = btn & BUTTON_X;
  s.y = btn & BUTTON_Y;

  return true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDAPIN, SCLPIN);

  if (!bno.begin()) {
    Serial.println("BNO055 NOT DETECTED");
  }

  delay(500);
  bno.setExtCrystalUse(true);
  delay(100);

  //  starts in standing position
  targetPitchDeg = standingTargetPitch;
  posState = true;

  pitchFiltered = targetPitchDeg;
  resetPid();

  // ===== SERVOS =====
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  sl.setPeriodHertz(50);
  sr.setPeriodHertz(50);

  sl.attach(SLPIN, 500, 2400);
  sr.attach(SRPIN, 500, 2400);

  sl.write(slStand);
  sr.write(srStand);

  // ===== MOTORS =====
  pinMode(AIN1PIN, OUTPUT);
  pinMode(AIN2PIN, OUTPUT);
  pinMode(BIN1PIN, OUTPUT);
  pinMode(BIN2PIN, OUTPUT);
  pinMode(STBYPIN, OUTPUT);

  digitalWrite(STBYPIN, HIGH);

  ledcSetup(pwmaCh, pwmFreq, pwmRes);
  ledcSetup(pwmbCh, pwmFreq, pwmRes);

  ledcAttachPin(PWMAPIN, pwmaCh);
  ledcAttachPin(PWMBPIN, pwmbCh);

  stopBothMotors();

  BP32.setup(&onConnected, &onDisconnected);

  lastUs = micros();

  Serial.println("Ready");
  Serial.print("Target pitch: ");
  Serial.println(targetPitchDeg);
}

// ================= LOOP =================
void loop() {
  BP32.update();

  u32 nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.030f) dt = 0.030f;

  // ================= IMU =================
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float rawPitch = euler.y();

  // Light smoothing to reduce twitching.
  pitchFiltered = 0.80f * pitchFiltered + 0.20f * rawPitch; // pitchFiltered = 0.85f * pitchFiltered + 0.15f * rawPitch; -> more smooth, slower
                                                            // pitchFiltered = 0.70f * pitchFiltered + 0.30f * rawPitch; -> more sensitive, faster
  float pitch = pitchFiltered;

  // ================= CONTROLLER SERVO CONTROL ONLY =================
  if (readController(xboxCtrl)) {
    // B = both crouch
    if (xboxCtrl.b && !lastB) {
      sl.write(slCrouch);
      sr.write(srCrouch);
      leftLegState = true;
      rightLegState = true;
    }

    // A = both stand
    if (xboxCtrl.a && !lastA) {
      sl.write(slStand);
      sr.write(srStand);
      leftLegState = false;
      rightLegState = false;
    }

    // X = toggle left leg
    if (xboxCtrl.x && !lastX) {
      leftLegState = !leftLegState;
      sl.write(leftLegState ? slCrouch : slStand);
    }

    // Y = toggle right leg
    if (xboxCtrl.y && !lastY) {
      rightLegState = !rightLegState;
      sr.write(rightLegState ? srCrouch : srStand);
    }

    lastA = xboxCtrl.a;
    lastB = xboxCtrl.b;
    lastX = xboxCtrl.x;
    lastY = xboxCtrl.y;
  } else {
    lastA = false;
    lastB = false;
    lastX = false;
    lastY = false;
  }

  // ================= TARGET PITCH SWITCH =================
  if (leftLegState && rightLegState) {
    targetPitchDeg = crouchingTargetPitch;
    setPidVals(crouchingKp, crouchingKd);
    posState = false;
  } else {
    targetPitchDeg = standingTargetPitch;
    setPidVals(standingKp, standingKd);
    posState = true;
  }

  // ================= FALL SAFETY =================
  if (fabsf(pitch - targetPitchDeg) > fallAngle) {
    stopBothMotors();
    resetPid();

    if (millis() - lastPrintMs > 250) {
      lastPrintMs = millis();
      Serial.print("FALL CUTOFF | pitch=");
      Serial.print(pitch);
      Serial.print(" target=");
      Serial.println(targetPitchDeg);
    }

    return;
  }

  // =====================================================
  //                  PID LOOP
  // =====================================================

  float error = pitch - targetPitchDeg;

  balanceIntegral += error * dt;
  balanceIntegral = clampf(balanceIntegral, -20.0f, 20.0f);

  float derivative = (error - balancePrevErr) / dt;
  balancePrevErr = error;

  derivative = clampf(derivative, -250.0f, 250.0f);

  float pid =
      (balanceKp * error) +
      (balanceKi * balanceIntegral) +
      (balanceKd * derivative);

  float motorOut = motorSign * pid;

  int basePwm = makeMotorPwm(motorOut);

  // ================= MOTOR DIRECTION =================
  if (basePwm == 0) {
    stopBothMotors();
  } else if (motorOut > 0) {
    motorForward(0, basePwm);
    motorForward(1, basePwm);
  } else {
    motorReverse(0, basePwm);
    motorReverse(1, basePwm);
  }

  // ================= DEBUG =================
  if (millis() - lastPrintMs > 100) {
    lastPrintMs = millis();

    Serial.print("pitch=");
    Serial.print(pitch);

    Serial.print(" target=");
    Serial.print(targetPitchDeg);

    Serial.print(" error=");
    Serial.print(error);

    Serial.print(" P=");
    Serial.print(balanceKp * error);

    Serial.print(" I=");
    Serial.print(balanceKi * balanceIntegral);

    Serial.print(" D=");
    Serial.print(balanceKd * derivative);

    Serial.print(" out=");
    Serial.print(motorOut);

    Serial.print(" pwm=");
    Serial.print(basePwm);

    Serial.print(" pos=");
    Serial.println(posState ? "standing" : "crouching");
  }
}