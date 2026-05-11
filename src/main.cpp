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

// pins
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

// servo angles
static const int slCrouch = 175;
static const int slStand  = 150;
static const int srCrouch = 25;
static const int srStand  = 50;

// motor pwm setup
static const u8 pwmaCh = 6;
static const u8 pwmbCh = 7;
static const u32 pwmFreq = 20000;
static const u8 pwmRes = 8;

// imu setup
static const u8 bnoAdr = 0x28;
static const u16 bnoModel = 55;

// objects
Servo sl;
Servo sr;
Adafruit_BNO055 bno(bnoModel, bnoAdr);
GamepadPtr gamepad;

// controller buttons and sticks
struct ControllerState {
  bool a, b, x, y;
  float leftX;
  float leftY;
};

ControllerState xboxCtrl;

// last button states
bool lastA = false;
bool lastB = false;
bool lastX = false;
bool lastY = false;

bool leftLegState = false;
bool rightLegState = false;

bool posState = true; // true = standing, false = crouching

static const float motorSign = -1.0f;

// left stick setup
static const int stickDeadband = 60;
static const int stickRawMax = 512;

// balance mode tuning
struct BalanceTune {
  const char* name;

  float targetPitch;

  float kp;
  float ki;
  float kd;

  float driftBrakeSign;
  float driftBrakeK;
  float driftBrakeMax;
  float driftBrakeDecay;

  float driveLeanSign;
  float maxDriveLeanDeg;
  float driveResponse;

  float turnSign;
  float maxTurnOut;
  float turnResponse;

  float fallAngle;
};

// standing values
static BalanceTune standingTune = {
  "standing",

  // targetPitch:
  4.25f,

  // kp:
  38.0f,

  // ki:
  0.0f,

  // kd:
  0.42f,

  // driftBrakeSign:
  1.0f,

  // driftBrakeK:
  0.050f,

  // driftBrakeMax:
  1.0f,

  // driftBrakeDecay:
  1.2f,

  // driveLeanSign:
  -1.0f,

  // maxDriveLeanDeg:
  5.2f,

  // driveResponse:
  22.0f,

  // turnSign:
  1.0f,

  // maxTurnOut:
  75.0f,

  // turnResponse:
  325.0f,

  // fallAngle:
  // safety cutoff
  38.0f
};

// crouching values
static BalanceTune crouchingTune = {
  "crouching",

  // targetPitch:
  2.15f,

  // kp:
  36.0f,

  // ki:
  0.0f,

  // kd:
  0.42f,

  // driftBrakeSign:
  1.0f,

  // driftBrakeK:
  0.050f,

  // driftBrakeMax:
  1.0f,

  // driftBrakeDecay:
  1.0f,

  // driveLeanSign:
  -1.0f,

  // maxDriveLeanDeg:
  3.4f,

  // driveResponse:
  22.0f,

  // turnSign:
  1.0f,

  // maxTurnOut:
  75.0f,

  // turnResponse:
  325.0f,

  // fallAngle:
  // safety cutoff
  35.0f
};

// active pid values
static float balanceKp = 0.0f;
static float balanceKi = 0.0f;
static float balanceKd = 0.0f;

// motor output limits
static const float outDeadband = 5.0f;
static const int minBalancePwm = 15;
static const int maxBalancePwm = 255;

// pid memory
static float balanceIntegral = 0.0f;
static float balancePrevErr = 0.0f;

// movement memory
static float driftBrakeBias = 0.0f;
static float driveLeanBias = 0.0f;
static float turnOutFiltered = 0.0f;
static float lastMotorOut = 0.0f;

// imu memory
static float pitchFiltered = 0.0f;
static float targetPitchDeg = 5.25f;

// timing
static u32 lastUs = 0;
static u32 lastPrintMs = 0;

// mode memory
static bool lastCrouchingMode = false;

// helper functions
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline int clampi(int x, int lo, int hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float moveTowardZero(float x, float amount) {
  if (x > amount) {
    return x - amount;
  }

  if (x < -amount) {
    return x + amount;
  }

  return 0.0f;
}

static inline float moveToward(float current, float target, float amount) {
  if (current < target) {
    current += amount;
    if (current > target) {
      current = target;
    }
  } else if (current > target) {
    current -= amount;
    if (current < target) {
      current = target;
    }
  }

  return current;
}

static float axisToUnit(int raw) {
  int mag = abs(raw);

  if (mag < stickDeadband) {
    return 0.0f;
  }

  float unit = (float)(mag - stickDeadband) / (float)(stickRawMax - stickDeadband);
  unit = clampf(unit, 0.0f, 1.0f);

  return raw > 0 ? unit : -unit;
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
  driftBrakeBias = 0.0f;
  driveLeanBias = 0.0f;
  turnOutFiltered = 0.0f;
  lastMotorOut = 0.0f;
}

// motor controls
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

static void driveMotorSigned(int motor, float output) {
  output = clampf(output, -maxBalancePwm, maxBalancePwm);

  int pwm = makeMotorPwm(output);

  if (pwm == 0) {
    motorStop(motor);
  } else if (output > 0) {
    motorForward(motor, pwm);
  } else {
    motorReverse(motor, pwm);
  }
}

// controller connection
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

  s.leftX = axisToUnit(gamepad->axisX());
  s.leftY = axisToUnit(gamepad->axisY());

  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDAPIN, SCLPIN);

  if (!bno.begin()) {
    Serial.println("BNO055 NOT DETECTED");
  }

  delay(500);
  bno.setExtCrystalUse(true);
  delay(100);

  // start standing
  targetPitchDeg = standingTune.targetPitch;
  posState = true;
  pitchFiltered = targetPitchDeg;
  resetPid();

  // servo setup
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

  // motor setup
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

void loop() {
  BP32.update();

  u32 nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.030f) dt = 0.030f;

  // read imu
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float rawPitch = euler.y();
  float yaw = euler.x();

  // smooth the pitch a little
  pitchFiltered = 0.80f * pitchFiltered + 0.20f * rawPitch;
  float pitch = pitchFiltered;

  float driveCmd = 0.0f;
  float turnCmd = 0.0f;

  // controller buttons and sticks
  if (readController(xboxCtrl)) {
    // both crouch
    if (xboxCtrl.b && !lastB) {
      sl.write(slCrouch);
      sr.write(srCrouch);
      leftLegState = true;
      rightLegState = true;
      resetPid();
    }

    // both stand
    if (xboxCtrl.a && !lastA) {
      sl.write(slStand);
      sr.write(srStand);
      leftLegState = false;
      rightLegState = false;
      resetPid();
    }

    // left leg toggle
    if (xboxCtrl.x && !lastX) {
      leftLegState = !leftLegState;
      sl.write(leftLegState ? slCrouch : slStand);
      resetPid();
    }

    // right leg toggle
    if (xboxCtrl.y && !lastY) {
      rightLegState = !rightLegState;
      sr.write(rightLegState ? srCrouch : srStand);
      resetPid();
    }

    // left stick vertical:
    // push up = forward, pull down = backward
    driveCmd = -xboxCtrl.leftY;

    // left stick horizontal:
    // left = turn left, right = turn right
    turnCmd = xboxCtrl.leftX;

    lastA = xboxCtrl.a;
    lastB = xboxCtrl.b;
    lastX = xboxCtrl.x;
    lastY = xboxCtrl.y;
  } else {
    driveCmd = 0.0f;
    turnCmd = 0.0f;

    lastA = false;
    lastB = false;
    lastX = false;
    lastY = false;
  }

  // crouching mode is when both legs are crouched
  bool crouchingMode = leftLegState && rightLegState;
  posState = !crouchingMode;

  if (crouchingMode != lastCrouchingMode) {
    resetPid();
    lastCrouchingMode = crouchingMode;
  }

  BalanceTune& tune = crouchingMode ? crouchingTune : standingTune;

  balanceKp = tune.kp;
  balanceKi = tune.ki;
  balanceKd = tune.kd;

  // push/drift recovery
  float baseError = pitch - tune.targetPitch;

  if (fabsf(lastMotorOut) > 25.0f && fabsf(baseError) < 4.0f && fabsf(driveCmd) < 0.15f) {
    driftBrakeBias += tune.driftBrakeSign * lastMotorOut * tune.driftBrakeK * dt;
    driftBrakeBias = clampf(driftBrakeBias, -tune.driftBrakeMax, tune.driftBrakeMax);
  } else {
    driftBrakeBias = moveTowardZero(driftBrakeBias, tune.driftBrakeDecay * dt);
  }

  // continual forward/backward movement
  float wantedDriveLean = tune.driveLeanSign * driveCmd * tune.maxDriveLeanDeg;
  driveLeanBias = moveToward(driveLeanBias, wantedDriveLean, tune.driveResponse * dt);

  targetPitchDeg = tune.targetPitch + driftBrakeBias + driveLeanBias;

  // stop if it falls too far
  if (fabsf(pitch - targetPitchDeg) > tune.fallAngle) {
    stopBothMotors();
    resetPid();

    if (millis() - lastPrintMs > 250) {
      lastPrintMs = millis();
      Serial.print("FALL CUTOFF | mode=");
      Serial.print(tune.name);
      Serial.print(" pitch=");
      Serial.print(pitch);
      Serial.print(" target=");
      Serial.println(targetPitchDeg);
    }

    return;
  }

  // main pid
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
  lastMotorOut = motorOut;

  int basePwm = makeMotorPwm(motorOut);

  // continual left/right turning
  float wantedTurnOut = tune.turnSign * turnCmd * tune.maxTurnOut;
  turnOutFiltered = moveToward(turnOutFiltered, wantedTurnOut, tune.turnResponse * dt);

  float leftMotorOut = motorOut + turnOutFiltered;
  float rightMotorOut = motorOut - turnOutFiltered;

  driveMotorSigned(0, leftMotorOut);
  driveMotorSigned(1, rightMotorOut);

  // debug
  if (millis() - lastPrintMs > 100) {
    lastPrintMs = millis();

    Serial.print("mode=");
    Serial.print(tune.name);

    Serial.print(" pitch=");
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

    Serial.print(" driftBias=");
    Serial.print(driftBrakeBias);

    Serial.print(" driveCmd=");
    Serial.print(driveCmd);

    Serial.print(" driveBias=");
    Serial.print(driveLeanBias);

    Serial.print(" turnCmd=");
    Serial.print(turnCmd);

    Serial.print(" turnOut=");
    Serial.print(turnOutFiltered);

    Serial.print(" leftOut=");
    Serial.print(leftMotorOut);

    Serial.print(" rightOut=");
    Serial.print(rightMotorOut);

    Serial.print(" yaw=");
    Serial.println(yaw);
  }
}