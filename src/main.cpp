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
static const int slCrouch = 180;
static const int slStand  = 150;
static const int srCrouch = 10;
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

// controller buttons
struct ControllerState {
  bool a, b, x, y;
  bool dpadLeft, dpadRight;
  bool dpadUp, dpadDown;
};

ControllerState xboxCtrl;

// dpad values
static const uint8_t DPAD_UP_MASK = 0x01;
static const uint8_t DPAD_DOWN_MASK = 0x02;
static const uint8_t DPAD_RIGHT_MASK = 0x04;
static const uint8_t DPAD_LEFT_MASK = 0x08;

// last button states
bool lastA = false;
bool lastB = false;
bool lastX = false;
bool lastY = false;
bool lastDpadLeft = false;
bool lastDpadRight = false;
bool lastDpadUp = false;
bool lastDpadDown = false;

bool leftLegState = false;
bool rightLegState = false;

bool posState = true; // true = standing, false = crouching

// balance targets
static float standingTargetPitch = 5.00f;
static float crouchingTargetPitch = 1.3f;
static float targetPitchDeg = standingTargetPitch;

static const float motorSign = -1.0f;

// standing balance tuning
static float standingKpNear = 40.0f;
static float standingKpMid = 42.0f;
static float standingKpFar = 46.0f;

static float standingKd = 0.30f;

// crouching balance tuning
static float crouchingKp = 34.0f;
static float crouchingKd = 0.70f;

// push recovery tuning
static float standingDriftBrakeSign = 1.0f;
static float standingDriftBrakeK = 0.025f;
static float standingDriftBrakeMax = 1.0f;
static float standingDriftBrakeDecay = 2.0f;

// turning tuning
static float turnSign = 1.0f;

static bool turning = false;
static float turnTargetYaw = 0.0f;

static float turnKp = 4.0f;
static float maxTurnOut = 35.0f;

static float turnDoneDeg = 4.0f;
static int turnDoneCount = 0;
static const int turnDoneNeeded = 8;

// forward and backward movement
static float driveNudgeSign = -1.0f;
static float driveNudgeDeg = 1.2f;
static u32 driveNudgeMs = 1000;
static float driveNudgeDecay = 6.0f;

static float driveNudgeBias = 0.0f;
static u32 driveNudgeEndMs = 0;

// active pid values
static float balanceKp = standingKpFar;
static float balanceKi = 0.0f;
static float balanceKd = standingKd;

void setPidVals(float Kp, float Kd) {
  balanceKp = Kp;
  balanceKd = Kd;
}

// motor output limits
static const float outDeadband = 5.0f;
static const int minBalancePwm = 18;
static const int maxBalancePwm = 255;

// fall cutoff
static const float fallAngle = 38.0f;

// pid memory
static float balanceIntegral = 0.0f;
static float balancePrevErr = 0.0f;

// push recovery memory
static float standingDriftBrakeBias = 0.0f;
static float lastMotorOut = 0.0f;

// imu memory
static float pitchFiltered = 0.0f;

// timing
static u32 lastUs = 0;
static u32 lastPrintMs = 0;

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

static float wrapAngle180(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }

  while (angle < -180.0f) {
    angle += 360.0f;
  }

  return angle;
}

static float addYawDegrees(float yaw, float addDeg) {
  float result = yaw + addDeg;

  while (result >= 360.0f) {
    result -= 360.0f;
  }

  while (result < 0.0f) {
    result += 360.0f;
  }

  return result;
}

static void startTurnLeft90(float currentYaw) {
  turning = true;
  turnDoneCount = 0;
  turnTargetYaw = addYawDegrees(currentYaw, -90.0f);
}

static void startTurnRight90(float currentYaw) {
  turning = true;
  turnDoneCount = 0;
  turnTargetYaw = addYawDegrees(currentYaw, 90.0f);
}

static void startDriveNudge(float direction) {
  driveNudgeBias = driveNudgeSign * direction * driveNudgeDeg;
  driveNudgeEndMs = millis() + driveNudgeMs;
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
  standingDriftBrakeBias = 0.0f;
  lastMotorOut = 0.0f;
  turning = false;
  turnDoneCount = 0;
  driveNudgeBias = 0.0f;
  driveNudgeEndMs = 0;
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
  const uint8_t dpad = gamepad->dpad();

  s.a = btn & BUTTON_A;
  s.b = btn & BUTTON_B;
  s.x = btn & BUTTON_X;
  s.y = btn & BUTTON_Y;

  s.dpadLeft = dpad & DPAD_LEFT_MASK;
  s.dpadRight = dpad & DPAD_RIGHT_MASK;
  s.dpadUp = dpad & DPAD_UP_MASK;
  s.dpadDown = dpad & DPAD_DOWN_MASK;

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
  targetPitchDeg = standingTargetPitch;
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

  // controller buttons
  if (readController(xboxCtrl)) {
    // both crouch
    if (xboxCtrl.b && !lastB) {
      sl.write(slCrouch);
      sr.write(srCrouch);
      leftLegState = true;
      rightLegState = true;
      turning = false;
      turnDoneCount = 0;
      driveNudgeBias = 0.0f;
    }

    // both stand
    if (xboxCtrl.a && !lastA) {
      sl.write(slStand);
      sr.write(srStand);
      leftLegState = false;
      rightLegState = false;
      turning = false;
      turnDoneCount = 0;
      driveNudgeBias = 0.0f;
    }

    // left leg toggle
    if (xboxCtrl.x && !lastX) {
      leftLegState = !leftLegState;
      sl.write(leftLegState ? slCrouch : slStand);
      turning = false;
      turnDoneCount = 0;
      driveNudgeBias = 0.0f;
    }

    // right leg toggle
    if (xboxCtrl.y && !lastY) {
      rightLegState = !rightLegState;
      sr.write(rightLegState ? srCrouch : srStand);
      turning = false;
      turnDoneCount = 0;
      driveNudgeBias = 0.0f;
    }

    // turn left
    if (xboxCtrl.dpadLeft && !lastDpadLeft && !turning && !(leftLegState && rightLegState)) {
      startTurnLeft90(yaw);
    }

    // turn right
    if (xboxCtrl.dpadRight && !lastDpadRight && !turning && !(leftLegState && rightLegState)) {
      startTurnRight90(yaw);
    }

    // move forward a little
    if (xboxCtrl.dpadUp && !lastDpadUp && !turning && !(leftLegState && rightLegState)) {
      startDriveNudge(1.0f);
    }

    // move backward a little
    if (xboxCtrl.dpadDown && !lastDpadDown && !turning && !(leftLegState && rightLegState)) {
      startDriveNudge(-1.0f);
    }

    lastA = xboxCtrl.a;
    lastB = xboxCtrl.b;
    lastX = xboxCtrl.x;
    lastY = xboxCtrl.y;
    lastDpadLeft = xboxCtrl.dpadLeft;
    lastDpadRight = xboxCtrl.dpadRight;
    lastDpadUp = xboxCtrl.dpadUp;
    lastDpadDown = xboxCtrl.dpadDown;
  } else {
    lastA = false;
    lastB = false;
    lastX = false;
    lastY = false;
    lastDpadLeft = false;
    lastDpadRight = false;
    lastDpadUp = false;
    lastDpadDown = false;
  }

  // switch balance target depending on leg position
  if (leftLegState && rightLegState) {
    targetPitchDeg = crouchingTargetPitch;
    setPidVals(crouchingKp, crouchingKd);
    posState = false;

    standingDriftBrakeBias = moveTowardZero(
      standingDriftBrakeBias,
      standingDriftBrakeDecay * dt
    );

    driveNudgeBias = moveTowardZero(
      driveNudgeBias,
      driveNudgeDecay * dt
    );
  } else {
    posState = true;
    setPidVals(standingKpFar, standingKd);

    // push recovery
    float standingErrorNoDrift = pitch - standingTargetPitch;

    if (fabsf(lastMotorOut) > 25.0f && fabsf(standingErrorNoDrift) < 4.0f) {
      standingDriftBrakeBias += standingDriftBrakeSign * lastMotorOut * standingDriftBrakeK * dt;
      standingDriftBrakeBias = clampf(
        standingDriftBrakeBias,
        -standingDriftBrakeMax,
        standingDriftBrakeMax
      );
    } else {
      standingDriftBrakeBias = moveTowardZero(
        standingDriftBrakeBias,
        standingDriftBrakeDecay * dt
      );
    }

    // fade movement back out
    if ((int32_t)(millis() - driveNudgeEndMs) > 0) {
      driveNudgeBias = moveTowardZero(
        driveNudgeBias,
        driveNudgeDecay * dt
      );
    }

    targetPitchDeg = standingTargetPitch + standingDriftBrakeBias + driveNudgeBias;
  }

  // stop if it falls too far
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

  // main pid
  float error = pitch - targetPitchDeg;

  float activeKp = balanceKp;

  if (posState) {
    float absError = fabsf(error);

    if (absError < 2.0f) {
      activeKp = standingKpNear;
    } else if (absError < 7.0f) {
      activeKp = standingKpMid;
    } else {
      activeKp = standingKpFar;
    }
  }

  balanceIntegral += error * dt;
  balanceIntegral = clampf(balanceIntegral, -20.0f, 20.0f);

  float derivative = (error - balancePrevErr) / dt;
  balancePrevErr = error;

  derivative = clampf(derivative, -250.0f, 250.0f);

  float pid =
      (activeKp * error) +
      (balanceKi * balanceIntegral) +
      (balanceKd * derivative);

  float motorOut = motorSign * pid;
  lastMotorOut = motorOut;

  int basePwm = makeMotorPwm(motorOut);

  // turning
  float turnOut = 0.0f;
  float yawError = 0.0f;

  if (turning) {
    yawError = wrapAngle180(turnTargetYaw - yaw);

    turnOut = turnSign * turnKp * yawError;
    turnOut = clampf(turnOut, -maxTurnOut, maxTurnOut);

    if (fabsf(yawError) < turnDoneDeg) {
      turnDoneCount++;
    } else {
      turnDoneCount = 0;
    }

    if (turnDoneCount >= turnDoneNeeded) {
      turning = false;
      turnDoneCount = 0;
      turnOut = 0.0f;
    }
  }

  float leftMotorOut = motorOut + turnOut;
  float rightMotorOut = motorOut - turnOut;

  driveMotorSigned(0, leftMotorOut);
  driveMotorSigned(1, rightMotorOut);

  // debug
  if (millis() - lastPrintMs > 100) {
    lastPrintMs = millis();

    Serial.print("pitch=");
    Serial.print(pitch);

    Serial.print(" target=");
    Serial.print(targetPitchDeg);

    Serial.print(" error=");
    Serial.print(error);

    Serial.print(" P=");
    Serial.print(activeKp * error);

    Serial.print(" I=");
    Serial.print(balanceKi * balanceIntegral);

    Serial.print(" D=");
    Serial.print(balanceKd * derivative);

    Serial.print(" out=");
    Serial.print(motorOut);

    Serial.print(" pwm=");
    Serial.print(basePwm);

    Serial.print(" driftBias=");
    Serial.print(standingDriftBrakeBias);

    Serial.print(" driveBias=");
    Serial.print(driveNudgeBias);

    Serial.print(" yaw=");
    Serial.print(yaw);

    Serial.print(" turn=");
    Serial.print(turning ? "on" : "off");

    Serial.print(" yawErr=");
    Serial.print(yawError);

    Serial.print(" turnOut=");
    Serial.print(turnOut);

    Serial.print(" pos=");
    Serial.println(posState ? "standing" : "crouching");
  }
}