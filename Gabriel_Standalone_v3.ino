/*
  GABRIEL v3 — standalone navigation + ORIGINAL Gabriel Eyes V7
  For educational testing: review, calibrate and adapt for your own NEA.

  ESP32-C6 | L9110S | HC-SR04 | MPU6050 | ILI9341 240x320 SPI
  No phone app and no Wi-Fi needed.

  POWER: The ESP32 still needs USB or an appropriate regulated supply.
  4xAA can separately power the motor driver ONLY if the motor voltage rating
  permits it. ESP32 and L9110 MUST share ground. Never apply 4xAA (~6V)
  directly to ESP32 5V/3V3. HC-SR04 ECHO requires 5V->3.3V level shifting.

  CAUTION: Only a FORWARD ultrasonic sensor is present. It cannot see behind,
  above/below, or to the sides without rotating. Do not test near ledges,
  stairs, people, or objects that can be damaged. Lift wheels on first boot.
  This is an obstacle avoidance experiment, NOT guaranteed collision avoidance.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ==== PINS: carried over from the previous sketch; CHECK YOUR WIRES ====
constexpr uint8_t MOTOR_B_1 = 0;
constexpr uint8_t MOTOR_B_2 = 1;
constexpr uint8_t MOTOR_A_1 = 2;
constexpr uint8_t MOTOR_A_2 = 3;
constexpr uint8_t SONAR_TRIG = 18;
constexpr uint8_t SONAR_ECHO = 19;  // MUST be level-shifted to 3.3 V
constexpr uint8_t MPU_SDA = 6;
constexpr uint8_t MPU_SCL = 7;
constexpr uint8_t TFT_SCK = 5;
constexpr uint8_t TFT_MOSI = 15;
constexpr uint8_t TFT_CS = 10;    // NOT T_CS
constexpr uint8_t TFT_DC = 4;
constexpr uint8_t TFT_RST = 11;

// ==== CUSTOMISABLE SETTINGS ====
constexpr bool MOTOR_A_IS_LEFT = true;
constexpr bool INVERT_MOTOR_A = false;
constexpr bool INVERT_MOTOR_B = true;
constexpr bool MOTORS_ENABLED = true;
constexpr bool DISPLAY_TEST_MODE = false; // true = eyes V7 only, no motors
constexpr bool ALLOW_TIMED_TURNS = false; // false = stop if MPU is unavailable
// Change to 0 for X or 1 for Y if the MPU6050 is mounted on its side.
// For a flat, horizontal MPU6050, vertical yaw is usually Z (2).
constexpr uint8_t GYRO_TURN_AXIS = 2;

// Keep these conservative; narrow passages need more space than just
// the distance from the sensor to the wall.
constexpr float BRAKE_DISTANCE_CM = 35.0f;
constexpr float ROUTE_CLEAR_CM = 48.0f;
constexpr float VALID_MAX_CM = 300.0f;
constexpr uint32_t SONAR_DRIVE_PERIOD_MS = 115;
constexpr uint32_t TURN_SETTLE_MS = 180;
constexpr uint32_t TIMED_MS_PER_DEG = 10;  // gyro-unavailable fallback only!
constexpr uint8_t MAX_FAILED_SCANS = 3;

// Scan geometry relative to original heading:
// centre -> -65 (left) -> +65 (right) -> +180 (behind).
// If left is chosen from +65, rotate back by -130 degrees.
constexpr float SCAN_SIDE_DEG = 65.0f;
constexpr float SCAN_ACROSS_DEG = 130.0f;
constexpr float SCAN_BACK_DEG = 115.0f;

// Use the dedicated SPI object and the exact wiring from your V7 eyes sketch.
SPIClass displaySPI(FSPI);
Adafruit_ILI9341 tft(&displaySPI, TFT_DC, TFT_CS, TFT_RST);

// Arduino normally auto-generates function prototypes; providing these explicit
// primitive-typed prototypes prevents enum-related preprocessor problems.
void updateEyes();
void transition(uint8_t next);
void beginRotation(float signedDegrees, uint8_t afterwards);

// ==== MOTOR CONTROL ====
void setMotor(uint8_t pin1, uint8_t pin2, int dir, bool invert) {
  if (invert) dir = -dir;
  if (!MOTORS_ENABLED) dir = 0;
  digitalWrite(pin1, dir > 0 ? HIGH : LOW);
  digitalWrite(pin2, dir < 0 ? HIGH : LOW);
}
void wheels(int left, int right) {
  int a = MOTOR_A_IS_LEFT ? left : right;
  int b = MOTOR_A_IS_LEFT ? right : left;
  setMotor(MOTOR_A_1, MOTOR_A_2, a, INVERT_MOTOR_A);
  setMotor(MOTOR_B_1, MOTOR_B_2, b, INVERT_MOTOR_B);
}
void motorStop() { wheels(0, 0); }

// ==== SONAR ====
float sonarOnce() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);
  uint32_t pulse = pulseIn(SONAR_ECHO, HIGH, 23000UL);
  if (!pulse) return -1.0f; // NO ECHO means UNKNOWN, not empty space
  float cm = pulse * 0.01715f;
  if (cm < 2.0f) return 0.0f; // extremely close; treat as obstructed
  if (cm > VALID_MAX_CM) return -1.0f;
  return cm;
}

// Stopped-only sampling: median of 3, require at least 2 valid readings.
// ~65-130 ms; NEVER call during a turn.
float sonarStable() {
  float v[3];
  int count = 0;
  for (int i = 0; i < 3; i++) {
    float d = sonarOnce();
    if (d >= 0) v[count++] = d;
    if (i < 2) {
      // Keep the V7 eyes animated during acoustic quiet-time between pings.
      uint32_t endWait = millis() + 38;
      while ((int32_t)(millis() - endWait) < 0) {
        updateEyes();
        delay(2);
      }
    }
  }
  if (count < 2) return -1.0f;
  if (count == 2) return (v[0] + v[1]) * 0.5f;
  if (v[0] > v[1]) { float a=v[0]; v[0]=v[1]; v[1]=a; }
  if (v[1] > v[2]) { float a=v[1]; v[1]=v[2]; v[2]=a; }
  if (v[0] > v[1]) { float a=v[0]; v[0]=v[1]; v[1]=a; }
  return v[1];
}

// ==== MPU6050 turn sensing ====
// Measure yaw on ONE selected physical gyro axis (not a magnitude of X/Y/Z,
// which can mistake vibration, wheel bouncing, or tilting for actual turning).
// The sensor should be securely mounted and kept still while calibrating.
bool gyroOK = false;
uint8_t mpuAddr = 0x68;
float gyroBias[3] = {0,0,0};

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool mpuRates(float rates[3]) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x43);  // GYRO_XOUT_H ... GYRO_ZOUT_L, six bytes
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)6) != 6) return false;
  for (int i=0; i<3; i++) {
    int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
    rates[i] = raw / 131.0f; // +/-250 deg/s mode
  }
  return true;
}
bool initGyro() {
  bool found = false;
  for (uint8_t addr=0x68; addr<=0x69; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      mpuAddr = addr;
      found = true;
      break;
    }
  }
  if (!found || !mpuWrite(0x6B, 0x00) || !mpuWrite(0x1B, 0x00)) return false;
  delay(100);
  float sums[3] = {0,0,0};
  int n = 0;
  for (int i=0; i<180; i++) {
    float r[3];
    if (mpuRates(r)) {
      for (int a=0; a<3; a++) sums[a] += r[a];
      n++;
    }
    delay(4);
  }
  if (n < 150) return false;
  for (int a=0; a<3; a++) gyroBias[a] = sums[a] / n;
  return true;
}

// ==== DISPLAY: Gabriel Eyes V7 (your original renderer, unchanged visually) ====
// Smaller off-screen eye buffer
const int BW = 112;
const int BH = 120;

GFXcanvas16 eye(BW, BH);

const int LEFT_X = 38;
const int RIGHT_X = 170;
const int TOP_Y = 60;

uint16_t BLUE;
uint16_t BG = ILI9341_BLACK;

// Expression types
enum Expression {
  NEUTRAL,
  HAPPY,
  CURIOUS,
  SURPRISED,
  SLEEPY
};

Expression expression = NEUTRAL;

// Position and expression animation
float gazeX = 0;
float gazeY = 0;
float targetX = 0;
float targetY = 0;

float widthNow = 76;
float heightNow = 78;
float widthTarget = 76;
float heightTarget = 78;

bool blinking = false;
unsigned long blinkStart = 0;

unsigned long nextBlink = 0;
unsigned long nextLook = 0;
unsigned long nextExpression = 0;
unsigned long lastFrame = 0;

const unsigned long BLINK_MS = 200;
const unsigned long FRAME_MS = 45;

bool needsDraw = true;

float ease(float current, float target) {
  if (fabsf(target - current) < 0.35f)
    return target;

  return current + (target - current) * 0.25f;
}

void chooseExpression() {
  int choice = random(0, 10);

  // Neutral most of the time
  if (choice < 5) expression = NEUTRAL;
  else if (choice < 7) expression = HAPPY;
  else if (choice == 7) expression = CURIOUS;
  else if (choice == 8) expression = SURPRISED;
  else expression = SLEEPY;

  switch (expression) {
    case NEUTRAL:
      widthTarget = 76;
      heightTarget = 78;
      break;

    case HAPPY:
      widthTarget = 80;
      heightTarget = 44;
      break;

    case CURIOUS:
      widthTarget = 72;
      heightTarget = 88;
      break;

    case SURPRISED:
      widthTarget = 76;
      heightTarget = 96;
      break;

    case SLEEPY:
      widthTarget = 80;
      heightTarget = 19;
      break;
  }

  nextExpression =
    millis() + random(6000, 11000);

  needsDraw = true;
}

void chooseLook() {
  int choice = random(0, 6);

  if (choice == 0) {
    targetX = -10;
    targetY = 0;
  }
  else if (choice == 1) {
    targetX = 10;
    targetY = 0;
  }
  else if (choice == 2) {
    targetX = 0;
    targetY = -7;
  }
  else {
    targetX = 0;
    targetY = 0;
  }

  nextLook = millis() + random(5000, 9000);
  needsDraw = true;
}

float blinkScale(unsigned long now) {
  if (!blinking) return 1.0f;

  unsigned long dt = now - blinkStart;

  if (dt >= BLINK_MS) {
    blinking = false;
    nextBlink = now + random(3000, 6000);
    return 1.0f;
  }

  float p = float(dt) / BLINK_MS;

  return 0.04f + 0.96f *
    fabsf(cosf(p * PI));
}

void renderEye(float opening, bool left) {
  eye.fillScreen(BG);

  int cx = BW / 2 + roundf(gazeX);
  int cy = BH / 2 + roundf(gazeY);

  int w = roundf(widthNow);
  int h = max(4, int(roundf(heightNow * opening)));

  // Curious: slightly asymmetrical eyes
  if (expression == CURIOUS && !left) {
    h = max(4, int(h * 0.72f));
  }

  int x = cx - w / 2;
  int y = cy - h / 2;

  int radius = min(17, h / 2);

  eye.fillRoundRect(
    x, y, w, h, radius, BLUE
  );

  // Happy: subtle curved lower eyelid
  if (expression == HAPPY && h > 20) {
    eye.fillCircle(
      cx, y + h + 24,
      36, BG
    );
  }

  // Sleepy: upper eyelid
  if (expression == SLEEPY && h > 9) {
    eye.fillRect(
      x, y, w, 4, BG
    );
  }
}

void drawEyes(float opening) {
  renderEye(opening, true);

  tft.drawRGBBitmap(
    LEFT_X, TOP_Y,
    eye.getBuffer(), BW, BH
  );

  renderEye(opening, false);

  tft.drawRGBBitmap(
    RIGHT_X, TOP_Y,
    eye.getBuffer(), BW, BH
  );
}

// V7 loop code moved into a function so autonomous driving can run beside it.
void updateEyes() {
  if (!eye.getBuffer()) return;

  unsigned long now = millis();

  if (now >= nextLook)
    chooseLook();

  if (now >= nextExpression)
    chooseExpression();

  if (!blinking && now >= nextBlink) {
    blinking = true;
    blinkStart = now;
    needsDraw = true;
  }

  if (now - lastFrame < FRAME_MS)
    return;

  lastFrame = now;

  float oldX = gazeX;
  float oldY = gazeY;
  float oldW = widthNow;
  float oldH = heightNow;

  gazeX = ease(gazeX, targetX);
  gazeY = ease(gazeY, targetY);

  widthNow = ease(widthNow, widthTarget);
  heightNow = ease(heightNow, heightTarget);

  bool moving =
    fabsf(gazeX - oldX) > 0.05f ||
    fabsf(gazeY - oldY) > 0.05f ||
    fabsf(widthNow - oldW) > 0.05f ||
    fabsf(heightNow - oldH) > 0.05f;

  // Don't refresh the screen continuously
  if (needsDraw || moving || blinking) {
    drawEyes(blinkScale(now));
    needsDraw = false;
  }
}

// ==== NAVIGATION ====
// Obstacle scan rotates the whole chassis: left / right / rear, then verifies.
// No single forward-facing sonar can prove a route is side-clear. If Gabriel is
// physically wedged, STOP him; do not let the wheels force a turn against walls.
enum NavState : uint8_t {
  CHECK_START, FORWARD, ROTATE,
  READ_LEFT, READ_RIGHT, READ_BACK, VERIFY_ROUTE, TRAPPED
};
NavState state = CHECK_START;
NavState afterRotate = CHECK_START;
uint32_t stateAt = 0;
uint32_t lastSonar = 0;
uint32_t lastTrappedCheck = 0;
uint32_t forwardSince = 0;
float leftCm = -1, rightCm = -1, backCm = -1;
uint8_t failedScans = 0;

// Rotate towards requested signed angle (negative=left, positive=right).
float targetAngle = 0;
float turnedAngle = 0;
int turnDirection = 0;
uint32_t turnAt = 0;
uint32_t gyroAt = 0;
uint32_t maxTurnMs = 0;
uint32_t settleUntil = 0;

void transition(uint8_t next) {
  motorStop();
  state = static_cast<NavState>(next);
  stateAt = millis();
}

void enterTrapped(const char *why) {
  transition(TRAPPED);
  Serial.print("BLOCKED/STOPPED: ");
  Serial.println(why);
}

void beginRotation(float signedDegrees, uint8_t afterwards) {
  motorStop();
  targetAngle = fabsf(signedDegrees);
  turnedAngle = 0;
  turnDirection = signedDegrees < 0 ? -1 : +1;
  afterRotate = static_cast<NavState>(afterwards);
  turnAt = millis();
  gyroAt = turnAt;
  // generous deadline; if the gyroscope detects stalled rotation we HALT.
  maxTurnMs = (uint32_t)(targetAngle * TIMED_MS_PER_DEG * 2.5f + 650);
  transition(ROTATE);
  Serial.print("TURN ");
  Serial.print(turnDirection < 0 ? "LEFT " : "RIGHT ");
  Serial.println(targetAngle, 1);
}

void beginScan() {
  if (++failedScans > MAX_FAILED_SCANS) {
    enterTrapped("Repeated blocked routes; inspect surroundings");
    return;
  }
  motorStop();
  leftCm = rightCm = backCm = -1;
  Serial.print("SCAN attempt "); Serial.println(failedScans);
  beginRotation(-SCAN_SIDE_DEG, READ_LEFT);
}

void startForward() {
  transition(FORWARD);
  lastSonar = millis() - SONAR_DRIVE_PERIOD_MS;
  forwardSince = millis();
  Serial.println("FORWARD: route confirmed clear");
}

void finishRotate() {
  motorStop();
  transition(afterRotate);
  settleUntil = millis() + TURN_SETTLE_MS;
}

void updateRotation(uint32_t now) {
  wheels(turnDirection < 0 ? -1 : +1,
         turnDirection < 0 ? +1 : -1);

  if (gyroOK && now - gyroAt >= 10) {
    float dt = (now - gyroAt) * 0.001f;
    gyroAt = now;
    float r[3];
    if (!mpuRates(r)) {
      enterTrapped("MPU6050 stopped communicating while rotating");
      return;
    }
    // Only integrate the rotation axis that represents yaw on your MPU mounting.
    float rate = fabsf(r[GYRO_TURN_AXIS] - gyroBias[GYRO_TURN_AXIS]);
    if (rate > 4.5f && rate < 350.0f) {
      // Don't integrate absurdly long loop gaps as though motion were known.
      turnedAngle += rate * fminf(dt, 0.085f);
    }
  }

  bool done = gyroOK ? (turnedAngle >= targetAngle)
                     : (now - turnAt >= (uint32_t)(targetAngle * TIMED_MS_PER_DEG));
  if (done) {
    Serial.print("TURN COMPLETED, gyro degrees: "); Serial.println(turnedAngle, 1);
    finishRotate();
  } else if (now - turnAt > maxTurnMs) {
    enterTrapped(gyroOK ? "Gyro sees insufficient rotation: motors/wires/MPU?"
                        : "Timed rotation exceeded safe limit");
  }
}

void printScan(const char* direction, float cm) {
  Serial.print(direction); Serial.print(": ");
  if (cm < 0) Serial.println("UNKNOWN");
  else { Serial.print(cm, 1); Serial.println(" cm"); }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_A_1, OUTPUT); pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT); pinMode(MOTOR_B_2, OUTPUT);
  motorStop();
  pinMode(SONAR_TRIG, OUTPUT); pinMode(SONAR_ECHO, INPUT);
  digitalWrite(SONAR_TRIG, LOW);

  // Exact TFT SPI settings and colour from your original V7 sketch.
  displaySPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  tft.begin(40000000);
  tft.setRotation(1);
  BLUE = tft.color565(22, 143, 205);
  tft.fillScreen(BG);
  if (!eye.getBuffer()) {
    Serial.println("ERROR: Eyes V7 buffer allocation failed. Motors will not run.");
    return;
  }
  randomSeed(micros());
  unsigned long eyeNow = millis();
  nextBlink = eyeNow + 3000;
  nextLook = eyeNow + 5000;
  nextExpression = eyeNow + 6500;
  drawEyes(1.0f);
  Serial.println("Gabriel Eyes V7 Online");

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000);
  Serial.println("Gabriel v3: keep STILL for gyro bias calibration...");
  gyroOK = initGyro();
  Serial.print("Gyro turn axis (0=X, 1=Y, 2=Z): ");
  Serial.println(GYRO_TURN_AXIS);
  Serial.println(gyroOK ? "MPU6050 OK: single-axis turn sensing" :
                          "MPU6050 not found: cannot safely use measured turns");
  if (!MOTORS_ENABLED) Serial.println("TEST MODE: MOTORS DISABLED");
  if (DISPLAY_TEST_MODE) Serial.println("DISPLAY TEST MODE: motors will stay off");
  if (!gyroOK && !ALLOW_TIMED_TURNS && !DISPLAY_TEST_MODE) {
    Serial.println("HALTED: gyro missing and timed turns disabled for safety");
  }
  transition(CHECK_START);
}

void loop() {
  uint32_t now = millis();
  // Display update is exactly the V7 timing/expression logic.
  updateEyes();
  if (!eye.getBuffer()) { motorStop(); delay(100); return; }
  // Eyes-only mode: run the exact original V7 animation, never drive.
  if (DISPLAY_TEST_MODE) { motorStop(); delay(2); return; }
  if (!gyroOK && !ALLOW_TIMED_TURNS) {
    motorStop();
    delay(2); // eyes continue animating even if navigation is disabled
    return;
  }
  switch (state) {
    case CHECK_START: {
      // A fresh stopped reading is required before the FIRST forward motion.
      if (now - stateAt < TURN_SETTLE_MS) break;
      float d = sonarStable();
      printScan("FRONT", d);
      if (d < 0) enterTrapped("Sonar reading unknown (check wiring)");
      else if (d >= ROUTE_CLEAR_CM) startForward();
      else beginScan();
      break;
    }
    case FORWARD: {
      wheels(+1, +1);
      if (now - lastSonar >= SONAR_DRIVE_PERIOD_MS) {
        lastSonar = now;
        float d = sonarOnce();
        if (d < 0) {
          enterTrapped("Sonar failed during forward motion");
        } else if (d <= BRAKE_DISTANCE_CM) {
          motorStop();
          Serial.print("FRONT obstacle "); Serial.println(d, 1);
          beginScan();
        } else if (now - forwardSince > 1900) {
          // It travelled for a while; permit another set of scans later.
          failedScans = 0;
        }
      }
      break;
    }
    case ROTATE:
      updateRotation(now);
      break;

    case READ_LEFT: {
      if (now < settleUntil) break;
      leftCm = sonarStable();
      printScan("LEFT", leftCm);
      beginRotation(+SCAN_ACROSS_DEG, READ_RIGHT);
      break;
    }
    case READ_RIGHT: {
      if (now < settleUntil) break;
      rightCm = sonarStable();
      printScan("RIGHT", rightCm);
      bool leftClear = leftCm >= ROUTE_CLEAR_CM;
      bool rightClear = rightCm >= ROUTE_CLEAR_CM;
      if (leftClear || rightClear) {
        if (leftClear && (!rightClear || leftCm > rightCm + 8.0f)) {
          Serial.println("SELECTED LEFT (more clearance)");
          beginRotation(-SCAN_ACROSS_DEG, VERIFY_ROUTE);
        } else {
          Serial.println("SELECTED RIGHT");
          transition(VERIFY_ROUTE);  // already facing RIGHT
          settleUntil = millis() + TURN_SETTLE_MS;
        }
      } else {
        Serial.println("Both sides blocked/unknown; checking BACK direction");
        beginRotation(+SCAN_BACK_DEG, READ_BACK);
      }
      break;
    }
    case READ_BACK: {
      if (now < settleUntil) break;
      backCm = sonarStable();
      printScan("BACK", backCm);
      if (backCm >= ROUTE_CLEAR_CM) {
        Serial.println("SELECTED BACK (after rotating; no reversing)");
        transition(VERIFY_ROUTE);
        settleUntil = millis() + TURN_SETTLE_MS;
      } else {
        enterTrapped("Front, both sides, and back are blocked/unknown");
      }
      break;
    }
    case VERIFY_ROUTE: {
      if (now < settleUntil) break;
      float d = sonarStable();
      printScan("VERIFY", d);
      if (d >= ROUTE_CLEAR_CM) startForward();
      else if (d < 0) enterTrapped("Selected route sensor reading unknown");
      else beginScan(); // verify failed -> retry, with hard cap
      break;
    }
    case TRAPPED: {
      motorStop();
      if (now - lastTrappedCheck >= 2500) {
        lastTrappedCheck = now;
        float d = sonarStable();
        if (d >= ROUTE_CLEAR_CM) {
          Serial.println("Path now clear; leaving TRAPPED state");
          failedScans = 0;
          transition(CHECK_START);
        }
      }
      break;
    }
  }
  updateEyes();
  delay(2);
}
