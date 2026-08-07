// ESP32 motor + hall-encoder odometry + MPU6050 bridge for a ROS2/slam_toolbox robot.
//
// Role: this sketch does NOT do any ROS2/SLAM math itself. It only:
//   1) receives normalized per-wheel speed commands from the Jetson over serial ("V,..")
//   2) drives 2 motors via DAC throttle + direction pin
//   3) counts hall-sensor ticks (direction-aware) for odometry feedback
//   4) reads the MPU6050 and reports raw accel/gyro in physical units
//   5) streams tick deltas + IMU data back to the Jetson over serial ("O,..")
//
// All distance/kinematics math (wheel diameter, wheelbase, cmd_vel -> wheel speed,
// odometry integration) happens on the Jetson side in esp32_bridge_node.py.
//
// Serial protocol (115200 baud, ASCII, newline-terminated):
//   Jetson -> ESP32:  V,<left_norm>,<right_norm>\n        left/right in [-1.0, 1.0]
//   ESP32 -> Jetson:  O,<dt_ms>,<left_ticks>,<right_ticks>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n
//                      ax,ay,az in m/s^2 ; gx,gy,gz in rad/s ; ticks are signed deltas
//
// Safety: if no "V," command is received for CMD_TIMEOUT_MS, motors are forced to stop.

#include <Wire.h>
#include <math.h>

// ---------------------------------------------------------------------------
// CONFIG - pins from your wiring
// ---------------------------------------------------------------------------
#define HALL_LA 32
#define HALL_LB 34   // input-only pin: no internal pull-up on ESP32, needs external/onboard pull-up
#define HALL_LC 35   // input-only pin: same as above
#define HALL_RA 13
#define HALL_RB 14
#define HALL_RC 27

#define THROTTLE_R_PIN 25  // dac_chan_0
#define THROTTLE_L_PIN 26  // dac_chan_1
#define DIR_L_PIN 2
#define DIR_R_PIN 4

#define DAC_MIN 108
#define DAC_MAX 135
#define POLE_PAIRS 10
#define TICKS_PER_REV (POLE_PAIRS * 6)  // = 60, informational only, Jetson also knows this

// Flip these to true if a motor's reported tick direction / spin direction
// comes out backwards once you test it.
#define LEFT_ENCODER_INVERT false
#define RIGHT_ENCODER_INVERT false
#define LEFT_MOTOR_INVERT false
#define RIGHT_MOTOR_INVERT false

// Below this normalized magnitude, motor output is forced fully off (not DAC_MIN).
#define SPEED_DEADZONE 0.02f

// If the Jetson stops sending velocity commands for this long, stop the motors.
#define CMD_TIMEOUT_MS 500

// Telemetry ("O,..") send rate
#define ODOM_PERIOD_MS 20  // 50 Hz

// I2C pins for MPU6050 (default ESP32 Wire pins). Change if wired differently.
#define I2C_SDA 21
#define I2C_SCL 22

// ---------------------------------------------------------------------------
// MPU6050 (raw register access, no external library dependency)
// ---------------------------------------------------------------------------
#define MPU_ADDR 0x68
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_WHO_AM_I 0x75

static const float ACCEL_SCALE = 9.80665f / 16384.0f;      // m/s^2 per LSB, +/-2g range
static const float GYRO_SCALE = (float)(M_PI / 180.0) / 131.0f;  // rad/s per LSB, +/-250 dps range

bool mpuOk = false;

bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 1) != 1) return false;
  uint8_t who = Wire.read();
  if (who != 0x68) return false;

  if (!mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00)) return false;   // wake up
  if (!mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00)) return false; // +/-2g
  if (!mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00)) return false;  // +/-250 dps
  return true;
}

bool mpuRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // temperature, unused
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx * ACCEL_SCALE;
  ay = rawAy * ACCEL_SCALE;
  az = rawAz * ACCEL_SCALE;
  gx = rawGx * GYRO_SCALE;
  gy = rawGy * GYRO_SCALE;
  gz = rawGz * GYRO_SCALE;
  return true;
}

// ---------------------------------------------------------------------------
// Hall-sensor tick counting (3-hall BLDC used as a direction-aware encoder)
// ---------------------------------------------------------------------------
// Standard 6-step Gray-like sequence for 3 hall sensors spaced 120 degrees
// apart is: 1,3,2,6,4,5 (repeating). seqPos[] maps a raw 3-bit hall state
// to its position (0..5) in that sequence; -1 marks an invalid/impossible state.
static const int8_t seqPos[8] = {-1, 0, 2, 1, 4, 5, 3, -1};

static inline int8_t hallDelta(uint8_t prev, uint8_t curr) {
  if (prev == curr) return 0;
  int8_t p = seqPos[prev];
  int8_t c = seqPos[curr];
  if (p < 0 || c < 0) return 0;  // glitch / invalid transition, ignore
  int8_t diff = c - p;
  if (diff == 1 || diff == -5) return 1;
  if (diff == -1 || diff == 5) return -1;
  return 0;  // skipped step, ignore rather than guess
}

struct HallCtx {
  uint8_t pinA, pinB, pinC;
  volatile uint8_t state;
  volatile long ticks;
  bool invert;
};

HallCtx leftHall  = {HALL_LA, HALL_LB, HALL_LC, 0, 0, LEFT_ENCODER_INVERT};
HallCtx rightHall = {HALL_RA, HALL_RB, HALL_RC, 0, 0, RIGHT_ENCODER_INVERT};

void IRAM_ATTR onHallChange(void *arg) {
  HallCtx *h = (HallCtx *)arg;
  uint8_t s = (digitalRead(h->pinA) << 2) | (digitalRead(h->pinB) << 1) | digitalRead(h->pinC);
  int8_t d = hallDelta(h->state, s);
  if (h->invert) d = -d;
  h->ticks += d;
  h->state = s;
}

void setupHall(HallCtx &h) {
  pinMode(h.pinA, INPUT);
  pinMode(h.pinB, INPUT);
  pinMode(h.pinC, INPUT);
  h.state = (digitalRead(h.pinA) << 2) | (digitalRead(h.pinB) << 1) | digitalRead(h.pinC);
  attachInterruptArg(digitalPinToInterrupt(h.pinA), onHallChange, &h, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(h.pinB), onHallChange, &h, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(h.pinC), onHallChange, &h, CHANGE);
}

long readAndResetTicks(HallCtx &h) {
  noInterrupts();
  long t = h.ticks;
  h.ticks = 0;
  interrupts();
  return t;
}

// ---------------------------------------------------------------------------
// Motor output
// ---------------------------------------------------------------------------
float targetLeft = 0.0f;
float targetRight = 0.0f;
unsigned long lastCmdMs = 0;

void applyMotor(float norm, int dacPin, int dirPin, bool invert) {
  if (invert) norm = -norm;
  float mag = fabsf(norm);
  if (mag < SPEED_DEADZONE) {
    dacWrite(dacPin, 0);
    return;
  }
  mag = constrain(mag, 0.0f, 1.0f);
  int dacVal = DAC_MIN + (int)(mag * (DAC_MAX - DAC_MIN));
  // Adjust HIGH/LOW here if a wheel spins opposite to the commanded direction.
  digitalWrite(dirPin, norm > 0 ? HIGH : LOW);
  dacWrite(dacPin, dacVal);
}

// ---------------------------------------------------------------------------
// Serial command parsing: "V,<left_norm>,<right_norm>\n"
// ---------------------------------------------------------------------------
String rxLine;

void parseCommand(const String &line) {
  if (line.length() < 2 || line[0] != 'V' || line[1] != ',') return;
  int comma = line.indexOf(',', 2);
  if (comma < 0) return;
  float l = line.substring(2, comma).toFloat();
  float r = line.substring(comma + 1).toFloat();
  targetLeft = constrain(l, -1.0f, 1.0f);
  targetRight = constrain(r, -1.0f, 1.0f);
  lastCmdMs = millis();
}

void pollSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseCommand(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      rxLine += c;
    }
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
unsigned long lastOdomMs = 0;

void setup() {
  Serial.begin(115200);

  pinMode(DIR_L_PIN, OUTPUT);
  pinMode(DIR_R_PIN, OUTPUT);
  digitalWrite(DIR_L_PIN, LOW);
  digitalWrite(DIR_R_PIN, LOW);
  dacWrite(THROTTLE_L_PIN, 0);
  dacWrite(THROTTLE_R_PIN, 0);

  setupHall(leftHall);
  setupHall(rightHall);

  mpuOk = mpuInit();

  lastCmdMs = millis();
  lastOdomMs = millis();
}

void loop() {
  pollSerialCommands();

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    targetLeft = 0.0f;
    targetRight = 0.0f;
  }

  applyMotor(targetLeft, THROTTLE_L_PIN, DIR_L_PIN, LEFT_MOTOR_INVERT);
  applyMotor(targetRight, THROTTLE_R_PIN, DIR_R_PIN, RIGHT_MOTOR_INVERT);

  unsigned long now = millis();
  if (now - lastOdomMs >= ODOM_PERIOD_MS) {
    unsigned long dt = now - lastOdomMs;
    lastOdomMs = now;

    long lt = readAndResetTicks(leftHall);
    long rt = readAndResetTicks(rightHall);

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (mpuOk) {
      if (!mpuRead(ax, ay, az, gx, gy, gz)) {
        mpuOk = false;  // will retry next boot; avoids repeated failed I2C transactions every cycle
      }
    }

    Serial.printf("O,%lu,%ld,%ld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                  dt, lt, rt, ax, ay, az, gx, gy, gz);
  }
}
