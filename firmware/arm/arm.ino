// Robot arm firmware: receives "P a0 a1 a2 a3 a4 a5" from the laptop,
// clamps to joint limits, moves servos smoothly via a PCA9685, freezes if the laptop goes quiet.
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== Calibration (tune these on the real arm, then re-upload) =====
struct Joint { int lo, hi, home, pulseMin, pulseMax; };  // degrees, degrees, degrees, us, us
Joint J[6] = {
  {20, 160, 90, 500, 2500},  // 0 Base
  {20, 160, 90, 500, 2500},  // 1 Shoulder
  {20, 160, 90, 500, 2500},  // 2 Elbow
  {20, 160, 90, 500, 2500},  // 3 Wrist
  { 0, 180, 90, 500, 2500},  // 4 Claw rotate (needs the full half-turn for the flip)
  {20, 160, 90, 500, 2500},  // 5 Claw grip
};
const long OSC_HZ = 27000000;     // PCA9685 oscillator; chips run a few % off, adjust if every joint is off equally
const int SERVO_HZ = 50;
const float MAX_DEG_PER_S = 90;   // top speed of any joint
const unsigned long WATCHDOG_MS = 500;
const long BAUD = 115200;
// ====================================================================

Adafruit_PWMServoDriver pwm;  // I2C address 0x40, SDA=A4, SCL=A5
float cur[6], tgt[6];
char buf[48];
byte len = 0;
bool overflow = false, gotMsg = false;
unsigned long lastMsg = 0, lastStep = 0;

void writeServo(int i) {
  pwm.writeMicroseconds(i, J[i].pulseMin + (long)(J[i].pulseMax - J[i].pulseMin) * cur[i] / 180);
}

void handleLine() {
  int a[6];
  if (sscanf(buf, "P %d %d %d %d %d %d", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) != 6) return;
  for (int i = 0; i < 6; i++) tgt[i] = constrain(a[i], J[i].lo, J[i].hi);
  lastMsg = millis();
  gotMsg = true;
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[len] = 0;
      if (!overflow) handleLine();
      len = 0;
      overflow = false;
    } else if (c != '\r') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      else overflow = true;  // too long: drop the whole line
    }
  }
}

void setup() {
  Serial.begin(BAUD);
  pwm.begin();
  pwm.setOscillatorFrequency(OSC_HZ);
  pwm.setPWMFreq(SERVO_HZ);
  for (int i = 0; i < 6; i++) {
    cur[i] = tgt[i] = J[i].home;
    writeServo(i);
  }
  Serial.println("READY");
  Serial.print("LIMITS");
  for (int i = 0; i < 6; i++) {
    Serial.print(' '); Serial.print(J[i].lo);
    Serial.print(' '); Serial.print(J[i].hi);
    Serial.print(' '); Serial.print(J[i].home);
  }
  Serial.println();
  lastStep = millis();
}

void loop() {
  readSerial();
  unsigned long now = millis();
  if (now - lastStep < 20) return;
  float maxStep = MAX_DEG_PER_S * (now - lastStep) / 1000.0;
  lastStep = now;
  if (gotMsg && now - lastMsg > WATCHDOG_MS)
    for (int i = 0; i < 6; i++) tgt[i] = cur[i];  // laptop went quiet: freeze
  for (int i = 0; i < 6; i++) {
    float d = constrain(tgt[i] - cur[i], -maxStep, maxStep);
    if (d != 0) {
      cur[i] += d;
      writeServo(i);
    }
  }
}
