// === Robot Dò Line ESP32 - 12 Mắt Cảm Biến Giải Sân Nâng Cao ===
// - Vi điều khiển: ESP32
// - Cảm biến: 12 mắt QTR (hoặc TCRT / analog sensor)
// - Driver: L298N / DRV8833

#include <Arduino.h>

// === KHAI BÁO CẢM BIẾN ===
#define NUM_SENSORS 12
int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13};
int sensorValues[NUM_SENSORS];

// === MOTOR CONTROL ===
#define ENA 5
#define IN1 18
#define IN2 19
#define ENB 17
#define IN3 16
#define IN4 4

// === PID THAM SỐ ===
float Kp = 0.4, Ki = 0.0, Kd = 2.0;
int lastError = 0, integral = 0;
int baseSpeed = 120; // Tốc độ cơ sở (PWM 0 - 255)

// === FSM STATES ===
enum RobotState {
  FOLLOW_LINE,
  LOST_LINE,
  CROSS_LINE,
  VONG_SO_8,
  TO_ONG,
  KET_THUC
};
RobotState state = FOLLOW_LINE;
void setMotor(int left, int right);


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorPins[i], INPUT);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}
void setMotor(int left, int right);

void loop() {
  readSensors();
  RobotState detected = detectPattern();

  switch (detected) {
    case FOLLOW_LINE: followLinePID(); break;
    case LOST_LINE: recoverLine(); break;
    case CROSS_LINE: handleCrossLine(); break;
    case VONG_SO_8: handleVongSo8(); break;
    case TO_ONG: handleToOng(); break;
    case KET_THUC: stopMotors(); break;
  }
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

// === TÍNH VỌ TRÍ LINE ===
int readLinePosition() {
  long sum = 0, total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = map(sensorValues[i], 0, 4095, 1000, 0); // Đổi sáng/thấp
    sum += (i * 1000L) * value;
    total += value;
  }
  if (total == 0) return -1; // Mất line
  return sum / total;
}

void followLinePID() {
  int position = readLinePosition();
  if (position == -1) {
    state = LOST_LINE; return;
  }
  int error = position - ((NUM_SENSORS - 1) * 1000 / 2);
  int derivative = error - lastError;
  integral += error;

  int correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int left = baseSpeed + correction;
  int right = baseSpeed - correction;
  setMotor(left, right);
  state = FOLLOW_LINE;
}

void setMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  analogWrite(ENA, abs(left));
  digitalWrite(IN1, left > 0); digitalWrite(IN2, left <= 0);

  analogWrite(ENB, abs(right));
  digitalWrite(IN3, right > 0); digitalWrite(IN4, right <= 0);
}

// === Mẫu nhận dạng ===
RobotState detectPattern() {
  int active = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] < 1500) active++;
  }
  if (active == 0) return LOST_LINE;
  if (active > 8) return CROSS_LINE;
  if (sensorValues[3] < 1500 && sensorValues[4] > 2500 && sensorValues[5] < 1500) return TO_ONG;
  if (sensorValues[0] < 1000 && sensorValues[11] < 1000) return VONG_SO_8;
  return FOLLOW_LINE;
}

void recoverLine() {
  setMotor(-80, 80);
  delay(200);
  if (readLinePosition() != -1) state = FOLLOW_LINE;
}

void handleCrossLine() {
  setMotor(100, 100);
  delay(250);
  state = FOLLOW_LINE;
}

void handleVongSo8() {
  setMotor(90, 80);
  delay(600);
  state = FOLLOW_LINE;
}

void handleToOng() {
  setMotor(80, 80);
  delay(400);
  state = FOLLOW_LINE;
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
} 
