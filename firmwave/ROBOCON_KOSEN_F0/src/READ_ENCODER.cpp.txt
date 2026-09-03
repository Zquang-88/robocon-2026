#include <Arduino.h>

/* =========================================================
   CẤU HÌNH CHÂN ENCODER — TỰ ĐIỀN CHÂN TẠI ĐÂY
   ========================================================= */

// Động cơ trước trái
#define ENC_FL_A  0
#define ENC_FL_B  1

// Động cơ trước phải
#define ENC_FR_A  2
#define ENC_FR_B  3

// Động cơ sau trái
#define ENC_RL_A  4
#define ENC_RL_B  5

// Động cơ sau phải
#define ENC_RR_A  6
#define ENC_RR_B  7

/* =========================================================
   BIẾN ĐẾM ENCODER
   ========================================================= */

volatile int32_t encoderFL = 0;
volatile int32_t encoderFR = 0;
volatile int32_t encoderRL = 0;
volatile int32_t encoderRR = 0;

volatile uint8_t lastStateFL = 0;
volatile uint8_t lastStateFR = 0;
volatile uint8_t lastStateRL = 0;
volatile uint8_t lastStateRR = 0;

/*
   Bảng giải mã encoder X4.

   Giá trị:
    +1: quay thuận
    -1: quay nghịch
     0: trạng thái không hợp lệ hoặc không thay đổi
*/
const int8_t QUADRATURE_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

/* =========================================================
   HÀM ĐỌC TRẠNG THÁI ENCODER
   ========================================================= */

inline uint8_t readEncoderState(uint8_t pinA, uint8_t pinB) {
  uint8_t stateA = digitalReadFast(pinA);
  uint8_t stateB = digitalReadFast(pinB);

  return (stateA << 1) | stateB;
}

/* =========================================================
   INTERRUPT ENCODER TRƯỚC TRÁI
   ========================================================= */

void updateEncoderFL() {
  uint8_t currentState = readEncoderState(ENC_FL_A, ENC_FL_B);
  uint8_t transition = (lastStateFL << 2) | currentState;

  encoderFL += QUADRATURE_TABLE[transition];
  lastStateFL = currentState;
}

/* =========================================================
   INTERRUPT ENCODER TRƯỚC PHẢI
   ========================================================= */

void updateEncoderFR() {
  uint8_t currentState = readEncoderState(ENC_FR_A, ENC_FR_B);
  uint8_t transition = (lastStateFR << 2) | currentState;

  encoderFR += QUADRATURE_TABLE[transition];
  lastStateFR = currentState;
}

/* =========================================================
   INTERRUPT ENCODER SAU TRÁI
   ========================================================= */

void updateEncoderRL() {
  uint8_t currentState = readEncoderState(ENC_RL_A, ENC_RL_B);
  uint8_t transition = (lastStateRL << 2) | currentState;

  encoderRL += QUADRATURE_TABLE[transition];
  lastStateRL = currentState;
}

/* =========================================================
   INTERRUPT ENCODER SAU PHẢI
   ========================================================= */

void updateEncoderRR() {
  uint8_t currentState = readEncoderState(ENC_RR_A, ENC_RR_B);
  uint8_t transition = (lastStateRR << 2) | currentState;

  encoderRR += QUADRATURE_TABLE[transition];
  lastStateRR = currentState;
}

/* =========================================================
   RESET ENCODER
   ========================================================= */

void resetEncoders() {
  noInterrupts();

  encoderFL = 0;
  encoderFR = 0;
  encoderRL = 0;
  encoderRR = 0;

  interrupts();
}

/* =========================================================
   SETUP
   ========================================================= */

void setup() {
  Serial.begin(115200);

  // Nếu encoder đã có điện trở kéo lên ngoài, có thể đổi
  // INPUT_PULLUP thành INPUT.
  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);

  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);

  pinMode(ENC_RL_A, INPUT_PULLUP);
  pinMode(ENC_RL_B, INPUT_PULLUP);

  pinMode(ENC_RR_A, INPUT_PULLUP);
  pinMode(ENC_RR_B, INPUT_PULLUP);

  // Đọc trạng thái ban đầu
  lastStateFL = readEncoderState(ENC_FL_A, ENC_FL_B);
  lastStateFR = readEncoderState(ENC_FR_A, ENC_FR_B);
  lastStateRL = readEncoderState(ENC_RL_A, ENC_RL_B);
  lastStateRR = readEncoderState(ENC_RR_A, ENC_RR_B);

  // Ngắt trên cả hai kênh A và B để đọc X4
  attachInterrupt(
    digitalPinToInterrupt(ENC_FL_A),
    updateEncoderFL,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(ENC_FL_B),
    updateEncoderFL,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_FR_A),
    updateEncoderFR,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(ENC_FR_B),
    updateEncoderFR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_RL_A),
    updateEncoderRL,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(ENC_RL_B),
    updateEncoderRL,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_RR_A),
    updateEncoderRR,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(ENC_RR_B),
    updateEncoderRR,
    CHANGE
  );

  resetEncoders();

  Serial.println("Bat dau doc 4 encoder X4...");
}

/* =========================================================
   LOOP
   ========================================================= */

void loop() {
  static uint32_t lastPrintTime = 0;

  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();

    int32_t countFL;
    int32_t countFR;
    int32_t countRL;
    int32_t countRR;

    // Sao chép an toàn dữ liệu đang được ISR cập nhật
    noInterrupts();

    countFL = encoderFL;
    countFR = encoderFR;
    countRL = encoderRL;
    countRR = encoderRR;

    interrupts();

    Serial.print("FL: ");
    Serial.print(countFL);

    Serial.print("\tFR: ");
    Serial.print(countFR);

    Serial.print("\tRL: ");
    Serial.print(countRL);

    Serial.print("\tRR: ");
    Serial.println(countRR);
  }

  // Gửi ký tự R trên Serial Monitor để reset encoder
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'R' || command == 'r') {
      resetEncoders();
      Serial.println("Da reset 4 encoder.");
    }
  }
}