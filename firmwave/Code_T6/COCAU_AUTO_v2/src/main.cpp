/*
========================================================
        ROBOT ACTUATOR CONTROLLER - ESP32
========================================================

CHỨC NĂNG:

1. Nhận UART từ TEENSY

    "POINT_A"
    "POINT_B"
    "DROP"

2. Điều khiển:
    - 2 động cơ step
    - 4 relay khí nén
    - đọc 8 sensor qua PCF8574

3. Quy trình:

POINT A:
    - hạ step
    - sensor P0-P3 chạm
    - kích RL1 RL2
    - nâng lên
    - UART gửi DONE_A

POINT B:
    - hạ step
    - sensor P4-P7 chạm
    - kích RL3 RL4
    - nâng lên
    - UART gửi DONE_B

DROP:
    - kích relay thả khối
    - UART gửi FINISH

========================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <AccelStepper.h>

// =====================================================
// UART
// =====================================================

#define RXD2 16
#define TXD2 17

// =====================================================
// I2C
// =====================================================

#define SDA_PIN 21
#define SCL_PIN 22

// =====================================================
// PCF8574
// =====================================================

PCF8574 pcfSensor(0x20);

// =====================================================
// RELAY
// =====================================================

#define RL1 25
#define RL2 26
#define RL3 27
#define RL4 14

// =====================================================
// STEPPER A
// =====================================================

#define STEP_A_PUL 19
#define STEP_A_DIR 18

// =====================================================
// STEPPER B
// =====================================================

#define STEP_B_PUL 15
#define STEP_B_DIR 5

AccelStepper stepA(AccelStepper::DRIVER, STEP_A_PUL, STEP_A_DIR);
AccelStepper stepB(AccelStepper::DRIVER, STEP_B_PUL, STEP_B_DIR);

// =====================================================
// HOME POSITION
// =====================================================

// chỉnh theo thực tế

long HOME_A = 0;
long HOME_B = 0;

long DOWN_A = -12000;
long DOWN_B = -12000;

// =====================================================
// SENSOR
// =====================================================

bool sw[8];

// =====================================================
// RELAY FUNCTION
// =====================================================

void relayOffAll()
{
  digitalWrite(RL1, LOW);
  digitalWrite(RL2, LOW);
  digitalWrite(RL3, LOW);
  digitalWrite(RL4, LOW);
}

void relayA_ON()
{
  digitalWrite(RL1, HIGH);
  digitalWrite(RL2, HIGH);
}

void relayB_ON()
{
  digitalWrite(RL3, HIGH);
  digitalWrite(RL4, HIGH);
}

// =====================================================
// SENSOR
// =====================================================

void readSensor()
{
  sw[0] = !pcfSensor.digitalRead(P0);
  sw[1] = !pcfSensor.digitalRead(P1);
  sw[2] = !pcfSensor.digitalRead(P2);
  sw[3] = !pcfSensor.digitalRead(P3);

  sw[4] = !pcfSensor.digitalRead(P4);
  sw[5] = !pcfSensor.digitalRead(P5);
  sw[6] = !pcfSensor.digitalRead(P6);
  sw[7] = !pcfSensor.digitalRead(P7);
}

// =====================================================
// MOVE BOTH STEPPER
// =====================================================

void runStepper()
{
  while(stepA.distanceToGo() != 0 ||
        stepB.distanceToGo() != 0)
  {
    stepA.run();
    stepB.run();
  }
}

// =====================================================
// GO HOME
// =====================================================

void goHome()
{
  Serial.println("GO HOME");

  stepA.moveTo(HOME_A);
  stepB.moveTo(HOME_B);

  runStepper();
}

// =====================================================
// POINT A
// =====================================================

void processPointA()
{
  Serial.println("PROCESS A");

  // ==========================================
  // HẠ XUỐNG
  // ==========================================

  stepA.moveTo(DOWN_A);
  stepB.moveTo(DOWN_B);

  while(1)
  {
    stepA.run();
    stepB.run();

    readSensor();

    // P0 -> P3

    if(sw[0] &&
       sw[1] &&
       sw[2] &&
       sw[3])
    {
      Serial.println("A DETECT");

      break;
    }
  }

  // ==========================================
  // KÍCH KHÍ
  // ==========================================

  relayA_ON();

  delay(700);

  relayOffAll();

  delay(300);

  // ==========================================
  // NÂNG LÊN
  // ==========================================

  goHome();

  // ==========================================
  // UART DONE
  // ==========================================

  Serial2.println("DONE_A");

  Serial.println("DONE A");
}

// =====================================================
// POINT B
// =====================================================

void processPointB()
{
  Serial.println("PROCESS B");

  // ==========================================
  // HẠ XUỐNG
  // ==========================================

  stepA.moveTo(DOWN_A);
  stepB.moveTo(DOWN_B);

  while(1)
  {
    stepA.run();
    stepB.run();

    readSensor();

    // P4 -> P7

    if(sw[4] &&
       sw[5] &&
       sw[6] &&
       sw[7])
    {
      Serial.println("B DETECT");

      break;
    }
  }

  // ==========================================
  // KÍCH KHÍ
  // ==========================================

  relayB_ON();

  delay(700);

  relayOffAll();

  delay(300);

  // ==========================================
  // NÂNG LÊN
  // ==========================================

  goHome();

  // ==========================================
  // UART DONE
  // ==========================================

  Serial2.println("DONE_B");

  Serial.println("DONE B");
}

// =====================================================
// DROP ALL
// =====================================================

void dropAll()
{
  Serial.println("DROP");

  // THẢ TỪNG CẶP

  digitalWrite(RL1, HIGH);
  digitalWrite(RL2, HIGH);

  delay(700);

  digitalWrite(RL1, LOW);
  digitalWrite(RL2, LOW);

  delay(500);

  digitalWrite(RL3, HIGH);
  digitalWrite(RL4, HIGH);

  delay(700);

  digitalWrite(RL3, LOW);
  digitalWrite(RL4, LOW);

  delay(500);

  Serial2.println("FINISH");

  Serial.println("FINISH");
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // =================================================
  // I2C
  // =================================================

  Wire.begin(SDA_PIN, SCL_PIN);

  // =================================================
  // PCF8574
  // =================================================

  pcfSensor.begin();

  pcfSensor.pinMode(P0, INPUT_PULLUP);
  pcfSensor.pinMode(P1, INPUT_PULLUP);
  pcfSensor.pinMode(P2, INPUT_PULLUP);
  pcfSensor.pinMode(P3, INPUT_PULLUP);

  pcfSensor.pinMode(P4, INPUT_PULLUP);
  pcfSensor.pinMode(P5, INPUT_PULLUP);
  pcfSensor.pinMode(P6, INPUT_PULLUP);
  pcfSensor.pinMode(P7, INPUT_PULLUP);

  // =================================================
  // RELAY
  // =================================================

  pinMode(RL1, OUTPUT);
  pinMode(RL2, OUTPUT);
  pinMode(RL3, OUTPUT);
  pinMode(RL4, OUTPUT);

  relayOffAll();

  // =================================================
  // STEPPER
  // =================================================

  stepA.setMaxSpeed(4000);
  stepA.setAcceleration(2500);

  stepB.setMaxSpeed(4000);
  stepB.setAcceleration(2500);

  // =================================================
  // HOME
  // =================================================

  goHome();

  Serial.println("SYSTEM READY");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // =================================================
  // UART RECEIVE
  // =================================================

  if(Serial2.available())
  {
    String cmd = Serial2.readStringUntil('\n');

    cmd.trim();

    Serial.print("CMD: ");
    Serial.println(cmd);

    // ===============================================
    // POINT A
    // ===============================================

    if(cmd == "POINT_A")
    {
      processPointA();
    }

    // ===============================================
    // POINT B
    // ===============================================

    else if(cmd == "POINT_B")
    {
      processPointB();
    }

    // ===============================================
    // DROP
    // ===============================================

    else if(cmd == "DROP")
    {
      dropAll();
    }
  }
}