#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <PCF8574.h>

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

// ================== MOTOR A ==================
#define STEP_A 18
#define DIR_A  19

// ================== MOTOR B ==================
#define STEP_B 5
#define DIR_B  15

AccelStepper motorA(AccelStepper::DRIVER, STEP_A, DIR_A);
AccelStepper motorB(AccelStepper::DRIVER, STEP_B, DIR_B);

MultiStepper steppers;

long target[2];

// =====================================================
// T8-8 + Microstep 16
// 3200 step/rev
// 8 mm/rev
// = 400 step/mm
// =====================================================

#define STEP_PER_MM 400

// =====================================================
// HOME OFFSET
// =====================================================

#define HOME_A_STEP -20000
#define HOME_B_STEP  0


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

void setup()
{
    Serial.begin(115200);

    motorA.setMaxSpeed(8000);
    motorA.setAcceleration(3000);

    motorB.setMaxSpeed(8000);
    motorB.setAcceleration(3000);

    steppers.addStepper(motorA);
    steppers.addStepper(motorB);

    // ================= HOME =================
    homeSystem();
}

// =====================================================
// HOME
// =====================================================

void homeSystem()
{
    motorA.moveTo(HOME_A_STEP);
    motorB.moveTo(HOME_B_STEP);

    while(
        motorA.distanceToGo() != 0 ||
        motorB.distanceToGo() != 0
    )
    {
        motorA.run();
        motorB.run();
    }

    motorA.setCurrentPosition(0);
    motorB.setCurrentPosition(0);

    Serial.println("HOME DONE");
}

// =====================================================
// HẠ XUỐNG
// =====================================================

void liftDown(float mm)
{
    long step = mm * STEP_PER_MM;

    target[0] = motorA.currentPosition() + step;
    target[1] = motorB.currentPosition() + step;

    steppers.moveTo(target);

    while(
        motorA.distanceToGo() != 0 ||
        motorB.distanceToGo() != 0
    )
    {
        steppers.run();
    }
}

// =====================================================
// NÂNG LÊN
// =====================================================

void liftUp(float mm)
{
    long step = mm * STEP_PER_MM;

    target[0] = motorA.currentPosition() - step;
    target[1] = motorB.currentPosition() - step;

    steppers.moveTo(target);

    while(
        motorA.distanceToGo() != 0 ||
        motorB.distanceToGo() != 0
    )
    {
        steppers.run();
    }
}

// =====================================================

void loop()
{
    // delay(1000);

    // liftDown(100); // hạ 100mm

    // delay(1000);

    // liftUp(100);   // nâng 100mm

    // delay(3000);
}
