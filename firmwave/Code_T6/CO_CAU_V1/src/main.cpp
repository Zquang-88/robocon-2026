#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <AccelStepper.h>
#include <U8g2lib.h>
#include <Encoder.h>
HardwareSerial TeensyUART(2);
PCF8574 pcfSensor(0x20);
PCF8574 pcfValve(0x21);
#define STEP_Z       18
#define DIR_Z        19

#define STEP_ROTATE  5
#define DIR_ROTATE   15

AccelStepper stepZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);
AccelStepper stepRotate(AccelStepper::DRIVER, STEP_ROTATE, DIR_ROTATE);
// =====================================================
// ROTARY ENCODER
// =====================================================

#define ENC_A 34
#define ENC_B 35
#define ENC_SW 23

Encoder myEnc(ENC_A, ENC_B);

// =====================================================
// OLED
// =====================================================

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// =====================================================
// PICK SENSOR
// =====================================================

bool sw1, sw2, sw3, sw4;
bool sw5, sw6, sw7, sw8;

// =====================================================
// VALVE OUTPUT
// =====================================================

#define VALVE_A 0
#define VALVE_B 1
#define VALVE_C 2
#define VALVE_D 3
// =====================================================
// PARAMETER
// =====================================================

long zDownPosition = -25000;
long zHomePosition = 0;

long rotateCenter = 0;
long rotateLeft = -1200;
long rotateRight = 1200;

int zSpeed = 3000;
int zAccel = 2000;

int rotateSpeed = 1500;
int rotateAccel = 1200;

// =====================================================
// STATE MACHINE
// =====================================================

enum State
{
  IDLE,
  WAIT_COMMAND,
  LOWER_Z,
  ALIGN_POSITION,
  CHECK_SENSOR,
  LOCK_BLOCK,
  VERIFY_LOCK,
  LIFT_Z,
  SEND_ACK,
  ERROR_STATE
};

State machineState = IDLE;
// =====================================================
// TIMER
// =====================================================

unsigned long stateTimer = 0;

// =====================================================
// MENU
// =====================================================

int menuIndex = 0;
long oldEnc = 0;

// =====================================================
// FUNCTION
// =====================================================

void readSensors();
void controlValve(uint8_t valve, bool state);
void updateOLED();
void processUART();
void runStateMachine();
void moveZDown();
void moveZUp();
void rotateToCenter();
void sendACK();
void stopAllValve();

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  TeensyUART.begin(115200, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);

  // =============================
  // PCF8574
  // =============================

  pcfSensor.begin();
  pcfValve.begin();

  for(int i=0;i<8;i++)
  {
    pcfSensor.pinMode(i, INPUT);
  }

  for(int i=0;i<8;i++)
  {
    pcfValve.pinMode(i, OUTPUT);
    pcfValve.digitalWrite(i, HIGH);
  }

  // =============================
  // STEPPER
  // =============================

  stepZ.setMaxSpeed(zSpeed);
  stepZ.setAcceleration(zAccel);

  stepRotate.setMaxSpeed(rotateSpeed);
  stepRotate.setAcceleration(rotateAccel);

  // =============================
  // OLED
  // =============================

  oled.begin();
  oled.clearBuffer();
  oled.sendBuffer();

  // =============================
  // ENCODER
  // =============================

  pinMode(ENC_SW, INPUT_PULLUP);

  // =============================
  // HOME POSITION
  // =============================

  stepZ.setCurrentPosition(0);
  stepRotate.setCurrentPosition(0);

  machineState = WAIT_COMMAND;
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  readSensors();

  processUART();

  runStateMachine();

  updateOLED();

  stepZ.run();
  stepRotate.run();
}
// =====================================================
// READ SENSOR
// =====================================================

void readSensors()
{
  sw1 = !pcfSensor.digitalRead(0);
  sw2 = !pcfSensor.digitalRead(1);
  sw3 = !pcfSensor.digitalRead(2);
  sw4 = !pcfSensor.digitalRead(3);

  sw5 = !pcfSensor.digitalRead(4);
  sw6 = !pcfSensor.digitalRead(5);
  sw7 = !pcfSensor.digitalRead(6);
  sw8 = !pcfSensor.digitalRead(7);
}

// =====================================================
// VALVE CONTROL
// =====================================================

void controlValve(uint8_t valve, bool state)
{
  pcfValve.digitalWrite(valve, !state);
}

void stopAllValve()
{
  for(int i=0;i<8;i++)
  {
    pcfValve.digitalWrite(i, HIGH);
  }
}
// =====================================================
// UART PROCESS
// =====================================================

void processUART()
{
  if(TeensyUART.available())
  {
    String cmd = TeensyUART.readStringUntil('\n');

    cmd.trim();

    if(cmd == "PICK")
    {
      machineState = LOWER_Z;
    }

    if(cmd == "HOME")
    {
      stepZ.moveTo(0);
      stepRotate.moveTo(0);
    }
  }
}
// =====================================================
void runStateMachine()
{
  switch(machineState)
  {
    case WAIT_COMMAND:
    {
      break;
    }

    case LOWER_Z:
    {
      stepZ.moveTo(zDownPosition);

      if(stepZ.distanceToGo() == 0)
      {
        machineState = ALIGN_POSITION;
      }

      break;
    }

    case ALIGN_POSITION:
    {
      stepRotate.moveTo(rotateCenter);

      if(stepRotate.distanceToGo() == 0)
      {
        machineState = CHECK_SENSOR;
      }

      break;
    }

    case CHECK_SENSOR:
    {
      bool leftOK  = sw1 && sw2;
      bool rightOK = sw3 && sw4;

      if(leftOK)
      {
        controlValve(VALVE_A, true);
      }

      if(rightOK)
      {
        controlValve(VALVE_B, true);
      }

      if(leftOK || rightOK)
      {
        stateTimer = millis();
        machineState = VERIFY_LOCK;
      }
      else
      {
        machineState = ERROR_STATE;
      }

      break;
    }

    case VERIFY_LOCK:
    {
      if(millis() - stateTimer > 300)
      {
        machineState = LIFT_Z;
      }

      break;
    }

    case LIFT_Z:
    {
      stepZ.moveTo(zHomePosition);

      if(stepZ.distanceToGo() == 0)
      {
        machineState = SEND_ACK;
      }

      break;
    }

    case SEND_ACK:
    {
      sendACK();

      machineState = WAIT_COMMAND;

      break;
    }

    case ERROR_STATE:
    {
      stopAllValve();

      TeensyUART.println("PICK_FAIL");

      machineState = WAIT_COMMAND;

      break;
    }
  }
}
// =====================================================
// SEND ACK
// =====================================================

void sendACK()
{
  TeensyUART.println("PICK_OK");
}

// =====================================================
// OLED MENU
// =====================================================

void updateOLED()
{
  long newEnc = myEnc.read() / 4;

  if(newEnc != oldEnc)
  {
    menuIndex += (newEnc - oldEnc);
    oldEnc = newEnc;
  }

  oled.clearBuffer();

  oled.setFont(u8g2_font_6x12_tf);

  oled.drawStr(0,10,"PICK SYSTEM");

  oled.setCursor(0,25);
  oled.print("Z POS:");
  oled.print(stepZ.currentPosition());

  oled.setCursor(0,40);
  oled.print("ROT:");
  oled.print(stepRotate.currentPosition());

  oled.setCursor(0,55);
  oled.print("MENU:");
  oled.print(menuIndex);

  oled.sendBuffer();
}