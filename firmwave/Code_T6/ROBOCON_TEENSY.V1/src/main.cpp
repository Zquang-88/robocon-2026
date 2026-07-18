#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// =====================================================
// BNO055
// =====================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

// =====================================================
// PWM
// =====================================================
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// =====================================================
// ENCODER PPR
// =====================================================
const float PPR = 320.0;

// =====================================================
// MOTOR DIR
// CHINH O DAY NEU SAI CHIEU
// =====================================================
#define FL_DIR  1
#define FR_DIR  1
#define BL_DIR  1
#define BR_DIR  1

// =====================================================
// PID MOTOR
// =====================================================
float Kp = 1.2;
float Ki = 0.15;
float Kd = 0.01;

// =====================================================
// PID YAW
// =====================================================
float yawKp = 2.0;
float yawKi = 0.5;
float yawKd = 0.05;

// =====================================================
// YAW
// =====================================================
float targetYaw = 0;
float currentYaw = 0;

float yawError = 0;
float yawPrevError = 0;
float yawIntegral = 0;
float yawDerivative = 0;

float wOutput = 0;

// =====================================================
// MOTOR STRUCT
// =====================================================
struct MotorPID
{
  float targetRPM = 0;
  float currentRPM = 0;

  float filterRPM = 0;

  float error = 0;
  float prevError = 0;

  float integral = 0;
  float derivative = 0;

  int pwm = 0;
};

// =====================================================
// MOTOR OBJECT
// =====================================================
MotorPID FL, FR, BL, BR;

// =====================================================
// FRONT LEFT
// =====================================================
#define FL_RPWM 26
#define FL_LPWM 27
Encoder encFL(18, 19);

// =====================================================
// FRONT RIGHT
// =====================================================
#define FR_RPWM 4
#define FR_LPWM 5
Encoder encFR(22, 23);

// =====================================================
// BACK LEFT
// =====================================================
#define BL_RPWM 24
#define BL_LPWM 25
Encoder encBL(14, 15);

// =====================================================
// BACK RIGHT
// =====================================================
#define BR_RPWM 26
#define BR_LPWM 27
Encoder encBR(28, 29);

// =====================================================
// COMMAND
// =====================================================
float vx = 0;
float vy = 0;
float wz = 0;

unsigned long lastCmdTime = 0;

// =====================================================
// MOTOR FUNCTION
// =====================================================
void setMotor(int rpwmPin, int lpwmPin, int pwm)
{
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0)
  {
    analogWrite(rpwmPin, pwm);
    analogWrite(lpwmPin, 0);
  }

  else if (pwm < 0)
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -pwm);
  }

  else
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

// =====================================================
// YAW PID
// =====================================================
void updateYawPID()
{
  imu::Vector<3> euler =
    bno.getVector(
      Adafruit_BNO055::VECTOR_EULER
    );

  currentYaw = euler.x();

  yawError =
    targetYaw - currentYaw;

  // wrap angle
  if (yawError > 180)
    yawError -= 360;

  if (yawError < -180)
    yawError += 360;

  float dt = 0.05;

  yawIntegral += yawError * dt;

  yawDerivative =
    (yawError - yawPrevError) / dt;

  wOutput =
    yawKp * yawError +
    yawKi * yawIntegral +
    yawKd * yawDerivative;

  yawPrevError = yawError;
}

// =====================================================
// PID UPDATE
// =====================================================
void updatePID(MotorPID &m)
{
  float dt = 0.05;

  m.error =
    m.targetRPM - m.filterRPM;

  m.integral += m.error * dt;

  // anti windup
  m.integral =
    constrain(m.integral, -300, 300);

  m.derivative =
    (m.error - m.prevError) / dt;

  float output =
    Kp * m.error +
    Ki * m.integral +
    Kd * m.derivative;

  m.prevError = m.error;

  m.pwm = constrain(output, -255, 255);

  // deadzone
  if (m.pwm > 0 && m.pwm < 70)
    m.pwm = 70;

  if (m.pwm < 0 && m.pwm > -70)
    m.pwm = -70;
}

// =====================================================
// OMNI KINEMATICS
// =====================================================
void moveOmni(float vx, float vy, float w)
{
  FL.targetRPM =  vy + vx + w;
  FR.targetRPM =  vy - vx - w;
  BL.targetRPM =  vy - vx + w;
  BR.targetRPM =  vy + vx - w;

  // normalize
  float maxValue = max(
    max(abs(FL.targetRPM), abs(FR.targetRPM)),
    max(abs(BL.targetRPM), abs(BR.targetRPM))
  );

  if (maxValue > 255)
  {
    FL.targetRPM *= 255.0 / maxValue;
    FR.targetRPM *= 255.0 / maxValue;
    BL.targetRPM *= 255.0 / maxValue;
    BR.targetRPM *= 255.0 / maxValue;
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);


  // =====================================
  // PWM
  // =====================================
  analogWriteResolution(PWM_RES);

  analogWriteFrequency(FL_RPWM, PWM_FREQ);
  analogWriteFrequency(FL_LPWM, PWM_FREQ);

  analogWriteFrequency(FR_RPWM, PWM_FREQ);
  analogWriteFrequency(FR_LPWM, PWM_FREQ);

  analogWriteFrequency(BL_RPWM, PWM_FREQ);
  analogWriteFrequency(BL_LPWM, PWM_FREQ);

  analogWriteFrequency(BR_RPWM, PWM_FREQ);
  analogWriteFrequency(BR_LPWM, PWM_FREQ);

  // =====================================
  // INPUT PULLUP
  // =====================================
  pinMode(38, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  // =====================================
  // BNO055
  // =====================================
Wire2.begin();
Wire2.setClock(100000);

if (!bno.begin())
{
  Serial.println("BNO055 ERROR");

  while(1);
}

  bno.setExtCrystalUse(true);

  imu::Vector<3> euler =
    bno.getVector(
      Adafruit_BNO055::VECTOR_EULER
    );

  targetYaw = euler.x();

  Serial.println("OMNI READY");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  static unsigned long lastPID = millis();

  // =====================================
  // UART COMMAND
  // =====================================
  if (Serial.available())
  {
    char cmd = Serial.read();
    // Serial.print("NHAN: ");
    // Serial.print(cmd);
    // Serial.println();
    lastCmdTime = millis();

    switch(cmd)
    {
      case 'F':
        vx = 0;
        vy = 150;
        break;

      case 'B':
        vx = 0;
        vy = -150;
        break;

      case 'L':
        vx = -150;
        vy = 0;
        break;

      case 'R':
        vx = 150;
        vy = 0;
        break;

      case 'Q':
        targetYaw -= 15;
        break;

      case 'E':
        targetYaw += 15;
        break;

      case 'S':
        vx = 0;
        vy = 0;
        break;
    }
  }

  // =====================================
  // FAIL SAFE
  // =====================================
  if (millis() - lastCmdTime > 500)
  {
    vx = 0;
    vy = 0;
  }

  // =====================================
  // PID LOOP
  // =====================================
  if (millis() - lastPID >= 50)
  {
    // =================================
    // ENCODER
    // =================================
    static long prevFL = 0;
    static long prevFR = 0;
    static long prevBL = 0;
    static long prevBR = 0;

    long nowFL = encFL.read();
    long nowFR = encFR.read();
    long nowBL = encBL.read();
    long nowBR = encBR.read();

    long dFL = nowFL - prevFL;
    long dFR = nowFR - prevFR;
    long dBL = nowBL - prevBL;
    long dBR = nowBR - prevBR;

    prevFL = nowFL;
    prevFR = nowFR;
    prevBL = nowBL;
    prevBR = nowBR;

    float rpmFL =
      (dFL * 1200.0) / PPR;

    float rpmFR =
      (dFR * 1200.0) / PPR;

    float rpmBL =
      (dBL * 1200.0) / PPR;

    float rpmBR =
      (dBR * 1200.0) / PPR;

    // =================================
    // LOW PASS FILTER
    // =================================
    FL.filterRPM =
      0.7 * FL.filterRPM +
      0.3 * rpmFL;

    FR.filterRPM =
      0.7 * FR.filterRPM +
      0.3 * rpmFR;

    BL.filterRPM =
      0.7 * BL.filterRPM +
      0.3 * rpmBL;

    BR.filterRPM =
      0.7 * BR.filterRPM +
      0.3 * rpmBR;

    // =================================
    // YAW PID
    // =================================
    updateYawPID();

    // =================================
    // OMNI
    // =================================
    moveOmni(vx, vy, wOutput);

    // =================================
    // PID
    // =================================
    updatePID(FL);
    updatePID(FR);
    updatePID(BL);
    updatePID(BR);

    // =================================
    // MOTOR
    // =================================
    setMotor(
      FL_RPWM,
      FL_LPWM,
      FL.pwm * FL_DIR
    );

    setMotor(
      FR_RPWM,
      FR_LPWM,
      FR.pwm * FR_DIR
    );

    setMotor(
      BL_RPWM,
      BL_LPWM,
      BL.pwm * BL_DIR
    );

    setMotor(
      BR_RPWM,
      BR_LPWM,
      BR.pwm * BR_DIR
    );

    // =================================
    // DEBUG
    // =================================
    Serial.print("Yaw:");
    Serial.print(currentYaw);

    Serial.print(" FL:");
    Serial.print(FL.filterRPM);

    Serial.print(" FR:");
    Serial.print(FR.filterRPM);

    Serial.print(" BL:");
    Serial.print(BL.filterRPM);

    Serial.print(" BR:");
    Serial.println(BR.filterRPM);

    lastPID = millis();
  }
}