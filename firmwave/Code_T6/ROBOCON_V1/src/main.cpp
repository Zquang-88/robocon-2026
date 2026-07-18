#include <Arduino.h>
#include <Encoder.h>

// =====================================================
// PWM SETTINGS
// =====================================================
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// =====================================================
// MOTOR FRONT LEFT
// =====================================================
#define FL_RPWM 28
#define FL_LPWM 29

Encoder encFL(8, 38);

// =====================================================
// MOTOR FRONT RIGHT
// =====================================================
#define FR_RPWM 14
#define FR_LPWM 15

Encoder encFR(6, 7);

// =====================================================
// MOTOR BACK LEFT
// =====================================================
#define BL_RPWM 11
#define BL_LPWM 39

Encoder encBL(2, 3);

// =====================================================
// MOTOR BACK RIGHT
// =====================================================
#define BR_RPWM 13
#define BR_LPWM 12

Encoder encBR(4, 5);

// =====================================================
// MOTOR FUNCTION
// =====================================================
void setMotor(int rpwmPin, int lpwmPin, int pwm)
{
  pwm = constrain(pwm, -255, 255);

  // forward
  if (pwm > 0)
  {
    analogWrite(rpwmPin, pwm);
    analogWrite(lpwmPin, 0);
  }

  // reverse
  else if (pwm < 0)
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -pwm);
  }

  // stop
  else
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // PWM setup
  analogWriteResolution(PWM_RES);

  analogWriteFrequency(FL_RPWM, PWM_FREQ);
  analogWriteFrequency(FL_LPWM, PWM_FREQ);

  analogWriteFrequency(FR_RPWM, PWM_FREQ);
  analogWriteFrequency(FR_LPWM, PWM_FREQ);

  analogWriteFrequency(BL_RPWM, PWM_FREQ);
  analogWriteFrequency(BL_LPWM, PWM_FREQ);

  analogWriteFrequency(BR_RPWM, PWM_FREQ);
  analogWriteFrequency(BR_LPWM, PWM_FREQ);

  // pin mode
  pinMode(FL_RPWM, OUTPUT);
  pinMode(FL_LPWM, OUTPUT);

  pinMode(FR_RPWM, OUTPUT);
  pinMode(FR_LPWM, OUTPUT);

  pinMode(BL_RPWM, OUTPUT);
  pinMode(BL_LPWM, OUTPUT);

  pinMode(BR_RPWM, OUTPUT);
  pinMode(BR_LPWM, OUTPUT);

  Serial.println("ROBOT READY");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  // =========================================
  // TEST: DI TIEN
  // =========================================
  setMotor(FL_RPWM, FL_LPWM, 120);
  setMotor(FR_RPWM, FR_LPWM, 120);
  setMotor(BL_RPWM, BL_LPWM, 120);
  setMotor(BR_RPWM, BR_LPWM, 120);

  // =========================================
  // READ ENCODER
  // =========================================
  Serial.print("FL: ");
  Serial.print(encFL.read());

  Serial.print("   FR: ");
  Serial.print(encFR.read());

  Serial.print("   BL: ");
  Serial.print(encBL.read());

  Serial.print("   BR: ");
  Serial.println(encBR.read());

  delay(100);
}