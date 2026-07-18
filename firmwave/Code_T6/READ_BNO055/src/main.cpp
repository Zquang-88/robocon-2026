#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno(55, 0x28, &Wire1);

void setup()
{
    Serial.begin(115200);

    Wire1.begin();

    if (!bno.begin())
    {
        Serial.println("BNO055 not found!");
        while (1);
    }

    bno.setExtCrystalUse(true);

    Serial.println("BNO055 Ready");
}

void loop()
{
    imu::Vector<3> euler =
        bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    float yaw = euler.x();

    Serial.print("Yaw: ");
    Serial.println(yaw);

    delay(10);
}