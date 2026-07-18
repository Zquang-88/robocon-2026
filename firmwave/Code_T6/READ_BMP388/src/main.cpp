#include <Wire.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA 1013.25

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("BMP388 Test - Teensy 4.1 Wire2");
    Serial.println("SDA = 25, SCL = 24");

    // Teensy 4.1:
    // Wire2 SDA = pin 25
    // Wire2 SCL = pin 24
    Wire2.begin();
    Wire2.setClock(400000);

    if (!bmp.begin_I2C(0x76, &Wire2))
    {
        Serial.println("Khong thay BMP388 tai 0x76, thu 0x77...");

        if (!bmp.begin_I2C(0x77, &Wire2))
        {
            Serial.println("Khong tim thay BMP388!");
            Serial.println("Kiem tra lai day SDA/SCL/VCC/GND/CSB/SDO");
            while (1);
        }
        else
        {
            Serial.println("Da tim thay BMP388 tai dia chi 0x77");
        }
    }
    else
    {
        Serial.println("Da tim thay BMP388 tai dia chi 0x76");
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Serial.println("BMP388 san sang!");
}

void loop()
{
    if (!bmp.performReading())
    {
        Serial.println("Loi doc BMP388");
        delay(200);
        return;
    }

    float temperature = bmp.temperature;
    float pressure_hPa = bmp.pressure / 100.0;
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.print("Nhiet do: ");
    Serial.print(temperature);
    Serial.print(" C | ");

    Serial.print("Ap suat: ");
    Serial.print(pressure_hPa);
    Serial.print(" hPa | ");

    Serial.print("Do cao: ");
    Serial.print(altitude);
    Serial.println(" m");

    delay(100);
}