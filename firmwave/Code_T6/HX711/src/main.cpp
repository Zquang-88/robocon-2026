#include "HX711.h"

#define DT  3
#define SCK 2

HX711 scale;
float calibration_factor = -7050; // giá trị khởi đầu

void setup() {
  Serial.begin(9600);
  scale.begin(DT, SCK);
  scale.set_scale();
  scale.tare();
  Serial.println("Bắt đầu cân... Đặt vật chuẩn để hiệu chuẩn.");
}

void loop() {
  Serial.print("Đọc: ");
  Serial.print(scale.get_units(10));
  Serial.print(" g");  
  Serial.print("  |  Hệ số hiện tại: ");
  Serial.println(calibration_factor);

  // Điều chỉnh bằng cách nhập từ Serial Monitor
  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == '+') calibration_factor += 100;
    else if (temp == '-') calibration_factor -= 100;
    scale.set_scale(calibration_factor);
  }
}
