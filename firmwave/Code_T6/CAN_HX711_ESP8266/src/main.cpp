#include <Arduino.h>
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------------
// Cấu hình chân
// -------------------------
#define DOUT D5
#define CLK  D6

HX711 scale;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); 
// -------------------------
// Hệ số hiệu chuẩn (calibration factor)
// -------------------------
 float calibration_factor = 110670; // Sẽ điều chỉnh sau

void setup() {
  Serial.begin(115200);
  scale.begin(DOUT, CLK);

  // Khởi động màn hình OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Không tìm thấy màn hình OLED!");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Thiết lập tỉ lệ
  scale.set_scale(calibration_factor);
  scale.tare();  // Đặt 0 cho cân

  Serial.println("Bắt đầu đo khối lượng...");
}

void loop() {
  float weight = scale.get_units(20); // Đọc trung bình 10 lần

  Serial.print("Khối lượng: ");
  Serial.print(weight * 10000, 2);
  Serial.println(" g");

  display.clearDisplay();
  display.setCursor(10, 20);
  display.setTextSize(2);
  display.print("  HELLO\n");
  display.print(  weight *10000, 2);
  display.print(" kg");
  display.display();

  delay(500);
}
