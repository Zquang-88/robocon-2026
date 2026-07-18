// ===== THÔNG TIN BLYNK CLOUD =====
#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "YOUR_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"

#include <BlynkEdgent.h>
#include <DHT.h>

// ===== KHAI BÁO CHÂN PHẦN CỨNG =====
#define RELAY1 23
#define RELAY2 22
#define RELAY3 21

#define LED_WIFI 15
#define LED_TEMP 2

#define DHTPIN 19
#define DHTTYPE DHT11

#define RESET_BUTTON 4     // Nút nhấn reset WiFi

// ===== ĐỐI TƯỢNG =====
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

// Biến cho nút reset
unsigned long pressTime = 0;
bool resetFlag = false;

// ===== ĐIỀU KHIỂN RELAY TỪ APP =====
BLYNK_WRITE(V0) {
  int value = param.asInt();
  digitalWrite(RELAY1, value);
}

BLYNK_WRITE(V1) {
  int value = param.asInt();
  digitalWrite(RELAY2, value);
}

BLYNK_WRITE(V2) {
  int value = param.asInt();
  digitalWrite(RELAY3, value);
}

// ===== HÀM ĐỌC DHT11 =====
void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Loi doc cam bien DHT11!");
    return;
  }

  Serial.print("Nhiet do: ");
  Serial.println(t);
  Serial.print("Do am: ");
  Serial.println(h);

  Blynk.virtualWrite(V5, t);
  Blynk.virtualWrite(V6, h);

  // ---- Xử lý LED cảnh báo nhiệt ----
  if (t >= 50) {
    digitalWrite(LED_TEMP, HIGH);        // Quá nóng -> sáng liên tục
  } 
  else if (t >= 40) {
    digitalWrite(LED_TEMP, millis() % 500 < 250);   // Nhấp nháy
  } 
  else {
    digitalWrite(LED_TEMP, LOW);         // Bình thường
  }
}

// ===== TRẠNG THÁI KẾT NỐI =====
BLYNK_CONNECTED() {
  Serial.println("Da ket noi Blynk!");
  digitalWrite(LED_WIFI, HIGH);   // Kết nối OK
}

BLYNK_DISCONNECTED() {
  Serial.println("Mat ket noi Blynk!");
  digitalWrite(LED_WIFI, millis() % 500 < 250);   // Nhấp nháy
}

// ===== KIỂM TRA NÚT RESET WIFI =====
void checkResetButton() {
  if (digitalRead(RESET_BUTTON) == LOW) {

    if (pressTime == 0) {
      pressTime = millis();
    }

    // Giữ nút 7 giây để reset
    if (millis() - pressTime > 7000 && !resetFlag) {

      Serial.println("RESET WIFI & BLYNK CONFIG...");

      digitalWrite(LED_WIFI, LOW);

      BlynkEdgent.resetConfig();   // Xóa WiFi cũ

      delay(1000);
      ESP.restart();               // Khởi động lại ESP32

      resetFlag = true;
    }

  } else {
    pressTime = 0;
    resetFlag = false;
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);

  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_TEMP, OUTPUT);

  pinMode(RESET_BUTTON, INPUT_PULLUP);

  // Mặc định tắt relay
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);

  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_TEMP, LOW);

  dht.begin();

  Serial.println("Khoi dong Blynk Edgent...");
  BlynkEdgent.begin();

  // Đọc DHT mỗi 2 giây
  timer.setInterval(2000L, readDHT);
}

// ===== LOOP =====
void loop() {
  BlynkEdgent.run();
  timer.run();

  checkResetButton();
}
