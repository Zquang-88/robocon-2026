#define BLYNK_TEMPLATE_ID "TMPL6khy0D5fT"
#define BLYNK_TEMPLATE_NAME "Hometownr"
#define BLYNK_AUTH_TOKEN "UT4u1he_-vIb6vftoCJqyhEVlmnFLghz"

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);

#include "DHTesp.h"
DHTesp dht;
#define dhtPin D0

// WiFi
char ssid[] = "DOREAMON 2.4GHz";
char pass[] = "NOBITA123";

// Nút nhấn
#define button1 D1
#define button2 D2
#define button3 D3
#define button4 D4

// Relay
#define relay1 D5
#define relay2 D6
#define relay3 D7
#define relay4 D8

unsigned long timeBlink=millis();
unsigned long timeDelay=millis();
boolean updateRelay=0;

WidgetLED LEDCONNECT(V0);

// Virtual pin
#define RELAY1 V1
#define RELAY2 V2
#define RELAY3 V5
#define RELAY4 V6
#define TEMP   V3
#define HUMI   V4

// Khi kết nối với server
BLYNK_CONNECTED() {
  Blynk.syncVirtual(RELAY1,RELAY2,RELAY3,RELAY4);
}

// Xử lý từ app Blynk
BLYNK_WRITE(RELAY1) { digitalWrite(relay1, param.asInt()); }
BLYNK_WRITE(RELAY2) { digitalWrite(relay2, param.asInt()); }
BLYNK_WRITE(RELAY3) { digitalWrite(relay3, param.asInt()); }
BLYNK_WRITE(RELAY4) { digitalWrite(relay4, param.asInt()); }

// Ngắt khi nhấn nút
ICACHE_RAM_ATTR void handleButton(){
  if(millis()-timeDelay>200){
    if(digitalRead(button1)==LOW) digitalWrite(relay1,!digitalRead(relay1));
    if(digitalRead(button2)==LOW) digitalWrite(relay2,!digitalRead(relay2));
    if(digitalRead(button3)==LOW) digitalWrite(relay3,!digitalRead(relay3));
    if(digitalRead(button4)==LOW) digitalWrite(relay4,!digitalRead(relay4));
    updateRelay=1;
    timeDelay=millis();
  }
}

void setup(){
  Serial.begin(115200);

  // Nút nhấn
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
  pinMode(button3,INPUT_PULLUP);
  pinMode(button4,INPUT_PULLUP);

  // Relay
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);
  pinMode(relay3,OUTPUT);
  pinMode(relay4,OUTPUT);

  // Ngắt ngoài
  attachInterrupt(button1,handleButton,FALLING);
  attachInterrupt(button2,handleButton,FALLING);
  attachInterrupt(button3,handleButton,FALLING);
  attachInterrupt(button4,handleButton,FALLING);

  // Cảm biến DHT
  dht.setup(dhtPin, DHTesp::DHT11);

  // LCD
  Wire.begin();
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Dang ket noi...");
  delay(1000);

  // Kết nối Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  lcd.clear();
  lcd.setCursor(0,0);        
  lcd.print("Nhiet do:");    
  lcd.setCursor(0,1);        
  lcd.print("Do am   :");
}

void loop(){
  Blynk.run();
  if(millis()-timeBlink>1000){
    // Nháy LED kết nối
    if(LEDCONNECT.getValue()) LEDCONNECT.off();
    else LEDCONNECT.on();

    // Đọc DHT
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();
    Serial.print(dht.getStatusString());
    Serial.print("\t");
    Serial.print(humidity, 1);
    Serial.print("\t\t");
    Serial.println(temperature, 1);

    if(dht.getStatusString()=="OK"){
      Blynk.virtualWrite(TEMP,temperature);
      Blynk.virtualWrite(HUMI,humidity);

      lcd.setCursor(9,0);        
      lcd.print(String(temperature,1)); 
      lcd.print((char)223);
      lcd.print("C  ");   
      lcd.setCursor(9,1);
      lcd.print(String(humidity,1));        
      lcd.print("%  ");
    }
    timeBlink=millis();
  }

  // Cập nhật trạng thái relay về app
  if(updateRelay==1){
    Blynk.virtualWrite(RELAY1,digitalRead(relay1));
    Blynk.virtualWrite(RELAY2,digitalRead(relay2));
    Blynk.virtualWrite(RELAY3,digitalRead(relay3));
    Blynk.virtualWrite(RELAY4,digitalRead(relay4));
    updateRelay=0;
  }
}