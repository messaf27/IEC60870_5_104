#include <WiFi.h>
#include "IEC60870_5_104.h"

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

IEC104Client iec104;

void setup() {
  Serial.begin(115200);
  
  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  
  // Настройка callback'ов
  iec104.onConnected([]() {
    Serial.println("IEC 104 Connected!");
  });
  
  iec104.onDisconnected([]() {
    Serial.println("IEC 104 Disconnected!");
  });
  
  iec104.onASDUReceived([](ASDUInfo& info) {
    Serial.printf("ASDU Received - Type: %d, Objects: %d, COT: %d\n", 
                  info.typeID, info.numberOfObjects, info.causeOfTransmission);
  });
  
  // Запуск задачи FreeRTOS
  iec104.start();
  
  // Подключение к серверу
  if (iec104.connect("192.168.1.100", 2404)) {
    Serial.println("Connecting to IEC 104 server...");
  } else {
    Serial.println("Failed to queue connection command");
  }
}

void loop() {
  // Основной поток может выполнять другие задачи
  // Все операции IEC 104 выполняются в отдельной задаче FreeRTOS
  delay(1000);
  
  // Пример отправки команды раз в 10 секунд
  static unsigned long lastCommand = 0;
  if (millis() - lastCommand > 10000) {
    iec104.sendInterrogationCommand();
    lastCommand = millis();
  }
}