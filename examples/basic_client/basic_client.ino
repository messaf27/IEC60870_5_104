#include <WiFi.h>
#include "IEC60870_5_104.h"

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

IEC104Client iec104;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting IEC 60870-5-104 Client with FreeRTOS...");
  
  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  // Вывод IP адреса
  
  // Настройка параметров ДО запуска задачи
  iec104.setTaskStackSize(8192);
  iec104.setTaskPriority(2);
  iec104.setKeepAliveInterval(15000);
  iec104.setCommonAddress(1);
  
  // Настройка callback'ов
  iec104.onConnected([]() {
    Serial.println("IEC 104 Connected!");
  });
  
  iec104.onDisconnected([]() {
    Serial.println("IEC 104 Disconnected!");
  });
  
  iec104.onASDUReceived([](ASDUInfo& info) {
    Serial.printf("ASDU: Type=%d, Objects=%d, COT=%d, CA=%d\n", 
                  info.typeID, info.numberOfObjects, info.causeOfTransmission, info.commonAddress);
  });
  
  // Запуск задачи FreeRTOS
  iec104.start();
  
  // Подключение к серверу
  if (iec104.connect("192.168.1.100", 2404)) {
    Serial.println("Connection command queued");
  } else {
    Serial.println("Failed to queue connection command");
  }
}

void loop() {
  // Основной поток свободен для других задач
  delay(5000);
  Serial.println("Main task running...");
}