#ifndef IEC60870_5_104_H
#define IEC60870_5_104_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Типы ASDU
enum ASDUType {
    M_SP_NA_1 = 1,      // Single point information
    M_DP_NA_1 = 3,      // Double point information
    M_ST_NA_1 = 5,      // Step position information
    M_ME_NA_1 = 9,      // Measured value, normalized value
    M_ME_NB_1 = 11,     // Measured value, scaled value
    M_ME_NC_1 = 13,     // Measured value, short floating point
    M_IT_NA_1 = 15,     // Integrated totals
    M_PS_NA_1 = 20,     // Packed single point information
    M_ME_ND_1 = 21,     // Measured value, normalized value without quality
    C_SC_NA_1 = 45,     // Single command
    C_DC_NA_1 = 46,     // Double command
    C_RC_NA_1 = 47,     // Regulating step command
    C_SE_NA_1 = 48,     // Set point command, normalized value
    C_SE_NB_1 = 49,     // Set point command, scaled value
    C_SE_NC_1 = 50,     // Set point command, short floating point
    C_BO_NA_1 = 51,     // Bitstring of 32 bits
    C_IC_NA_1 = 100,    // Interrogation command
    C_CI_NA_1 = 101,    // Counter interrogation command
    C_RD_NA_1 = 102,    // Read command
    C_CS_NA_1 = 103,    // Clock synchronization command
    C_TS_NA_1 = 104,    // Test command
    C_RP_NA_1 = 105,    // Reset process command
    C_CD_NA_1 = 106,    // Delay acquisition command
    C_TS_TA_1 = 107     // Test command with time tag CP56Time2a
};

// Причины передачи (Cause of Transmission)
enum CauseOfTransmission {
    PERIODIC = 1,
    BACKGROUND_SCAN = 2,
    SPONTANEOUS = 3,
    INITIALIZED = 4,
    REQUEST = 5,
    ACTIVATION = 6,
    ACTIVATION_CON = 7,
    DEACTIVATION = 8,
    DEACTIVATION_CON = 9,
    ACTIVATION_TERMINATION = 10,
    RETURN_INFO_REMOTE = 11,
    RETURN_INFO_LOCAL = 12,
    FILE_TRANSFER = 13,
    AUTHENTICATION = 14,
    MAINTENANCE_OF_AUTH_SESSION_KEY = 15,
    MAINTENANCE_OF_USER_ROLE_AND_UPDATE_KEY = 16,
    INTERROGATED_BY_STATION = 20,
    INTERROGATED_BY_GROUP_1 = 21,
    INTERROGATED_BY_GROUP_2 = 22,
    INTERROGATED_BY_GROUP_3 = 23,
    INTERROGATED_BY_GROUP_4 = 24,
    INTERROGATED_BY_GROUP_5 = 25,
    INTERROGATED_BY_GROUP_6 = 26,
    INTERROGATED_BY_GROUP_7 = 27,
    INTERROGATED_BY_GROUP_8 = 28,
    INTERROGATED_BY_GROUP_9 = 29,
    INTERROGATED_BY_GROUP_10 = 30,
    INTERROGATED_BY_GROUP_11 = 31,
    INTERROGATED_BY_GROUP_12 = 32,
    INTERROGATED_BY_GROUP_13 = 33,
    INTERROGATED_BY_GROUP_14 = 34,
    INTERROGATED_BY_GROUP_15 = 35,
    INTERROGATED_BY_GROUP_16 = 36,
    REQUESTED_BY_GENERAL_COUNTER = 37,
    REQUESTED_BY_GROUP_1_COUNTER = 38,
    REQUESTED_BY_GROUP_2_COUNTER = 39,
    REQUESTED_BY_GROUP_3_COUNTER = 40,
    REQUESTED_BY_GROUP_4_COUNTER = 41
};

// Структура для информации об ASDU
struct ASDUInfo {
    uint8_t typeID;
    uint8_t numberOfObjects;
    uint8_t causeOfTransmission;
    uint8_t originatorAddress;
    uint16_t commonAddress;
    uint8_t* data;
    uint16_t dataLength;
};

// Структура для команд
struct IECCommand {
    uint8_t type;
    uint8_t* data;
    uint16_t length;
};

// Типы команд
#define CMD_CONNECT 1
#define CMD_DISCONNECT 2
#define CMD_INTERROGATION 3
#define CMD_CLOCK_SYNC 4
#define CMD_SINGLE_CMD 5
#define CMD_DOUBLE_CMD 6

// Callback типы
typedef std::function<void()> OnConnectedCallback;
typedef std::function<void()> OnDisconnectedCallback;
typedef std::function<void(ASDUInfo& asdu)> OnASDUReceivedCallback;
typedef std::function<void(uint16_t sendSeq, uint16_t recvSeq)> OnIFormatReceivedCallback;

class IEC104Client {
public:
    IEC104Client();
    ~IEC104Client();
    
    // Основные методы
    bool connect(const char* serverIP, uint16_t port = 2404);
    void disconnect();
    bool isConnected();
    void start(); // Запуск задач FreeRTOS
    void stop();  // Остановка задач FreeRTOS
    
    // Команды
    bool sendInterrogationCommand(uint16_t commonAddress = 1, uint8_t qualifier = 20);
    bool sendClockSyncCommand(uint16_t commonAddress = 1);
    bool sendSingleCommand(uint16_t commonAddress, uint32_t ioa, bool state);
    bool sendDoubleCommand(uint16_t commonAddress, uint32_t ioa, uint8_t state);
    
    // Callback установки
    void onConnected(OnConnectedCallback callback);
    void onDisconnected(OnDisconnectedCallback callback);
    void onASDUReceived(OnASDUReceivedCallback callback);
    void onIFormatReceived(OnIFormatReceivedCallback callback);
    
    // Управление параметрами
    void setKeepAliveInterval(unsigned long interval) { keepAliveInterval = interval; }
    void setCommonAddress(uint16_t address) { localCommonAddress = address; }
    void setTaskPriority(UBaseType_t priority) { taskPriority = priority; }
    void setTaskStackSize(uint32_t stackSize) { taskStackSize = stackSize; }
    
private:
    WiFiClient client;
    
    // Параметры соединения
    String serverIP;
    uint16_t serverPort;
    uint16_t sendSequence;
    uint16_t receiveSequence;
    uint16_t localCommonAddress;
    
    // FreeRTOS элементы
    TaskHandle_t communicationTaskHandle;
    QueueHandle_t commandQueue;
    SemaphoreHandle_t connectionMutex;
    bool taskRunning;
    UBaseType_t taskPriority;
    uint32_t taskStackSize;
    
    // Таймеры
    unsigned long lastKeepAlive;
    unsigned long keepAliveInterval;
    
    // Callbacks
    OnConnectedCallback connectedCallback;
    OnDisconnectedCallback disconnectedCallback;
    OnASDUReceivedCallback asduReceivedCallback;
    OnIFormatReceivedCallback iFormatReceivedCallback;
    
    // Внутренние методы
    static void communicationTask(void* pvParameters);
    void communicationTaskImpl();
    void processCommands();
    void sendStartDT();
    void sendKeepAlive();
    void handleIncomingData();
    void processIECFrame(uint8_t* frame, int length);
    void processIFormat(uint8_t* frame, int length);
    void processSFormat(uint8_t* frame, int length);
    void processUFormat(uint8_t* frame, int length);
    void sendSFormat(uint16_t sequenceNumber);
    bool sendIFormat(uint8_t* data, uint16_t dataLength);
    void processASDU(uint8_t* asdu, uint16_t length);
    void cleanupConnection();
    
    // Вспомогательные методы
    uint16_t getTimestamp();
    void getCP56Time(uint8_t* timeBuffer);
    
    // Методы для работы с очередью команд
    bool queueCommand(uint8_t type, uint8_t* data = nullptr, uint16_t length = 0);
};

#endif