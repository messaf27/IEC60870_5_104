#ifndef IEC60870_5_104_H
#define IEC60870_5_104_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <functional>

#if defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#endif

/**
 * @brief Типы ASDU (Application Service Data Unit)
 */
enum ASDUType {
    M_SP_NA_1 = 1,      ///< Single point information
    M_DP_NA_1 = 3,      ///< Double point information  
    M_ST_NA_1 = 5,      ///< Step position information
    M_ME_NA_1 = 9,      ///< Measured value, normalized value
    M_ME_NB_1 = 11,     ///< Measured value, scaled value
    M_ME_NC_1 = 13,     ///< Measured value, short floating point
    M_IT_NA_1 = 15,     ///< Integrated totals
    M_PS_NA_1 = 20,     ///< Packed single point information
    M_ME_ND_1 = 21,     ///< Measured value, normalized value without quality
    C_SC_NA_1 = 45,     ///< Single command
    C_DC_NA_1 = 46,     ///< Double command
    C_RC_NA_1 = 47,     ///< Regulating step command
    C_SE_NA_1 = 48,     ///< Set point command, normalized value
    C_SE_NB_1 = 49,     ///< Set point command, scaled value
    C_SE_NC_1 = 50,     ///< Set point command, short floating point
    C_BO_NA_1 = 51,     ///< Bitstring of 32 bits
    C_IC_NA_1 = 100,    ///< Interrogation command
    C_CI_NA_1 = 101,    ///< Counter interrogation command
    C_RD_NA_1 = 102,    ///< Read command
    C_CS_NA_1 = 103,    ///< Clock synchronization command
    C_TS_NA_1 = 104,    ///< Test command
    C_RP_NA_1 = 105,    ///< Reset process command
    C_CD_NA_1 = 106,    ///< Delay acquisition command
    C_TS_TA_1 = 107     ///< Test command with time tag CP56Time2a
};

/**
 * @brief Причины передачи (Cause of Transmission)
 */
enum CauseOfTransmission {
    PERIODIC = 1,                   ///< Периодическая передача
    BACKGROUND_SCAN = 2,            ///< Фоновое сканирование
    SPONTANEOUS = 3,                ///< Спонтанная передача
    INITIALIZED = 4,                ///< Инициализация
    REQUEST = 5,                    ///< Запрос
    ACTIVATION = 6,                 ///< Активация
    ACTIVATION_CON = 7,             ///< Подтверждение активации
    DEACTIVATION = 8,               ///< Деактивация
    DEACTIVATION_CON = 9,           ///< Подтверждение деактивации
    ACTIVATION_TERMINATION = 10,    ///< Завершение активации
    INTERROGATED_BY_STATION = 20    ///< Опрос станции
};

/**
 * @brief Структура для информации об ASDU
 */
struct ASDUInfo {
    uint8_t typeID;                 ///< Тип ASDU
    uint8_t numberOfObjects;        ///< Количество объектов
    uint8_t causeOfTransmission;    ///< Причина передачи
    uint8_t originatorAddress;      ///< Адрес инициатора
    uint16_t commonAddress;         ///< Общий адрес
    uint8_t* data;                  ///< Данные ASDU
    uint16_t dataLength;            ///< Длина данных
};

/**
 * @brief Структура для команд
 */
struct IECCommand {
    uint8_t type;       ///< Тип команды
    uint8_t* data;      ///< Данные команды
    uint16_t length;    ///< Длина данных
};

// Типы команд
#define CMD_CONNECT 1       ///< Команда подключения
#define CMD_DISCONNECT 2    ///< Команда отключения
#define CMD_INTERROGATION 3 ///< Команда общего опроса
#define CMD_CLOCK_SYNC 4    ///< Команда синхронизации времени
#define CMD_SINGLE_CMD 5    ///< Одиночная команда
#define CMD_DOUBLE_CMD 6    ///< Двойная команда

// Callback типы
typedef std::function<void()> OnConnectedCallback;
typedef std::function<void()> OnDisconnectedCallback;
typedef std::function<void(ASDUInfo& asdu)> OnASDUReceivedCallback;

/**
 * @brief Клиент IEC 60870-5-104 для ESP32
 * 
 * Реализует клиентскую часть протокола IEC 60870-5-104 с поддержкой FreeRTOS
 * для многозадачной обработки сетевых коммуникаций.
 */
class IEC104Client {
public:
    /**
     * @brief Конструктор клиента IEC 104
     */
    IEC104Client();
    
    /**
     * @brief Деструктор клиента IEC 104
     */
    ~IEC104Client();
    
    /**
     * @brief Подключение к серверу IEC 104
     * @param serverIP IP адрес сервера
     * @param port Порт сервера (по умолчанию 2404)
     * @return true если команда успешно поставлена в очередь
     */
    bool connect(const char* serverIP, uint16_t port = 2404);
    
    /**
     * @brief Отключение от сервера IEC 104
     */
    void disconnect();
    
    /**
     * @brief Проверка состояния подключения
     * @return true если клиент подключен к серверу
     */
    bool isConnected();
    
    /**
     * @brief Запуск задачи FreeRTOS для обработки коммуникаций
     */
    void start();
    
    /**
     * @brief Остановка задачи FreeRTOS
     */
    void stop();
    
    /**
     * @brief Отправка команды общего опроса
     * @param commonAddress Общий адрес (по умолчанию 1)
     * @param qualifier Квалификатор опроса (по умолчанию 20)
     * @return true если команда успешно поставлена в очередь
     */
    bool sendInterrogationCommand(uint16_t commonAddress = 1, uint8_t qualifier = 20);
    
    /**
     * @brief Отправка команды синхронизации времени
     * @param commonAddress Общий адрес (по умолчанию 1)
     * @return true если команда успешно поставлена в очередь
     */
    bool sendClockSyncCommand(uint16_t commonAddress = 1);
    
    /**
     * @brief Отправка одиночной команды
     * @param commonAddress Общий адрес
     * @param ioa Адрес объекта информации
     * @param state Состояние (true/false)
     * @return true если команда успешно поставлена в очередь
     */
    bool sendSingleCommand(uint16_t commonAddress, uint32_t ioa, bool state);
    
    /**
     * @brief Отправка двойной команды
     * @param commonAddress Общий адрес
     * @param ioa Адрес объекта информации
     * @param state Состояние (0-3)
     * @return true если команда успешно поставлена в очередь
     */
    bool sendDoubleCommand(uint16_t commonAddress, uint32_t ioa, uint8_t state);
    
    /**
     * @brief Установка callback'а при подключении
     * @param callback Функция обратного вызова
     */
    void onConnected(OnConnectedCallback callback) { connectedCallback = callback; }
    
    /**
     * @brief Установка callback'а при отключении
     * @param callback Функция обратного вызова
     */
    void onDisconnected(OnDisconnectedCallback callback) { disconnectedCallback = callback; }
    
    /**
     * @brief Установка callback'а при получении ASDU
     * @param callback Функция обратного вызова
     */
    void onASDUReceived(OnASDUReceivedCallback callback) { asduReceivedCallback = callback; }
    
    /**
     * @brief Установка интервала keep-alive
     * @param interval Интервал в миллисекундах
     */
    void setKeepAliveInterval(unsigned long interval) { keepAliveInterval = interval; }
    
    /**
     * @brief Установка общего адреса
     * @param address Общий адрес
     */
    void setCommonAddress(uint16_t address) { localCommonAddress = address; }
    
    /**
     * @brief Установка приоритета задачи FreeRTOS
     * @param priority Приоритет задачи
     */
    void setTaskPriority(UBaseType_t priority) { taskPriority = priority; }
    
    /**
     * @brief Установка размера стека задачи FreeRTOS
     * @param stackSize Размер стека в словах
     */
    void setTaskStackSize(uint32_t stackSize) { taskStackSize = stackSize; }
    
private:
    WiFiClient client;              ///< TCP клиент для соединения
    
    // Параметры соединения
    String serverIP;                ///< IP адрес сервера
    uint16_t serverPort;            ///< Порт сервера
    uint16_t sendSequence;          ///< Посылающая последовательность
    uint16_t receiveSequence;       ///< Принимающая последовательность
    uint16_t localCommonAddress;    ///< Локальный общий адрес
    
    // FreeRTOS элементы (указатели)
    #if defined(ESP32)
    TaskHandle_t communicationTaskHandle;   ///< Дескриптор задачи коммуникаций
    QueueHandle_t commandQueue;             ///< Очередь команд
    #endif
    
    bool taskRunning;               ///< Флаг запущенной задачи
    UBaseType_t taskPriority;       ///< Приоритет задачи FreeRTOS
    uint32_t taskStackSize;         ///< Размер стека задачи FreeRTOS
    
    // Таймеры
    unsigned long lastKeepAlive;    ///< Время последнего keep-alive
    unsigned long keepAliveInterval;///< Интервал keep-alive
    
    // Callbacks
    OnConnectedCallback connectedCallback;      ///< Callback при подключении
    OnDisconnectedCallback disconnectedCallback;///< Callback при отключении
    OnASDUReceivedCallback asduReceivedCallback;///< Callback при получении ASDU
    
    // Внутренние методы
    #if defined(ESP32)
    /**
     * @brief Статическая функция задачи FreeRTOS
     */
    static void communicationTask(void* pvParameters);
    
    /**
     * @brief Реализация задачи коммуникаций
     */
    void communicationTaskImpl();
    
    /**
     * @brief Инициализация FreeRTOS объектов
     */
    void initializeFreeRTOS();
    
    /**
     * @brief Обработка команд из очереди
     */
    void processCommands();
    #endif
    
    /**
     * @brief Отправка STARTDT команды
     */
    void sendStartDT();
    
    /**
     * @brief Отправка keep-alive (TESTFR)
     */
    void sendKeepAlive();
    
    /**
     * @brief Обработка входящих данных
     */
    void handleIncomingData();
    
    /**
     * @brief Обработка IEC фрейма
     * @param frame Указатель на фрейм
     * @param length Длина фрейма
     */
    void processIECFrame(uint8_t* frame, int length);
    
    /**
     * @brief Обработка I-формата фрейма
     * @param frame Указатель на фрейм
     * @param length Длина фрейма
     */
    void processIFormat(uint8_t* frame, int length);
    
    /**
     * @brief Обработка S-формата фрейма
     * @param frame Указатель на фрейм
     * @param length Длина фрейма
     */
    void processSFormat(uint8_t* frame, int length);
    
    /**
     * @brief Обработка U-формата фрейма
     * @param frame Указатель на фрейм
     * @param length Длина фрейма
     */
    void processUFormat(uint8_t* frame, int length);
    
    /**
     * @brief Отправка S-формата подтверждения
     * @param sequenceNumber Номер последовательности
     */
    void sendSFormat(uint16_t sequenceNumber);
    
    /**
     * @brief Отправка I-формата данных
     * @param data Указатель на данные
     * @param dataLength Длина данных
     * @return true если отправка успешна
     */
    bool sendIFormat(uint8_t* data, uint16_t dataLength);
    
    /**
     * @brief Обработка ASDU данных
     * @param asdu Указатель на ASDU
     * @param length Длина ASDU
     */
    void processASDU(uint8_t* asdu, uint16_t length);
    
    /**
     * @brief Очистка соединения
     */
    void cleanupConnection();
    
    /**
     * @brief Постановка команды в очередь
     * @param type Тип команды
     * @param data Данные команды
     * @param length Длина данных
     * @return true если команда успешно поставлена в очередь
     */
    bool queueCommand(uint8_t type, uint8_t* data = nullptr, uint16_t length = 0);
    
    /**
     * @brief Получение времени в формате CP56Time2a
     * @param timeBuffer Буфер для времени (7 байт)
     */
    void getCP56Time(uint8_t* timeBuffer);
};

#endif