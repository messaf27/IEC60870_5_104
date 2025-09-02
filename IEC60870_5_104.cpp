#include "IEC60870_5_104.h"
#include <Arduino.h>
#include <time.h>

IEC104Client::IEC104Client() {
    sendSequence = 0;
    receiveSequence = 0;
    localCommonAddress = 1;
    lastKeepAlive = 0;
    keepAliveInterval = 30000; // 30 секунд
    taskRunning = false;
    taskPriority = 1;
    taskStackSize = 4096;
    communicationTaskHandle = nullptr;
    commandQueue = xQueueCreate(10, sizeof(IECCommand));
    connectionMutex = xSemaphoreCreateMutex();
    
    // Инициализация Wi-Fi
    WiFi.mode(WIFI_STA);
}

IEC104Client::~IEC104Client() {
    stop();
    if (commandQueue) {
        vQueueDelete(commandQueue);
    }
    if (connectionMutex) {
        vSemaphoreDelete(connectionMutex);
    }
}

bool IEC104Client::connect(const char* ip, uint16_t port) {
    serverIP = String(ip);
    serverPort = port;
    return queueCommand(CMD_CONNECT);
}

void IEC104Client::disconnect() {
    queueCommand(CMD_DISCONNECT);
}

bool IEC104Client::isConnected() {
    if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool connected = client.connected();
        xSemaphoreGive(connectionMutex);
        return connected;
    }
    return false;
}

void IEC104Client::start() {
    if (!taskRunning) {
        taskRunning = true;
        xTaskCreate(
            communicationTask,
            "IEC104_Task",
            taskStackSize,
            this,
            taskPriority,
            &communicationTaskHandle
        );
    }
}

void IEC104Client::stop() {
    if (taskRunning) {
        taskRunning = false;
        if (communicationTaskHandle) {
            vTaskDelete(communicationTaskHandle);
            communicationTaskHandle = nullptr;
        }
    }
    cleanupConnection();
}

void IEC104Client::communicationTask(void* pvParameters) {
    IEC104Client* client = static_cast<IEC104Client*>(pvParameters);
    client->communicationTaskImpl();
    vTaskDelete(nullptr);
}

void IEC104Client::communicationTaskImpl() {
    while (taskRunning) {
        // Обработка команд из очереди
        processCommands();
        
        // Обработка сетевых данных
        if (isConnected()) {
            handleIncomingData();
            sendKeepAlive();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void IEC104Client::processCommands() {
    IECCommand cmd;
    while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
        switch (cmd.type) {
            case CMD_CONNECT: {
                if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    if (!client.connected()) {
                        if (client.connect(serverIP.c_str(), serverPort)) {
                            sendStartDT();
                        }
                    }
                    xSemaphoreGive(connectionMutex);
                }
                break;
            }
            
            case CMD_DISCONNECT: {
                if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    if (client.connected()) {
                        client.stop();
                        if (disconnectedCallback) {
                            disconnectedCallback();
                        }
                    }
                    xSemaphoreGive(connectionMutex);
                }
                break;
            }
            
            case CMD_INTERROGATION: {
                if (cmd.data && cmd.length >= 3) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    uint8_t qualifier = cmd.data[2];
                    sendInterrogationCommand(addr, qualifier);
                }
                break;
            }
            
            case CMD_CLOCK_SYNC: {
                if (cmd.data && cmd.length >= 2) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    sendClockSyncCommand(addr);
                }
                break;
            }
            
            case CMD_SINGLE_CMD: {
                if (cmd.data && cmd.length >= 6) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    uint32_t ioa = cmd.data[2] | (cmd.data[3] << 8) | (cmd.data[4] << 16);
                    bool state = cmd.data[5];
                    sendSingleCommand(addr, ioa, state);
                }
                break;
            }
            
            case CMD_DOUBLE_CMD: {
                if (cmd.data && cmd.length >= 6) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    uint32_t ioa = cmd.data[2] | (cmd.data[3] << 8) | (cmd.data[4] << 16);
                    uint8_t state = cmd.data[5];
                    sendDoubleCommand(addr, ioa, state);
                }
                break;
            }
        }
        
        // Освобождаем память команды
        if (cmd.data) {
            delete[] cmd.data;
        }
    }
}

bool IEC104Client::queueCommand(uint8_t type, uint8_t* data, uint16_t length) {
    IECCommand cmd;
    cmd.type = type;
    cmd.data = data;
    cmd.length = length;
    
    BaseType_t result = xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
    return result == pdTRUE;
}

bool IEC104Client::sendInterrogationCommand(uint16_t commonAddress, uint8_t qualifier) {
    uint8_t* data = new uint8_t[3];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    data[2] = qualifier;
    
    IECCommand cmd;
    cmd.type = CMD_INTERROGATION;
    cmd.data = data;
    cmd.length = 3;
    
    BaseType_t result = xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
        delete[] data;
        return false;
    }
    return true;
}

bool IEC104Client::sendClockSyncCommand(uint16_t commonAddress) {
    uint8_t* data = new uint8_t[2];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    
    IECCommand cmd;
    cmd.type = CMD_CLOCK_SYNC;
    cmd.data = data;
    cmd.length = 2;
    
    BaseType_t result = xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
        delete[] data;
        return false;
    }
    return true;
}

bool IEC104Client::sendSingleCommand(uint16_t commonAddress, uint32_t ioa, bool state) {
    uint8_t* data = new uint8_t[6];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    data[2] = ioa & 0xFF;
    data[3] = (ioa >> 8) & 0xFF;
    data[4] = (ioa >> 16) & 0xFF;
    data[5] = state ? 1 : 0;
    
    IECCommand cmd;
    cmd.type = CMD_SINGLE_CMD;
    cmd.data = data;
    cmd.length = 6;
    
    BaseType_t result = xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
        delete[] data;
        return false;
    }
    return true;
}

bool IEC104Client::sendDoubleCommand(uint16_t commonAddress, uint32_t ioa, uint8_t state) {
    uint8_t* data = new uint8_t[6];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    data[2] = ioa & 0xFF;
    data[3] = (ioa >> 8) & 0xFF;
    data[4] = (ioa >> 16) & 0xFF;
    data[5] = state & 0x03;
    
    IECCommand cmd;
    cmd.type = CMD_DOUBLE_CMD;
    cmd.data = data;
    cmd.length = 6;
    
    BaseType_t result = xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
        delete[] data;
        return false;
    }
    return true;
}

void IEC104Client::sendStartDT() {
    uint8_t startDT[] = {0x68, 0x04, 0x07, 0x00, 0x00, 0x00};
    if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (client.connected()) {
            client.write(startDT, sizeof(startDT));
        }
        xSemaphoreGive(connectionMutex);
    }
}

void IEC104Client::sendKeepAlive() {
    if (millis() - lastKeepAlive > keepAliveInterval) {
        uint8_t testfr[] = {0x68, 0x04, 0x43, 0x00, 0x00, 0x00};
        if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (client.connected()) {
                client.write(testfr, sizeof(testfr));
            }
            xSemaphoreGive(connectionMutex);
        }
        lastKeepAlive = millis();
    }
}

void IEC104Client::handleIncomingData() {
    static uint8_t buffer[256];
    static int bufferPos = 0;
    
    if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    if (!client.connected()) {
        xSemaphoreGive(connectionMutex);
        if (disconnectedCallback) {
            disconnectedCallback();
        }
        return;
    }
    
    // Читаем доступные данные в буфер
    while (client.available() && bufferPos < sizeof(buffer)) {
        buffer[bufferPos++] = client.read();
    }
    
    // Обрабатываем данные в буфере
    int processed = 0;
    while (bufferPos - processed >= 2) {
        if (buffer[processed] != 0x68) {
            processed++;
            continue;
        }
        
        if (bufferPos - processed >= 6) {
            uint8_t length = buffer[processed + 1];
            int totalLength = 2 + length;
            
            if (bufferPos - processed >= totalLength) {
                // У нас есть полный фрейм
                uint8_t* frame = new uint8_t[totalLength];
                memcpy(frame, buffer + processed, totalLength);
                processIECFrame(frame, totalLength);
                delete[] frame;
                
                processed += totalLength;
            } else {
                break; // Неполный фрейм, ждем больше данных
            }
        } else {
            break; // Недостаточно данных для заголовка
        }
    }
    
    // Сдвигаем необработанные данные в начало буфера
    if (processed > 0 && processed < bufferPos) {
        memmove(buffer, buffer + processed, bufferPos - processed);
        bufferPos -= processed;
    } else {
        bufferPos = 0;
    }
    
    xSemaphoreGive(connectionMutex);
}

void IEC104Client::processIECFrame(uint8_t* frame, int length) {
    if (length < 6) return;
    
    uint8_t start = frame[0];
    if (start != 0x68) return;
    
    uint8_t format = (frame[2] & 0x03);
    
    switch (format) {
        case 0x00: // I-format
            processIFormat(frame, length);
            break;
        case 0x01: // S-format
            processSFormat(frame, length);
            break;
        case 0x03: // U-format
            processUFormat(frame, length);
            break;
    }
}

void IEC104Client::processIFormat(uint8_t* frame, int length) {
    if (length < 6) return;
    
    uint16_t recvSeq = (frame[2] >> 1) | ((frame[3] & 0x7F) << 7);
    uint16_t sendSeq = (frame[4] >> 1) | ((frame[5] & 0x7F) << 7);
    
    receiveSequence = recvSeq + 1;
    sendSFormat(receiveSequence);
    
    if (iFormatReceivedCallback) {
        iFormatReceivedCallback(sendSeq, recvSeq);
    }
    
    if (length > 6) {
        processASDU(frame + 6, length - 6);
    }
}

void IEC104Client::processSFormat(uint8_t* frame, int length) {
    if (length < 6) return;
    uint16_t recvSeq = (frame[2] >> 1) | ((frame[3] & 0x7F) << 7);
}

void IEC104Client::processUFormat(uint8_t* frame, int length) {
    if (length < 6) return;
    
    uint8_t control = frame[2] & 0xFC;
    
    switch (control) {
        case 0x07: // STARTDT_CON
            if (connectedCallback) {
                connectedCallback();
            }
            sendInterrogationCommand();
            break;
        case 0x43: // TESTFR_CON
            break;
    }
}

void IEC104Client::sendSFormat(uint16_t sequenceNumber) {
    uint8_t sFrame[6];
    sFrame[0] = 0x68;
    sFrame[1] = 0x04;
    sFrame[2] = (sequenceNumber << 1) & 0xFF;
    sFrame[3] = ((sequenceNumber << 1) >> 8) & 0x7F;
    sFrame[4] = 0x01;
    sFrame[5] = 0x00;
    
    if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (client.connected()) {
            client.write(sFrame, sizeof(sFrame));
        }
        xSemaphoreGive(connectionMutex);
    }
}

bool IEC104Client::sendIFormat(uint8_t* data, uint16_t dataLength) {
    if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    
    if (!client.connected()) {
        xSemaphoreGive(connectionMutex);
        return false;
    }
    
    uint16_t frameLength = 4 + dataLength;
    uint8_t* frame = new uint8_t[frameLength + 2];
    
    frame[0] = 0x68;
    frame[1] = frameLength & 0xFF;
    frame[2] = (sendSequence << 1) & 0xFF;
    frame[3] = ((sendSequence << 1) >> 8) & 0x7F;
    frame[4] = (receiveSequence << 1) & 0xFF;
    frame[5] = ((receiveSequence << 1) >> 8) & 0x7F;
    
    memcpy(frame + 6, data, dataLength);
    
    client.write(frame, frameLength + 2);
    sendSequence++;
    
    xSemaphoreGive(connectionMutex);
    delete[] frame;
    return true;
}

void IEC104Client::processASDU(uint8_t* asdu, uint16_t length) {
    if (length < 6) return;
    
    ASDUInfo info;
    info.typeID = asdu[0];
    info.numberOfObjects = asdu[1];
    info.causeOfTransmission = asdu[2] & 0x3F; // Младшие 6 бит
    info.originatorAddress = asdu[3];
    info.commonAddress = asdu[4] | (asdu[5] << 8);
    info.data = asdu + 6;
    info.dataLength = length - 6;
    
    if (asduReceivedCallback) {
        asduReceivedCallback(info);
    }
}

void IEC104Client::onConnected(OnConnectedCallback callback) {
    connectedCallback = callback;
}

void IEC104Client::onDisconnected(OnDisconnectedCallback callback) {
    disconnectedCallback = callback;
}

void IEC104Client::onASDUReceived(OnASDUReceivedCallback callback) {
    asduReceivedCallback = callback;
}

void IEC104Client::onIFormatReceived(OnIFormatReceivedCallback callback) {
    iFormatReceivedCallback = callback;
}

void IEC104Client::cleanupConnection() {
    if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (client.connected()) {
            client.stop();
        }
        xSemaphoreGive(connectionMutex);
    }
}

uint16_t IEC104Client::getTimestamp() {
    return millis() / 1000;
}

void IEC104Client::getCP56Time(uint8_t* timeBuffer) {
    time_t now = time(nullptr);
    struct tm* tm_info = localtime(&now);
    
    // Milliseconds (0 в данном случае)
    timeBuffer[0] = 0;
    timeBuffer[1] = 0;
    
    // Минуты
    timeBuffer[2] = tm_info->tm_min;
    // Часы
    timeBuffer[3] = tm_info->tm_hour;
    // День недели + день месяца
    timeBuffer[4] = tm_info->tm_wday << 5 | tm_info->tm_mday;
    // Месяц
    timeBuffer[5] = tm_info->tm_mon + 1;
    // Год (от 0 до 99)
    timeBuffer[6] = tm_info->tm_year - 100;
}