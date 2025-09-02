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
    
    #if defined(ESP32)
    communicationTaskHandle = nullptr;
    commandQueue = nullptr;
    #endif
}

IEC104Client::~IEC104Client() {
    stop();
    #if defined(ESP32)
    if (commandQueue) {
        vQueueDelete(commandQueue);
        commandQueue = nullptr;
    }
    #endif
}

#if defined(ESP32)
void IEC104Client::initializeFreeRTOS() {
    if (commandQueue == nullptr) {
        commandQueue = xQueueCreate(10, sizeof(IECCommand));
    }
}
#endif

bool IEC104Client::connect(const char* ip, uint16_t port) {
    serverIP = String(ip);
    serverPort = port;
    return queueCommand(CMD_CONNECT);
}

void IEC104Client::disconnect() {
    queueCommand(CMD_DISCONNECT);
}

bool IEC104Client::isConnected() {
    return client.connected();
}

void IEC104Client::start() {
    #if defined(ESP32)
    if (!taskRunning) {
        initializeFreeRTOS();
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
    #endif
}

void IEC104Client::stop() {
    #if defined(ESP32)
    if (taskRunning) {
        taskRunning = false;
        if (communicationTaskHandle) {
            vTaskDelete(communicationTaskHandle);
            communicationTaskHandle = nullptr;
        }
    }
    #endif
    cleanupConnection();
}

#if defined(ESP32)
void IEC104Client::communicationTask(void* pvParameters) {
    IEC104Client* client = static_cast<IEC104Client*>(pvParameters);
    client->communicationTaskImpl();
    vTaskDelete(nullptr);
}

void IEC104Client::communicationTaskImpl() {
    while (taskRunning) {
        processCommands();
        
        if (isConnected()) {
            handleIncomingData();
            sendKeepAlive();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void IEC104Client::processCommands() {
    if (commandQueue == nullptr) return;
    
    IECCommand cmd;
    while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
        switch (cmd.type) {
            case CMD_CONNECT: {
                if (!client.connected()) {
                    if (client.connect(serverIP.c_str(), serverPort)) {
                        sendStartDT();
                    }
                }
                break;
            }
            
            case CMD_DISCONNECT: {
                if (client.connected()) {
                    client.stop();
                    if (disconnectedCallback) {
                        disconnectedCallback();
                    }
                }
                break;
            }
            
            case CMD_INTERROGATION: {
                if (cmd.data && cmd.length >= 3) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    uint8_t qualifier = cmd.data[2];
                    sendInterrogationCommand(addr, qualifier);
                }
                if (cmd.data) delete[] cmd.data;
                break;
            }
            
            case CMD_CLOCK_SYNC: {
                if (cmd.data && cmd.length >= 2) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    sendClockSyncCommand(addr);
                }
                if (cmd.data) delete[] cmd.data;
                break;
            }
            
            case CMD_SINGLE_CMD: {
                if (cmd.data && cmd.length >= 6) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    uint32_t ioa = cmd.data[2] | (cmd.data[3] << 8) | (cmd.data[4] << 16);
                    bool state = cmd.data[5];
                    sendSingleCommand(addr, ioa, state);
                }
                if (cmd.data) delete[] cmd.data;
                break;
            }
            
            case CMD_DOUBLE_CMD: {
                if (cmd.data && cmd.length >= 6) {
                    uint16_t addr = cmd.data[0] | (cmd.data[1] << 8);
                    uint32_t ioa = cmd.data[2] | (cmd.data[3] << 8) | (cmd.data[4] << 16);
                    uint8_t state = cmd.data[5];
                    sendDoubleCommand(addr, ioa, state);
                }
                if (cmd.data) delete[] cmd.data;
                break;
            }
        }
    }
}
#endif

bool IEC104Client::queueCommand(uint8_t type, uint8_t* data, uint16_t length) {
    #if defined(ESP32)
    if (commandQueue == nullptr) {
        // Если очередь не создана, выполняем команду немедленно
        switch (type) {
            case CMD_CONNECT:
                return client.connect(serverIP.c_str(), serverPort);
            case CMD_DISCONNECT:
                client.stop();
                return true;
            default:
                if (data) delete[] data;
                return false;
        }
    }
    
    IECCommand cmd;
    cmd.type = type;
    cmd.data = data;
    cmd.length = length;
    
    BaseType_t result = xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
    return result == pdTRUE;
    #else
    // Без FreeRTOS выполняем немедленно
    switch (type) {
        case CMD_CONNECT:
            return client.connect(serverIP.c_str(), serverPort);
        case CMD_DISCONNECT:
            client.stop();
            return true;
        default:
            if (data) delete[] data;
            return false;
    }
    #endif
}

bool IEC104Client::sendInterrogationCommand(uint16_t commonAddress, uint8_t qualifier) {
    #if defined(ESP32)
    uint8_t* data = new uint8_t[3];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    data[2] = qualifier;
    return queueCommand(CMD_INTERROGATION, data, 3);
    #else
    // Простая реализация без FreeRTOS
    uint8_t data[10];
    int len = 0;
    data[len++] = C_IC_NA_1;
    data[len++] = 0x01;
    data[len++] = ACTIVATION;
    data[len++] = 0x00;
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = 0x00;
    data[len++] = 0x00;
    data[len++] = 0x00;
    data[len++] = qualifier;
    return sendIFormat(data, len);
    #endif
}

bool IEC104Client::sendClockSyncCommand(uint16_t commonAddress) {
    #if defined(ESP32)
    uint8_t* data = new uint8_t[2];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    return queueCommand(CMD_CLOCK_SYNC, data, 2);
    #else
    uint8_t data[13];
    int len = 0;
    data[len++] = C_CS_NA_1;
    data[len++] = 0x01;
    data[len++] = ACTIVATION;
    data[len++] = 0x00;
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = 0x00;
    data[len++] = 0x00;
    data[len++] = 0x00;
    uint8_t timeBuffer[7];
    getCP56Time(timeBuffer);
    memcpy(data + len, timeBuffer, 7);
    len += 7;
    return sendIFormat(data, len);
    #endif
}

bool IEC104Client::sendSingleCommand(uint16_t commonAddress, uint32_t ioa, bool state) {
    #if defined(ESP32)
    uint8_t* data = new uint8_t[6];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    data[2] = ioa & 0xFF;
    data[3] = (ioa >> 8) & 0xFF;
    data[4] = (ioa >> 16) & 0xFF;
    data[5] = state ? 1 : 0;
    return queueCommand(CMD_SINGLE_CMD, data, 6);
    #else
    uint8_t data[10];
    int len = 0;
    data[len++] = C_SC_NA_1;
    data[len++] = 0x01;
    data[len++] = ACTIVATION;
    data[len++] = 0x00;
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = ioa & 0xFF;
    data[len++] = (ioa >> 8) & 0xFF;
    data[len++] = (ioa >> 16) & 0xFF;
    data[len++] = state ? 0x81 : 0x80;
    return sendIFormat(data, len);
    #endif
}

bool IEC104Client::sendDoubleCommand(uint16_t commonAddress, uint32_t ioa, uint8_t state) {
    #if defined(ESP32)
    uint8_t* data = new uint8_t[6];
    data[0] = commonAddress & 0xFF;
    data[1] = (commonAddress >> 8) & 0xFF;
    data[2] = ioa & 0xFF;
    data[3] = (ioa >> 8) & 0xFF;
    data[4] = (ioa >> 16) & 0xFF;
    data[5] = state & 0x03;
    return queueCommand(CMD_DOUBLE_CMD, data, 6);
    #else
    uint8_t data[10];
    int len = 0;
    data[len++] = C_DC_NA_1;
    data[len++] = 0x01;
    data[len++] = ACTIVATION;
    data[len++] = 0x00;
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = ioa & 0xFF;
    data[len++] = (ioa >> 8) & 0xFF;
    data[len++] = (ioa >> 16) & 0xFF;
    data[len++] = (state & 0x03) | 0x80;
    return sendIFormat(data, len);
    #endif
}

void IEC104Client::sendStartDT() {
    uint8_t startDT[] = {0x68, 0x04, 0x07, 0x00, 0x00, 0x00};
    if (client.connected()) {
        client.write(startDT, sizeof(startDT));
    }
}

void IEC104Client::sendKeepAlive() {
    if (millis() - lastKeepAlive > keepAliveInterval) {
        uint8_t testfr[] = {0x68, 0x04, 0x43, 0x00, 0x00, 0x00};
        if (client.connected()) {
            client.write(testfr, sizeof(testfr));
        }
        lastKeepAlive = millis();
    }
}

void IEC104Client::handleIncomingData() {
    static uint8_t buffer[256];
    static int bufferPos = 0;
    
    if (!client.connected()) {
        if (disconnectedCallback) {
            disconnectedCallback();
        }
        bufferPos = 0;
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
                break;
            }
        } else {
            break;
        }
    }
    
    // Сдвигаем необработанные данные в начало буфера
    if (processed > 0 && processed < bufferPos) {
        memmove(buffer, buffer + processed, bufferPos - processed);
        bufferPos -= processed;
    } else {
        bufferPos = 0;
    }
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
    
    if (length > 6) {
        processASDU(frame + 6, length - 6);
    }
}

void IEC104Client::processSFormat(uint8_t* frame, int length) {
    if (length < 6) return;
    // S-format обработка
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
    
    if (client.connected()) {
        client.write(sFrame, sizeof(sFrame));
    }
}

bool IEC104Client::sendIFormat(uint8_t* data, uint16_t dataLength) {
    if (!client.connected()) return false;
    
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
    
    delete[] frame;
    return true;
}

void IEC104Client::processASDU(uint8_t* asdu, uint16_t length) {
    if (length < 6) return;
    
    ASDUInfo info;
    info.typeID = asdu[0];
    info.numberOfObjects = asdu[1];
    info.causeOfTransmission = asdu[2] & 0x3F;
    info.originatorAddress = asdu[3];
    info.commonAddress = asdu[4] | (asdu[5] << 8);
    info.data = asdu + 6;
    info.dataLength = length - 6;
    
    if (asduReceivedCallback) {
        asduReceivedCallback(info);
    }
}

void IEC104Client::cleanupConnection() {
    if (client.connected()) {
        client.stop();
    }
}

void IEC104Client::getCP56Time(uint8_t* timeBuffer) {
    time_t now = time(nullptr);
    struct tm* tm_info = localtime(&now);
    
    timeBuffer[0] = 0;
    timeBuffer[1] = 0;
    timeBuffer[2] = tm_info->tm_min;
    timeBuffer[3] = tm_info->tm_hour;
    timeBuffer[4] = tm_info->tm_wday << 5 | tm_info->tm_mday;
    timeBuffer[5] = tm_info->tm_mon + 1;
    timeBuffer[6] = tm_info->tm_year - 100;
}