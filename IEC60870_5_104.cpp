#include "IEC60870_5_104.h"
#include <Arduino.h>
#include <time.h>

IEC104Client::IEC104Client() {
    sendSequence = 0;
    receiveSequence = 0;
    localCommonAddress = 1;
    lastKeepAlive = 0;
    keepAliveInterval = 30000; // 30 секунд
}

bool IEC104Client::connect(const char* ip, uint16_t port) {
    serverIP = String(ip);
    serverPort = port;
    
    if (client.connect(ip, port)) {
        sendStartDT();
        if (connectedCallback) {
            connectedCallback();
        }
        return true;
    }
    return false;
}

void IEC104Client::disconnect() {
    client.stop();
    if (disconnectedCallback) {
        disconnectedCallback();
    }
}

bool IEC104Client::isConnected() {
    return client.connected();
}

void IEC104Client::run() {
    if (client.connected()) {
        handleIncomingData();
        sendKeepAlive();
    }
}

void IEC104Client::sendStartDT() {
    uint8_t startDT[] = {0x68, 0x04, 0x07, 0x00, 0x00, 0x00};
    client.write(startDT, sizeof(startDT));
}

void IEC104Client::sendKeepAlive() {
    if (millis() - lastKeepAlive > keepAliveInterval) {
        uint8_t testfr[] = {0x68, 0x04, 0x43, 0x00, 0x00, 0x00};
        client.write(testfr, sizeof(testfr));
        lastKeepAlive = millis();
    }
}

void IEC104Client::handleIncomingData() {
    while (client.available() >= 2) {
        if (client.peek() != 0x68) {
            client.read();
            continue;
        }
        
        if (client.available() >= 6) {
            uint8_t length = client.peek(1);
            int totalLength = 2 + length;
            
            if (client.available() >= totalLength) {
                uint8_t* buffer = new uint8_t[totalLength];
                client.read(buffer, totalLength);
                processIECFrame(buffer, totalLength);
                delete[] buffer;
            }
        }
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
    
    client.write(sFrame, sizeof(sFrame));
}

bool IEC104Client::sendInterrogationCommand(uint16_t commonAddress, uint8_t qualifier) {
    uint8_t data[10];
    int len = 0;
    
    data[len++] = C_IC_NA_1;        // Type ID
    data[len++] = 0x01;             // Number of objects
    data[len++] = ACTIVATION;       // Cause of transmission
    data[len++] = 0x00;             // Originator address
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = 0x00;             // IOA
    data[len++] = 0x00;
    data[len++] = 0x00;
    data[len++] = qualifier;        // QOI
    
    return sendIFormat(data, len);
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

bool IEC104Client::sendClockSyncCommand(uint16_t commonAddress) {
    uint8_t data[13];
    int len = 0;
    
    data[len++] = C_CS_NA_1;        // Type ID
    data[len++] = 0x01;             // Number of objects
    data[len++] = ACTIVATION;       // Cause of transmission
    data[len++] = 0x00;             // Originator address
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = 0x00;             // IOA
    data[len++] = 0x00;
    data[len++] = 0x00;
    
    // CP56Time2a (8 байт)
    uint8_t timeBuffer[7];
    getCP56Time(timeBuffer);
    memcpy(data + len, timeBuffer, 7);
    len += 7;
    
    return sendIFormat(data, len);
}

bool IEC104Client::sendSingleCommand(uint16_t commonAddress, uint32_t ioa, bool state) {
    uint8_t data[10];
    int len = 0;
    
    data[len++] = C_SC_NA_1;        // Type ID
    data[len++] = 0x01;             // Number of objects
    data[len++] = ACTIVATION;       // Cause of transmission
    data[len++] = 0x00;             // Originator address
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = ioa & 0xFF;
    data[len++] = (ioa >> 8) & 0xFF;
    data[len++] = (ioa >> 16) & 0xFF;
    data[len++] = state ? 0x81 : 0x80; // SCS + QU=0 + SE=1
    
    return sendIFormat(data, len);
}

bool IEC104Client::sendDoubleCommand(uint16_t commonAddress, uint32_t ioa, uint8_t state) {
    uint8_t data[10];
    int len = 0;
    
    data[len++] = C_DC_NA_1;        // Type ID
    data[len++] = 0x01;             // Number of objects
    data[len++] = ACTIVATION;       // Cause of transmission
    data[len++] = 0x00;             // Originator address
    data[len++] = commonAddress & 0xFF;
    data[len++] = (commonAddress >> 8) & 0xFF;
    data[len++] = ioa & 0xFF;
    data[len++] = (ioa >> 8) & 0xFF;
    data[len++] = (ioa >> 16) & 0xFF;
    data[len++] = (state & 0x03) | 0x80; // DCS + QU=0 + SE=1
    
    return sendIFormat(data, len);
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