
#include "Arduino.h"
#include "RMDx8Arduino.h"
#include <mcp_can.h>

// constructor
RMDx8Arduino::RMDx8Arduino(MCP_CAN &CAN) 
    :_CAN(CAN){}


void RMDx8Arduino::canSetup() {
    while (CAN_OK != _CAN.begin(CAN_1000KBPS)) {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}


void RMDx8Arduino::readBuf(const uint16_t motor_addr) {
    if (CAN_MSGAVAIL == _CAN.checkReceive()) {
        _CAN.readMsgBuf(&len, reply_buf);
    }
}


void RMDx8Arduino::readPID(const uint16_t motor_addr) {
    cmd_buf[0] = 0x30;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    
    // Send message
    writeCmd(motor_addr, cmd_buf);
    delay(100);
    readBuf(motor_addr);
}


void RMDx8Arduino::writePID(const uint16_t motor_addr, int posKp, int posKi, int velKp, int velKi, int iqKp, int iqKi) {
    cmd_buf[0] = 0x31;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = posKp;
    cmd_buf[3] = posKi;
    cmd_buf[4] = velKp;
    cmd_buf[5] = velKi;
    cmd_buf[6] = iqKp;
    cmd_buf[7] = iqKi;

    // Send message
    writeCmd(motor_addr, cmd_buf);
    readBuf(motor_addr);
}


void RMDx8Arduino::clearState(const uint16_t motor_addr) {
    cmd_buf[0] = 0x80;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(motor_addr, cmd_buf);
    readBuf(motor_addr);
}


void RMDx8Arduino::writeCurrent(const uint16_t motor_addr, int16_t current) {
    // current control is int16_t type. (2byteの符号付き整数)
    cmd_buf[0] = 0xA1;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = current & 0xFF;
    cmd_buf[5] = (current >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(motor_addr, cmd_buf);
    readBuf(motor_addr);
}


void RMDx8Arduino::writeVelocity(const uint16_t motor_addr, int32_t velocity) {
    // velocity control is int32_t type. (4byteの符号付き整数)
    cmd_buf[0] = 0xA2;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = velocity & 0xFF;
    cmd_buf[5] = (velocity >> 8) & 0xFF;
    cmd_buf[6] = (velocity >> 16) & 0xFF;
    cmd_buf[7] = (velocity >> 24) & 0xFF;

    // Send message
    writeCmd(motor_addr, cmd_buf);
    readBuf(motor_addr);
}


void RMDx8Arduino::writePosition(const uint16_t motor_addr, int32_t position) {
    // position control is int32_t type. (4byteの符号付き整数)
    cmd_buf[0] = 0xA3;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = (position >> 16) & 0xFF;
    cmd_buf[7] = (position >> 24) & 0xFF;

    // Send message
    writeCmd(motor_addr, cmd_buf);
    readBuf(motor_addr);
}


// General function
void RMDx8Arduino::serialWriteTerminator() {
    Serial.write(13);
    Serial.write(10);
}


// Private
void RMDx8Arduino::writeCmd(const uint16_t motor_addr, unsigned char *buf) {
    // CAN通信で送る
    unsigned char sendState = _CAN.sendMsgBuf(motor_addr, 0, 8, buf);
    if (sendState != CAN_OK) {
        Serial.println("Error Sending Message...");
        Serial.println(sendState);
    }
}
