
#include "Arduino.h"
#include "RMDx8Arduino.h"
#include <mcp_can.h>

// constructor
RMDx8Arduino::RMDx8Arduino(MCP_CAN &CAN, const uint16_t motor_addr) 
    :_CAN(CAN){
        MOTOR_ADDRESS = motor_addr;
    }


void RMDx8Arduino::canSetup() {
    while (CAN_OK != _CAN.begin(CAN_1000KBPS)) {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}


void RMDx8Arduino::readPID() {
    cmd_buf[0] = 0x30;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    
    // Send message
    writeCmd(cmd_buf);
    delay(100);
    readBuf(cmd_buf);

    posKp = reply_buf[2];
    posKi = reply_buf[3];
    velKp = reply_buf[4];
    velKi = reply_buf[5];
    curKp = reply_buf[6];
    curKi = reply_buf[7];
}


void RMDx8Arduino::writePID(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
    cmd_buf[0] = 0x31;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = anglePidKp;
    cmd_buf[3] = anglePidKi;
    cmd_buf[4] = speedPidKp;
    cmd_buf[5] = speedPidKi;
    cmd_buf[6] = iqPidKp;
    cmd_buf[7] = iqPidKi;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);
}


void RMDx8Arduino::writeEncoderOffset(uint16_t offset) {
    cmd_buf[0] = 0x91;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = offset & 0xFF;
    cmd_buf[7] = (offset >> 8) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);
}


int32_t RMDx8Arduino::readPosition() {
    cmd_buf[0] = 0x92;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    pos_u32t = ((uint32_t)cmd_buf[4] << 24) + ((uint32_t)cmd_buf[3] << 16) + ((uint32_t)cmd_buf[2] << 8) + cmd_buf[1];

    if (pos_u32t > 2147483648) {
        position = pos_u32t - 4294967296;
    }
    else {
        position = pos_u32t;
    }

    return position;
}


void RMDx8Arduino::clearState() {
    cmd_buf[0] = 0x80;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);
}


/**
 * current is int16_t type, the value range:-2000~2000, corresponding to the actual torque current range -12.5A ~ 12.5A.
 * (the bus current and the actual torque of motor vary with different motors)
 */
void RMDx8Arduino::writeCurrent(int16_t current) {
    cmd_buf[0] = 0xA1;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = current & 0xFF;
    cmd_buf[5] = (current >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    temperature = reply_buf[1];
    current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}


/**
 * velocity is int32_t type, which corresponds to the actual speed of 0.01 dps/LSB.
 */
void RMDx8Arduino::writeVelocity(int32_t velocity) {
    cmd_buf[0] = 0xA2;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = velocity & 0xFF;
    cmd_buf[5] = (velocity >> 8) & 0xFF;
    cmd_buf[6] = (velocity >> 16) & 0xFF;
    cmd_buf[7] = (velocity >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    temperature = reply_buf[1];
    current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}


/**
 * # Position control command 1, multi turns
 * position is int32_t type, and the actual position is 0.01 degree/LSB, 36000 represents 360°.
 * The motor rotation direction is determined by the difference between the target position and the current position.
 */
void RMDx8Arduino::writePosition(int32_t position) {
    cmd_buf[0] = 0xA3;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = (position >> 16) & 0xFF;
    cmd_buf[7] = (position >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    temperature = reply_buf[1];
    current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}


/**
 * # Position control command 2, multi turns
 * In addition to Position control command 1, the following functions have been added.
 * The control value max_speed limits the maximum speed at which the motor rotates, uint16_t type, corresponding to the actual speed of 1 dps/LSB.
 */
void RMDx8Arduino::writePosition(int32_t position, uint16_t max_speed) {
    cmd_buf[0] = 0xA4;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = max_speed & 0xFF;
    cmd_buf[3] = (max_speed >> 8) & 0xFF;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = (position >> 16) & 0xFF;
    cmd_buf[7] = (position >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    temperature = reply_buf[1];
    current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}


/**
 * # Position control command 3, single turn
 * position is uint16_t type, the value range is 0~35999, and the actual position is 0.01 degree/LSB, the actual angle range is 0°~359.99°.
 * The control value spin_direction sets the direction in which the motor rotates, which is uint8_t type, 0x00 for CW and 0x01 for CCW.
 */
void RMDx8Arduino::writePosition(uint16_t position, uint8_t spin_direction) {
    cmd_buf[0] = 0xA5;
    cmd_buf[1] = spin_direction;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    temperature = reply_buf[1];
    current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}


/** 
 * # Position control command 4, single turn
 * position is uint16_t type, the value range is 0~35999, and the actual position is 0.01 degree/LSB, the actual angle range is 0°~359.99°.
 * The control value max_speed limits the maximum speed at which the motor rotates, uint16_t type, corresponding to the actual speed of 1 dps/LSB.
 * The control value spin_direction sets the direction in which the motor rotates, which is uint8_t type, 0x00 for CW and 0x01 for CCW.
 */
void RMDx8Arduino::writePosition(uint16_t position, uint16_t max_speed, uint8_t spin_direction) {
    cmd_buf[0] = 0xA6;
    cmd_buf[1] = spin_direction;
    cmd_buf[2] = max_speed & 0xFF;
    cmd_buf[3] = (max_speed >> 8) & 0xFF;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    temperature = reply_buf[1];
    current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

// General function
void RMDx8Arduino::serialWriteTerminator() {
    Serial.write(13);
    Serial.write(10);
}


// Private
void RMDx8Arduino::readBuf(unsigned char *buf) {
    delayMicroseconds(50);    // 50us
    if (CAN_MSGAVAIL == _CAN.checkReceive()) {
        _CAN.readMsgBuf(&len, tmp_buf);
        if (tmp_buf[0] == buf[0]) {
            reply_buf[0] = tmp_buf[0];
            reply_buf[1] = tmp_buf[1];
            reply_buf[2] = tmp_buf[2];
            reply_buf[3] = tmp_buf[3];
            reply_buf[4] = tmp_buf[4];
            reply_buf[5] = tmp_buf[5];
            reply_buf[6] = tmp_buf[6];
            reply_buf[7] = tmp_buf[7];
        }
    }
}


void RMDx8Arduino::writeCmd(unsigned char *buf) {
    // CAN通信で送る
    unsigned char sendState = _CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, buf);
    if (sendState != CAN_OK) {
        Serial.println("Error Sending Message...");
        Serial.println(sendState);
    }
}
