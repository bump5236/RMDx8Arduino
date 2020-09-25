
#ifndef RMDx8Arduino_h
#define RMDx8Arduino_h

#include "Arduino.h"
#include <mcp_can.h>


class RMDx8Arduino {
public:
    unsigned char len;
    unsigned char tmp_buf[8], cmd_buf[8], reply_buf[8], pos_buf[8];
    int8_t temperature;
    uint8_t posKp, posKi, velKp, velKi, curKp, curKi;
    int16_t current, velocity;
    uint16_t MOTOR_ADDRESS, encoder_pos;
    int32_t position;

    RMDx8Arduino(MCP_CAN &CAN, const uint16_t motor_addr);    // クラスと同一の名前にするとコンストラクタ扱い

    // Commands
    void canSetup();
    void readPID();
    void writePID(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);
    void writeEncoderOffset(uint16_t offset);
    void readPosition();
    void clearState();
    void writeCurrent(int16_t current);
    void writeVelocity(int32_t velocity); 
    void writePosition(int32_t position);
    void writePosition(int32_t position, uint16_t max_speed);
    void writePosition(uint16_t position, uint8_t spin_direction);
    void writePosition(uint16_t position, uint16_t max_speed, uint8_t spin_direction);

    // General function
    void serialWriteTerminator();

private:
    MCP_CAN _CAN;
    uint32_t pos_u32t;

    void readBuf(unsigned char *buf);
    void writeCmd(unsigned char *buf);
};

#endif