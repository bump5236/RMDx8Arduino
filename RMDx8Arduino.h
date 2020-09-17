
#ifndef RMDx8Arduino_h
#define RMDx8Arduino_h

#include "Arduino.h"
#include <mcp_can.h>


class RMDx8Arduino {
public:
    unsigned char len;
    unsigned char tmp_buf[8], cmd_buf[8], reply_buf[8], pos_buf[8];
    uint16_t MOTOR_ADDRESS;

    RMDx8Arduino(MCP_CAN &CAN, const uint16_t motor_addr);    // クラスと同一の名前にするとコンストラクタ扱い

    // Commands
    void canSetup();
    void readPID();
    void writePID(int posKp, int posKi, int velKp, int velKi, int iqKp, int iqKi);
    void writeEncoderOffset(uint16_t offset);
    void readAngle(char n);
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
    void readBuf(unsigned char *buf);
    void writeCmd(unsigned char *buf);
};

#endif