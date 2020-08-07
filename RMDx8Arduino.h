
#ifndef RMDx8Arduino_h
#define RMDx8Arduino_h

#include "Arduino.h"
#include <mcp_can.h>


class RMDx8Arduino {
public:
    // この変数たちは関数ごとに書いておいたほうがいいのかな？
    unsigned char len = 0;
    unsigned char cmd_buf[8], reply_buf[8];

    RMDx8Arduino(MCP_CAN &CAN);    // クラスと同一の名前にするとコンストラクタ扱い

    // Commands
    void ppp(const unsigned char motor_addr);
    void canSetup();
    void readState();
    void clearState(const unsigned char motor_addr);
    void readPID(const unsigned char motor_addr);
    void writePID(const unsigned char motor_addr, int posKp, int posKi, int velKp, int velKi, int iqKp, int iqKi);
    void writeCurrent(const unsigned char motor_addr, int16_t current);
    void writeVelocity(const unsigned char motor_addr, int32_t velocity); 
    void writePosition(const unsigned char motor_addr, int32_t position);

    // General function
    void serialWriteTerminator();

private:
    MCP_CAN _CAN;
    void writeCmd(const unsigned char motor_addr, unsigned char *buf);
};

#endif