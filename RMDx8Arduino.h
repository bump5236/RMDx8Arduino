
#ifndef RMDx8Arduino_h
#define RMDx8Arduino_h

#include "Arduino.h"
#include <mcp_can.h>


class RMDx8Arduino {
public:
    unsigned char len;
    unsigned char tmp_buf[8], cmd_buf[8], reply_buf[8], pos_buf[8];

    RMDx8Arduino(MCP_CAN &CAN);    // クラスと同一の名前にするとコンストラクタ扱い

    // Commands
    void canSetup();
    void readPID(const uint16_t motor_addr);
    void writePID(const uint16_t motor_addr, int posKp, int posKi, int velKp, int velKi, int iqKp, int iqKi);
    void writeEncoderOffset(const uint16_t motor_addr, uint16_t offset);
    void readAngle(const uint16_t motor_addr, char n);
    void clearState(const uint16_t motor_addr);
    void writeCurrent(const uint16_t motor_addr, int16_t current);
    void writeVelocity(const uint16_t motor_addr, int32_t velocity); 
    void writePosition(const uint16_t motor_addr, int32_t position);
    void writePosition(const uint16_t motor_addr, int32_t position, uint16_t max_speed);
    void writePosition(const uint16_t motor_addr, uint16_t position, uint8_t spin_direction);
    void writePosition(const uint16_t motor_addr, uint16_t position, uint16_t max_speed, uint8_t spin_direction);

    // General function
    void serialWriteTerminator();

private:
    MCP_CAN _CAN;
    void readBuf(const uint16_t motor_addr, unsigned char *buf);
    void writeCmd(const uint16_t motor_addr, unsigned char *buf);
};

#endif