#include <mcp_can.h>
#include <SPI.h>
#include <RMDx8Arduino.h>    //librariesに入っていないと読み込めない

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

#define BAUDRATE 115200

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const uint16_t MOTOR_ADDRESS = 0x141; //0x140 + ID(1~32)
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN
RMDx8Arduino rmd1(CAN, MOTOR_ADDRESS);
// RMDx8Arduino rmd2(CAN, 0x142);

void setup() {
    SERIAL.begin(BAUDRATE);
    delay(1000);

    rmd1.canSetup();
    // rmd2.canSetup();

    rmd1.readPID();
    // rmd2.readPID();

    SERIAL.print("POSKp1:");
    SERIAL.println(rmd1.reply_buf[2]);
    SERIAL.print("POSKi1:");
    SERIAL.println(rmd1.reply_buf[3]);
    
    // SERIAL.print("POSKp2:");
    // SERIAL.println(rmd2.reply_buf[2]);
    // SERIAL.print("POSKi2:");
    // SERIAL.println(rmd2.reply_buf[3]);

    rmd1.writePosition(10000);
    delay(1000);
    
    // rmd2.writePosition(10000);
    // delay(1000);

}

void loop() {
    
}