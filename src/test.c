#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <errno.h>
#include <modbus.h>
#include <wiringSerial.h>
#include "JoyStick.h"

#define JS_PATH "/dev/input/js0"

int motorTest()
{
    modbus_t *ctx;
    uint16_t tab_reg[64];
    int i, rc;

    ctx = modbus_new_rtu("/dev/ttyAMA0", 115200, 'N', 8, 1);
    if (ctx == NULL)
    {
        fprintf(stdout, "Unable to create the libmodbus context\n");
        return -1;
    }
 
    modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
    modbus_set_slave(ctx, 0);
    modbus_set_byte_timeout(ctx, 0, 0);
    modbus_set_response_timeout(ctx, 0, 4000);

    if (modbus_connect(ctx) == -1)
    {
        fprintf(stdout, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // rc = modbus_read_registers(ctx, 0x202E, 1, tab_reg);
    // if (rc == -1)
    // {
    //     fprintf(stdout, "%s\n", modbus_strerror(errno));
    //     return -1;
    // }
    // printf("rc = %d\n", rc);

    // for (i = 0; i < rc; i++)
    // {
    //     printf("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
    // }
    // modbus_write_register(ctx, 0x2031, 0x0006);

    modbus_set_slave(ctx, 0);
    modbus_write_register(ctx, 0x2032, 0x0003);
    usleep(4000);

    // modbus_set_slave(ctx, 1);
    // modbus_write_register(ctx, 0x2032, 0x0003);
    // usleep(4000);
    // modbus_write_register(ctx, 0x2031, 0x0008);
    // usleep(4000);

    // modbus_set_slave(ctx, 2);
    // modbus_write_register(ctx, 0x2032, 0x0003);
    // usleep(4000);
    // modbus_write_register(ctx, 0x2031, 0x0008);
    // usleep(4000);
    int t = 0;

    modbus_set_slave(ctx, 1);
    modbus_write_register(ctx, 0x203A, -30);
    usleep(4000);
    modbus_set_slave(ctx, 2);
    modbus_write_register(ctx, 0x203A, 30);
    usleep(4000);

    modbus_set_slave(ctx, 0);
    modbus_write_register(ctx, 0x2031, 0x0008);
    for (t = 0; t < 30; t++)
    {
        modbus_read_registers(ctx, 0x202E, 1, tab_reg);
        usleep(100000);
    }
    modbus_set_slave(ctx, 1);
    modbus_write_register(ctx, 0x2031, 0x0007);
    usleep(4000);
    modbus_set_slave(ctx, 2);
    modbus_write_register(ctx, 0x2031, 0x0007);

    //     modbus_set_slave(ctx, 1);
    // modbus_write_register(ctx, 0x2031, 0x0007);
    // usleep(4000);
    // modbus_set_slave(ctx, 2);
    // modbus_write_register(ctx, 0x2031, 0x0007);

    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}

union packet
{
    uint8_t byte;
    struct
    {
        uint8_t format : 6;
        uint8_t trackCount : 2;
    } trackNo;
    struct
    {
        uint8_t offset : 6;
        uint8_t sign : 1;
        uint8_t trackNum : 1;
    } trackOffset;
    uint8_t trackWidth;
};

int main(int argc, char **argv)
{
    int fd = 0;
    union packet rxBuff[3];
    int rxIndex = 0;
    fd = serialOpen("/dev/ttyAMA1", 115200);
    if (fd == -1)
    {
        printf("open serial failed\n");
        return -1;
    }
    printf("serialOpen success\n");

    while (1)
    {
        if (serialDataAvail(fd))
        {
            rxBuff[rxIndex].byte = serialGetchar(fd);
            //printf("0x%X 0x%X\t", rxBuff[rxIndex].trackNo.trackCount, rxBuff[rxIndex].trackNo.format);
            if ((rxBuff[0].trackNo.format) == 0x39)
            {
                rxIndex++;
                if (rxIndex == (rxBuff[0].trackNo.trackCount ? 3 : 1))
                {
                    system("clear");
                    printf("packet receive, length = %d\n", rxIndex);
                    printf("Track Count = %d\n", rxBuff[0].trackNo.trackCount);
                    switch (rxBuff[0].trackNo.trackCount)
                    {
                    case 1:
                        printf("Track Offset = %d\n", (rxBuff[1].trackOffset.sign ? -1 : 1) * rxBuff[1].trackOffset.offset << 1);
                        printf("Track Width = %d\n", (((rxBuff[2].trackWidth & 0xC0) >> 2) + (rxBuff[2].trackWidth - 0x29)) << 1);
                        break;
                    case 2:
                        printf("Track 1: Offset = %d\n", (rxBuff[1].trackOffset.sign ? -1 : 1) * rxBuff[1].trackOffset.offset << 1);
                        printf("Track 2: Offset = %d\n", (rxBuff[2].trackOffset.sign ? -1 : 1) * rxBuff[2].trackOffset.offset << 1);
                        break;
                    default:
                        break;
                    }
                    usleep(5000);
                    //sleep(1);
                    rxIndex = 0;
                    serialFlush(fd);
                }
            }
            else
            {
                rxIndex = 0;
                serialFlush(fd);
            }
        }
        usleep(1000);
    }

    serialClose(fd);

    return 0;
}