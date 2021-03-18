#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <errno.h>
#include <modbus.h>
#include <wiringSerial.h>
#include "JoyStick.h"
#include "QRCode.h"

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
    QRCode qrCode;
    int16_t x, y, angle;
    uint32_t tagnum;

    while (1)
    {
        if (qrCode.getInformation(x, y, angle, tagnum))
        {
            printf("X: %d\tY: %d\tAngle: %d\tTagNum: %d\n", x, y, angle, tagnum);
        }
        usleep(10000);
    }

    return 0;
}