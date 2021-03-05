#include "Car.h"

Car::Car(/* args */)
{
    modbus_ctx = modbus_new_rtu("/dev/ttyAMA0", 115200, 'N', 8, 1);
    if (modbus_ctx == NULL)
    {
        fprintf(stdout, "Unable to create the libmodbus context\n");
        return;
    }

    modbus_rtu_set_serial_mode(modbus_ctx, MODBUS_RTU_RS485);
    modbus_set_slave(modbus_ctx, 0);
    modbus_set_byte_timeout(modbus_ctx, 0, 0);
    modbus_set_response_timeout(modbus_ctx, 0, 10000);

    if (modbus_connect(modbus_ctx) == -1)
    {
        fprintf(stdout, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(modbus_ctx);
        return;
    }

    modbus_write_register(modbus_ctx, 0x2032, 0x0003); //Set all drivers to velocity mode
    usleep(SEND_DELAY);
    modbus_write_register(modbus_ctx, 0x2031, 0x0008); //Enable all drivers
    usleep(SEND_DELAY);
}

Car::~Car()
{
    modbus_close(modbus_ctx);
    modbus_free(modbus_ctx);
}

void Car::run(int16_t V, float W)
{
    Vl = V - ((W * AXLE_LENGTH) / 2);
    Vr = V + ((W * AXLE_LENGTH) / 2);

    modbus_set_slave(modbus_ctx, 1);
    modbus_write_register(modbus_ctx, 0x203A, convertToRPM(Vl)); //Set Target Velocity
    usleep(SEND_DELAY);
    modbus_set_slave(modbus_ctx, 2);
    modbus_write_register(modbus_ctx, 0x203A, convertToRPM(-Vr)); //Set Target Velocity
    usleep(SEND_DELAY);
}

void Car::enableDrivers(void)
{
    modbus_set_slave(modbus_ctx, 0);
    modbus_write_register(modbus_ctx, 0x2031, 0x0008); //Enable all drivers
    usleep(SEND_DELAY);
}

void Car::disableDrivers(void)
{
    modbus_set_slave(modbus_ctx, 0);
    modbus_write_register(modbus_ctx, 0x2031, 0x0007); //Enable all drivers
    usleep(SEND_DELAY);
}

void Car::sendHeartbeat(void)
{
    modbus_set_slave(modbus_ctx, 0);
    modbus_read_registers(modbus_ctx, 0x202E, 1, nullptr);
    usleep(SEND_DELAY);
}

//Convert mm/s to RPM
int16_t Car::convertToRPM(float v)
{
    //printf("RPM = %f\n",v / wheel_circumference * 60.0f);
    return (v / wheel_circumference * 60.0f);
}

void Car::clearError(void)
{
    modbus_set_slave(modbus_ctx, 0);
    modbus_write_register(modbus_ctx, 0x2031, 0x0006);
    usleep(SEND_DELAY);
} 