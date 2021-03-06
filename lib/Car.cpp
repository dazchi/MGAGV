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
    modbus_set_response_timeout(modbus_ctx, 0, 100000);

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

    carComm_t = new std::thread(&Car::carComm, this);
}

Car::~Car()
{
    modbus_close(modbus_ctx);
    modbus_free(modbus_ctx);
    free(carComm_t);
}

void Car::setParams(int16_t V, float W)
{
    Vl = (float)V - ((W * AXLE_LENGTH) / 2.0f);
    Vr = (float)V + ((W * AXLE_LENGTH) / 2.0f);

    // printf("set: Vl = %f, Vr = %f\n", Vl, Vr);
    setFlag = true;
    //run();
}

void Car::run()
{
    //  printf("run:Vl = %f, Vr = %f\n", Vl, Vr);
    modbus_set_slave(modbus_ctx, 1);
    // modbus_write_register(modbus_ctx, 0x203A, convertToRPM(Vl)); //Set Target Velocity
    if (modbus_write_register(modbus_ctx, 0x203A, convertToRPM(Vl)) == -1)
    {
        perror("NO1");
    }
    usleep(SEND_DELAY);
    modbus_set_slave(modbus_ctx, 2);
    // modbus_write_register(modbus_ctx, 0x203A, convertToRPM(-Vr)); //Set Target Velocity
    if (modbus_write_register(modbus_ctx, 0x203A, -convertToRPM(Vr)) == -1)
    {
        perror("NO2");
    }
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

//Convert mm/s to RPM
int16_t Car::convertToRPM(float v)
{
    //printf("RPM = %f\n", (v / wheel_circumference) * 60.0f);
    return ((v / wheel_circumference) * 60.0f);
}

void Car::clearError(void)
{
    modbus_set_slave(modbus_ctx, 0);
    modbus_write_register(modbus_ctx, 0x2031, 0x0006);
    usleep(SEND_DELAY);
}

void Car::carComm(void)
{
    int count = 0;
    while (1)
    {
        // if ((count > 700) )
        // {
        //     printf("count = %d\n",count);
        //     run();
        //     count = 0;            
        // }
        // if(setFlag == true){
        //     run();            
        //     setFlag = false;           
        // }
        run();
        count++;
        usleep(10000);
    }
}

void Car::getRPM(void)
{
    uint16_t rxBuffer;
    modbus_set_slave(modbus_ctx, 1);
    modbus_read_registers(modbus_ctx, 0x202C, 1, &rxBuffer);
    RPML = ((int16_t)rxBuffer) / 10.0;
    usleep(SEND_DELAY);
    modbus_set_slave(modbus_ctx, 2);
    modbus_read_registers(modbus_ctx, 0x202C, 1, &rxBuffer);
    RPMR = ((int16_t)rxBuffer) / 10.0;
}