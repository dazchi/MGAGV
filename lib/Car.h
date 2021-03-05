#ifndef CAR_H
#define CAR_H

#include <cstdint>
#include <cstdio>
#include <cerrno>
#include <unistd.h>
#include <modbus.h>

#define WHEEL_RADIUS (64) //Wheel radius in mm
#define AXLE_LENGTH (310) //Axle effective length in mm
#define SEND_DELAY (5000)
#define PI (3.14159265359f)

class Car
{
private:
    modbus_t *modbus_ctx;
    const float wheel_circumference = 2.0f * WHEEL_RADIUS * PI;  //Wheel's Circumference in mm
    int16_t V = 0;  //Car Linear Velocity (mm/s)
    float W = 0.0;  //Car Angular Velocity (1000*rad/s)
    float Vl = 0.0; //Left Wheel Velocity (mm/s)
    float Vr = 0.0; //Right Wheel Velocity (mm/s)
    int16_t convertToRPM(float v);

public:
    Car(/* args */);
    ~Car();
    void run(int16_t _V, float _W);
    void enableDrivers(void);
    void disableDrivers(void);
    void sendHeartbeat(void);
    void clearError(void);
};

#endif