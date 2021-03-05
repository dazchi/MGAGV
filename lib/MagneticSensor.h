#ifndef MAGNETICSENSOR_H
#define MAGNETICSENSOR_H

#include <iostream>
#include <unistd.h>
#include <thread>
#include <wiringSerial.h>


class MagneticSensor
{
private:
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

    std::thread *receivePacket_t;
    int fd = -1;
    union packet rxBuff[3];
    int rxIndex = 0;
    bool isUpdating = true;        
    volatile int trackCount;
    volatile int trackOffset[2];
    volatile int trackWidth;

    void receivePacket(void);

public:
    MagneticSensor(const char *devName);
    ~MagneticSensor();

    int getTrackCount(void);
    int getTrackOffset(int num);
    int getTrackWidth();    
};

#endif