#include "MagneticSensor.h"

MagneticSensor::MagneticSensor(const char *devName)
{
    fd = serialOpen("/dev/ttyAMA1", 115200);
    if (fd == -1)
    {
        perror("Failed to Open Magnetic Sensor");
        //std::cout << "Magnetic Sensor: Failed to Open " << devName << std::endl;
        return;
    }
    std::cout << "Magnetic Sensor Initialized!" << std::endl;

    receivePacket_t = new std::thread(&MagneticSensor::receivePacket, this);
}

MagneticSensor::~MagneticSensor()
{
    isUpdating = false;
    receivePacket_t->join();
    free(receivePacket_t);
}

void MagneticSensor::receivePacket(void)
{
    while (isUpdating)
    {
        if (serialDataAvail(fd))
        {
            rxBuff[rxIndex].byte = serialGetchar(fd);
            if ((rxBuff[0].trackNo.format) == 0x39)
            {
                rxIndex++;

                trackCount = rxBuff[0].trackNo.trackCount;
                if (rxIndex == (trackCount ? 3 : 1))
                {
                    //system("clear");
                    //printf("packet receive, length = %d\n", rxIndex);
                    //printf("Track Count = %d\n", rxBuff[0].trackNo.trackCount);
                    switch (trackCount)
                    {
                    case 1:
                        //printf("Track Offset = %d\n", (rxBuff[1].trackOffset.sign ? -1 : 1) * rxBuff[1].trackOffset.offset << 1);
                        //printf("Track Width = %d\n", (((rxBuff[2].trackWidth & 0xC0) >> 2) + (rxBuff[2].trackWidth - 0x29)) << 1);
                        trackOffset[0] = (rxBuff[1].trackOffset.sign ? -1 : 1) * rxBuff[1].trackOffset.offset << 1;
                        trackWidth = (((rxBuff[2].trackWidth & 0xC0) >> 2) + (rxBuff[2].trackWidth - 0x29)) << 1;
                        break;
                    case 2:
                        //printf("Track 1: Offset = %d\n", (rxBuff[1].trackOffset.sign ? -1 : 1) * rxBuff[1].trackOffset.offset << 1);
                        //printf("Track 2: Offset = %d\n", (rxBuff[2].trackOffset.sign ? -1 : 1) * rxBuff[2].trackOffset.offset << 1);
                        trackOffset[0] = (rxBuff[1].trackOffset.sign ? -1 : 1) * rxBuff[1].trackOffset.offset << 1;
                        trackOffset[1] = (rxBuff[2].trackOffset.sign ? -1 : 1) * rxBuff[2].trackOffset.offset << 1;
                        break;
                    default:
                        break;
                    }
                    rxIndex = 0;
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
}

int MagneticSensor::getTrackCount(void)
{
    return trackCount;
}

int MagneticSensor::getTrackOffset(int num)
{
    if (trackCount == 1)
    {
        if (num == 0)
        {
            return trackOffset[0];
        }        
    }
    else if (trackCount == 2)
    {
        if ((num == 0) || (num == 1))
        {
            return trackOffset[num];
        }            
    }
    return -1;
}

int MagneticSensor::getTrackWidth(void)
{
    if (trackCount == 1)
    {
        return trackWidth;
    }
    return -1;
}
