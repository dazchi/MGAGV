#include "QRCode.h"

QRCode::QRCode(/* args */)
{
    if (wiringPiSetupGpio() < 0)
    { //use BCM2835 Pin number table
        printf("set wiringPi lib failed	!!! \r\n");
        printf("Failed Initializing QRCode!\n");
        return;
    }
    else
    {
        //printf("set wiringPi lib success  !!! \r\n");
    }

    pinMode(EN_485, OUTPUT);
    digitalWrite(EN_485, HIGH);

    fd = serialOpen("/dev/ttyUSB0", 115200);
    if (fd < 0)
    {
        throw "Serial Error!";
    }
    printf("QRCode Initialized!\n");
    serialFlush(fd);
}

uint8_t QRCode::getInformation(int16_t &x, int16_t &y, int16_t &angle, uint32_t &tagnum)
{
    char rxbuff[21] = {0};
    uint16_t timeout_count = 0;

    serialFlush(fd);
    for (int i = 0; i < 2; i++)
    {
        serialPutchar(fd, txData[i]);
    }

    //printf("Data sENT\n");

    while (serialDataAvail(fd) == 0){
        if(timeout_count++ > 5){
            return 0;
        }
        usleep(1000);
    };
    for (int i = 0; i < 21; i++)
    {
        rxbuff[i] = serialGetchar(fd);
        //printf("%02X\t", rxbuff[i]);
    }
    //puts("");

    if ((rxbuff[0] & 0b00000010) > 0)
    {        
        return 0;
    }

    x = (int16_t)(((rxbuff[4] & 0x7F) << 7) | (rxbuff[5] & 0x7F));
    x = (int16_t)((x & 0x2000) > 0 ? -(~(x | 0xC000) + 1) : x);
    y = (int16_t)(((rxbuff[6] & 0x7F) << 7) | (rxbuff[7] & 0x7F));
    y = (int16_t)((y & 0x2000) > 0 ? -(~(y | 0xC000) + 1) : y);
    angle = (int16_t)(((rxbuff[10] & 0x7F) << 7) | (rxbuff[11] & 0x7F));
    angle -=135;
    angle = (360 - angle);
    angle = angle > 180 ? angle - 360 : angle;
    //angle = (int16_t)(angle > 180 ? angle - 360 : angle);
    tagnum = (uint32_t)(((rxbuff[13] & 0x0F) << 7) | (rxbuff[14] & 0x7F));
    tagnum <<= 14;
    tagnum |= (uint32_t)(((rxbuff[15] & 0x7F) << 7) | (rxbuff[16] & 0x7F));
    tagnum <<= 7;
    tagnum |= (uint32_t)(rxbuff[17] & 0x7F);

    // delay(5);
    return 1;
}

QRCode::~QRCode()
{
    serialClose(fd);
}
