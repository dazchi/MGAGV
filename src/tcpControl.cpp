#include <iostream>
#include <unistd.h>
#include <thread>
#include <sys/socket.h> //for TCP socket
#include <netinet/in.h>
#include <netdb.h> //hostent
#include <arpa/inet.h>

#include "Car.h"
#include "SRampGenerator.h"

#define MAX_BUFFER_SIZE (64)

float setV = 0;
float setW = 0.0f;
Car *myCar;
SRampGenerator rampGenerator;

//TCP Socket Vars
int sock0;
struct sockaddr_in addr;
struct sockaddr_in client;
socklen_t len;
int sock_client;
std::thread *handleClientMessage_t;
bool closeFlag = false;
bool stopFlag = false;
int heartbeatTimeout = 0;

void handleClientMessage(void);

int main(int argc, char **argv)
{
    myCar = new Car();
    myCar->setParams(0, 0);

    /* 製作 socket */
    sock0 = socket(AF_INET, SOCK_STREAM, 0);

    /* 設定 socket */
    addr.sin_family = AF_INET;
    addr.sin_port = htons(6666);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
    printf("\t[Info] binding...\n");

    /* 設定成等待來自 TCP 用戶端的連線要求狀態 */
    listen(sock0, 5);
    printf("\t[Info] listening...\n");

    /* 接受來自 TCP 用戶端地連線要求*/
    printf("\t[Info] wait for connection...\n");
    len = sizeof(client);
    sock_client = accept(sock0, (struct sockaddr *)&client, &len);
    printf("\t[Info] Testing...\n");
    char *paddr_str = inet_ntoa(client.sin_addr);
    printf("\t[Info] Receive connection from %s...\n", paddr_str);

    handleClientMessage_t = new std::thread(handleClientMessage);

    // /* 傳送 5 個字元 */
    // printf("\t[Info] Say hello back...\n");
    // write(sock_client, "HELLO\n", 6);
    while (!closeFlag)
    {
        if (stopFlag)
        {
            int16_t setV_ramp = rampGenerator.getV();
            printf("%d\n", setV_ramp);
            myCar->setParams(setV_ramp, 0);
            if (setV_ramp == 0)
            {
                stopFlag = false;
                puts("stop flag  stopped");
            }
        }
        else
        {
            myCar->setParams(setV, setW);
        }
        if (heartbeatTimeout > 10)
        {
            setV = 0;
            setW = 0;
        }
        heartbeatTimeout++;
        usleep(10000);
    }
    closeFlag = true;
    handleClientMessage_t->join();

    return 0;
}

void handleClientMessage(void)
{
    char rxBuffer[MAX_BUFFER_SIZE];
    int rxCount;

    while (!closeFlag)
    {
        rxCount = read(sock_client, rxBuffer, MAX_BUFFER_SIZE);
        if (rxCount > 0)
        {
            int16_t v;
            float w;
            float VBAT = myCar->getDriverVoltage();
            v = *((int16_t *)(&rxBuffer[0]));
            w = *((float *)(&rxBuffer[2]));
            if (abs(setV - v) > 50)
            {
                stopFlag = true;
                rampGenerator.generateVelocityProfile(setV, 0, 100);
                puts("stop flag");
            }
            setV = v;
            setW = w;
            // printf("\n");
            //printf("V = %d\tW = %f\nVBAT = %2.2f\n", v, w, VBAT);
            write(sock_client, (char *)(&VBAT), 4);
            heartbeatTimeout = 0;
        }
        else
        {
            myCar->setParams(0, 0);
            closeFlag = true;
        }
    }

    printf("\t[Info] Close client connection...\n");
    close(sock_client);

    /* 結束 listen 的 socket */
    printf("\t[Info] Close self connection...\n");
    close(sock0);
}
