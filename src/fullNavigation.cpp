#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <thread>
#include <cmath>
#include <wiringSerial.h>

#include "JoyStick.h"
#include "PIDController.h"
#include "MagneticSensor.h"
#include "SRampGenerator.h"
#include "QRCode.h"
#include "Car.h"

#define _USE_MATH_DEFINES
#define JS_PATH "/dev/input/js0"
#define MS_PATH1 "/dev/ttyAMA1"
#define MS_PATH2 "/dev/ttyAMA2"
#define MAX_SPEED (1000)
#define S_RAMP_TIME (150)

enum OperationMode
{
    None,
    Manual,
    Auto,
    WaitingTrack,
};
OperationMode mode = None;

bool slowFlag = false;
bool qrLocalizaiton = false;
bool noStop = true;

std::thread *showStatus_t;
float angle_status, d_status;

//Magnetic Sensor Variables
MagneticSensor *magSen1, *magSen2;
int lastOffset = 0;

//QRCode Variables
QRCode *myQR;
int16_t xpos, ypos, qrAngle;
uint32_t tagNum;

//JoyStick Variables
int js;
std::thread *joyStickReceive_t;
bool isJoyStickAlive = false;

//Car Variables
SRampGenerator rampGenerator;
Car *dejaVu;
float setV = 0;
float setV_prev = 0;
float setW = 0.0f;
int dir = 1;

//PID Controllers
PIDContorller headingPID(0.9, 0, 9, 3, -3, 0.2);
PIDContorller offsetPID(0.02, 0.0, 0.02, 3, -3, 0.2);
PIDContorller linearPID(2, 0.003, 0.00, 1000, -1000, 10);

int calcPose(float &angle, float &d);
void followTrack(void);
void joyStickReceive(void);
void showStatus(void);

int main(int argc, char **argv)
{
    magSen1 = new MagneticSensor(MS_PATH1);
    magSen2 = new MagneticSensor(MS_PATH2);
    dejaVu = new Car();
    myQR = new QRCode();

    js = open(JS_PATH, O_RDONLY);
    if (js == -1)
    {
        perror("Could not open joystick");
        exit(0);
    }
    isJoyStickAlive = true;
    joyStickReceive_t = new std::thread(joyStickReceive);

    showStatus_t = new std::thread(showStatus);

    while (1)
    {
        if (isJoyStickAlive == false)
        {
            perror("JoyStick Disconnected");
            exit(0);
        }
        switch (mode)
        {
        case None:
            break;
        case Manual:
            calcPose(angle_status, d_status);
            // system("clear");
            // printf("Angle = %3.2f, d = %3.2f\n", a / M_PI * 180.0f, b);
            // printf("width = %3d\n", magSen1->getTrackWidth());
            break;
        case Auto:
            followTrack();
            break;
        case WaitingTrack:
            if ((magSen1->getTrackCount() > 0) && (magSen2->getTrackCount() > 0))
            {
                dir *= -1;
                offsetPID.clear();
                headingPID.clear();
                linearPID.clear();
                mode = Auto;
                rampGenerator.generateVelocityProfile(dir * MAX_SPEED, S_RAMP_TIME);
            }
            else
            {
                setV = rampGenerator.getV();
                dejaVu->setParams(setV, 0);
                usleep(10000);
            }
            break;
        }
        //system("clear");
        // printf("Count = %d\n", magSen1->getTrackCount());
        // printf("Width = %d\n", magSen1->getTrackWidth());
        // printf("Offset0 = %d\n", magSen1->getTrackOffset(0));
        // printf("Offset1 = %d\n", magSen1->getTrackOffset(1));
        //usleep(100000);
    }

    return 0;
}

int calcPose(float &angle, float &d)
{
    if (magSen1->getTrackCount() >= 1 && magSen2->getTrackCount() >= 1)
    {
        int16_t offset1, offset2;
        int16_t offset1_prev, offset2_prev;
        // float angle;
        // float d;
        if (magSen1->getTrackCount() == 2)
        {
            offset1 = abs(magSen1->getTrackOffset(0)) < abs(magSen1->getTrackOffset(1)) ? magSen1->getTrackOffset(0) : magSen1->getTrackOffset(1);
        }
        else
        {
            offset1 = magSen1->getTrackOffset(0);
        }
        if (magSen2->getTrackCount() == 2)
        {
            offset2 = abs(magSen2->getTrackOffset(0)) < abs(magSen2->getTrackOffset(1)) ? magSen2->getTrackOffset(0) : magSen2->getTrackOffset(1);
        }
        else
        {
            offset2 = magSen2->getTrackOffset(0);
        }

        // offset1 = magSen1->getTrackOffset(0);
        // offset2 = magSen2->getTrackOffset(0);

        offset1_prev = offset1;
        offset2_prev = offset2;
        //printf("off1 = %3d\t off2 = %3d\n", offset1, offset2);
        if ((offset1 == 0) || (offset2 == 0))
        {
            if (offset1 == offset2)
            {
                //printf("Both offset is Zero\n");
                angle = 0;
                d = 0;
            }
            else
            {
                //printf("One offset is Zero\n");
                if (offset1 == 0)
                {
                    angle = -atan2(offset2, CAR_LENGTH);
                    d = cos(angle) * offset2 / 2.0f;
                }
                else
                {
                    angle = -atan2(offset1, CAR_LENGTH);
                    d = -cos(angle) * offset1 / 2.0f;
                }
            }
        }
        else if ((offset1 & 0x8000) ^ (offset2 & 0x8000)) //Determine if AGV Center crosses track
        {
            //printf("Same Side\n");
            if (abs(offset1) == abs(offset2))
            {
                angle = 0;
                d = -offset1;
            }
            else
            {

                if (offset1 > 0)
                {
                    angle = atan2(abs(offset2) - abs(offset1), CAR_LENGTH);
                    d = -cos(angle) * (abs(offset1) + abs(offset2)) / 2.0f;
                }
                else
                {
                    angle = -atan2(abs(offset2) - abs(offset1), CAR_LENGTH);
                    d = cos(angle) * (abs(offset1) + abs(offset2)) / 2.0f;
                }
            }
        }
        else
        {
            //printf("Diff Side\n");
            angle = -atan2(offset1, fabs(offset1 * CAR_LENGTH / abs(offset1 + offset2)));
            if (abs(offset2) > abs(offset1))
            {
                d = -sin(angle) * abs(offset2 - offset1) * CAR_LENGTH / (2.0f * abs(offset1 + offset2));
            }
            else
            {
                d = sin(angle) * abs(offset1 - offset2) * CAR_LENGTH / (2.0f * abs(offset1 + offset2));
            }
        }
        //printf("Angle = %3.2f, d = %3.2f\n", angle / M_PI * 180.0f, d);
        return 1;
    }
    else
    {
        return 0;
    }
}

void followTrack(void)
{
    // system("clear");
    float angle, d;
    float setV_prev = 0;
    int qrDetected = 0;
    if (calcPose(angle, d))
    {
        angle_status = angle;
        d_status = d;
        qrDetected = myQR->getInformation(xpos, ypos, qrAngle, tagNum);
        if (qrDetected && (noStop == false))
        {
            if ((tagNum == 2) && (slowFlag == false) && (setV < 0))
            {
                slowFlag = true;
                rampGenerator.generateVelocityProfile(dir * 200, 75);
            }
            if ((tagNum == 3) && (slowFlag == false) && (setV > 0))
            {
                slowFlag = true;
                rampGenerator.generateVelocityProfile(dir * 200, 75);
            }

            if ((tagNum == 1) && (slowFlag == true))
            {
                qrLocalizaiton = true;
            }
            if ((tagNum == 4) && (slowFlag == true))
            {
                qrLocalizaiton = true;
            }
        }

        if (qrLocalizaiton)
        {
            if (qrDetected)
            {
                setV = linearPID.calculate(-xpos);
                setW = dir * headingPID.calculate(0, dir * angle);
                if ((abs(xpos) < 3) && (abs(setV) < 10))
                {
                    sleep(3);
                    mode = Auto;
                    offsetPID.clear();
                    headingPID.clear();
                    linearPID.clear();
                    if ((qrLocalizaiton == true) && (tagNum == 1))
                    {
                        dir *= -1;
                    }
                    rampGenerator.generateVelocityProfile(0, dir * MAX_SPEED, S_RAMP_TIME);
                    setV_prev = 0;
                    qrLocalizaiton = false;
                    slowFlag = false;
                }
            }
            else
            {
                setV = 0;
                setW = 0;
            }
        }
        else
        {
            setV = rampGenerator.getV();
            setW = dir * headingPID.calculate(offsetPID.calculate(0, d), dir * angle);
        }

        // if (qrLocalizaiton)
        // {
        //     setV = linearPID.calculate(-xpos);
        // }
        // else
        // {
        //     setV = rampGenerator.getV();
        //     setW = dir * headingPID.calculate(offsetPID.calculate(0, d), dir * angle);
        // }
        //setW = offsetPID.calculate(0, d);
        //setW = headingPID.calculate(0, angle);
    }
    else
    {
        setW = 0;
        rampGenerator.generateVelocityProfile(0, 110);
        for (size_t i = 0; i < rampGenerator.getTotalTimeFrames(); i++)
        {
            setV = rampGenerator.getV();
            dejaVu->setParams(setV, setW);
            usleep(10000);
        }
        mode = WaitingTrack;
        slowFlag = false;
        rampGenerator.generateVelocityProfile(0, -dir * 500, 80);
    }
    dejaVu->setParams(setV, setW);
    setV_prev = setV;
    // printf("Angle = %3.2f, d = %3.2f\n", angle / M_PI * 180.0f, d);
    // printf("V = %3.2f\tW = %3.2f\n", setV, setW);
    usleep(10000);
}

void joyStickReceive(void)
{
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    while (read_event(js, &event) == 0)
    {
        switch (event.type)
        {
        case JS_EVENT_BUTTON:
            printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
            if (event.value)
            {
                if (event.number == 8)
                {
                    printf("Manual Mode\n");
                    mode = Manual;
                    setV = 0;
                    setW = 0;
                    dejaVu->setParams(setV, setW);
                }
                else if (event.number == 9)
                {
                    printf("Auto Mode\n");
                    mode = Auto;
                    offsetPID.clear();
                    headingPID.clear();
                    linearPID.clear();
                    if ((qrLocalizaiton == true) && (tagNum == 1))
                    {
                        dir *= -1;
                    }
                    rampGenerator.generateVelocityProfile(0, dir * MAX_SPEED, S_RAMP_TIME);
                    setV_prev = 0;
                    qrLocalizaiton = false;
                    slowFlag = false;
                }
                else if (event.number == 0)
                {
                    isJoyStickAlive = false;
                    exit(0);
                }
                else if (event.number == 4)
                {
                    dejaVu->setParams(0, 0);
                    dejaVu->disableDrivers();
                    dejaVu->clearError();
                }
                else if (event.number == 5)
                {
                    dejaVu->enableDrivers();
                }
                else if (event.number == 3)
                {
                    dir *= -1;
                    printf("DIR CHANGED!\n");
                }
                else if (event.number == 1)
                {
                    noStop ^= 1;
                }
            }
            break;
        case JS_EVENT_AXIS:
            axis = get_axis_state(&event, axes);
            if (axis < 3)
                //printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                switch (axis)
                {
                case 0:
                    if (mode == Manual)
                    {
                        setV = -(float)axes[axis].y / 32767.0 * 800.0;
                        dejaVu->setParams(setV, setW);
                    }
                    break;
                case 1:
                    if (mode == Manual)
                    {
                        setW = (float)axes[axis].x / 32767.0 * 1.5;
                        dejaVu->setParams(setV, setW);
                    }
                    break;
                case 2:
                    break;
                }

            break;
        default:
            /* Ignore init events. */
            break;
        }
        // printf("V = %3.2f\tW = %3.2f\n", setV, setW);
    }

    isJoyStickAlive = false;

    close(js);
}

void showStatus(void)
{
    while (isJoyStickAlive)
    {
        system("clear");
        printf("V = %3.2f\tW = %3.2f\n", setV, setW);
        printf("Angle = %3.2f, d = %3.2f\n", angle_status / M_PI * 180.0f, d_status);
        printf("X: %d\tY: %d\tAngle: %d\tTagNum: %d\n", xpos, ypos, qrAngle, tagNum);
        usleep(100000);
    }
}