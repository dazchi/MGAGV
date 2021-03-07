#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <thread>
#include <cmath>
#include <wiringSerial.h>

#include "JoyStick.h"
#include "PIDController.h"
#include "MagneticSensor.h"
#include "Car.h"

#define _USE_MATH_DEFINES
#define JS_PATH "/dev/input/js0"
#define MS_PATH1 "/dev/ttyAMA1"
#define MS_PATH2 "/dev/ttyAMA2"

enum OperationMode
{
    None,
    Manual,
    Auto
};
OperationMode mode = None;

//Magnetic Sensor Variables
MagneticSensor *magSen1, *magSen2;
int lastOffset = 0;

//JoyStick Variables
int js;
std::thread *joyStickReceive_t;
bool isJoyStickAlive = false;

//Car Variables
Car *dejaVu;
float setV = 0;
float setV_prev = 0;
float setW = 0.0f;

//PID Controllers
PIDContorller headingPID(0.7, 0, 0.2, 3, -3, 10);
PIDContorller offsetPID(0.02, 0.0, 0.001, 3, -3, 10);
PIDContorller linearPID(5, 0.0, 0.00, 600, -1000, 4);

int calcPose(float &angle, float &d);
void followTrack(void);
void joyStickReceive(void);

int main(int argc, char **argv)
{
    magSen1 = new MagneticSensor(MS_PATH1);
    magSen2 = new MagneticSensor(MS_PATH2);
    dejaVu = new Car();

    js = open(JS_PATH, O_RDONLY);
    if (js == -1)
    {
        perror("Could not open joystick");
        exit(0);
    }
    isJoyStickAlive = true;
    joyStickReceive_t = new std::thread(joyStickReceive);

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
            // float a, b;
            // calcPose(a, b);
            break;
        case Auto:
            followTrack();
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
    if (magSen1->getTrackCount() == 1 && magSen2->getTrackCount() == 1)
    {
        int16_t offset1, offset2;
        // float angle;
        // float d;

        offset1 = magSen1->getTrackOffset(0);
        offset2 = magSen2->getTrackOffset(0);
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
    system("clear");
    float angle, d;
    float setV_prev = 0;
    if (calcPose(angle, d))
    {
        setV = linearPID.calculate(200, setV_prev);
        //setW = offsetPID.calculate(headingPID.calculate(0, angle), d);

        setW = headingPID.calculate(offsetPID.calculate(0, d), angle);
        //setW = offsetPID.calculate(0, d);
        //setW = headingPID.calculate(0, angle);
    }
    else
    {
        // setW = 0;
        // if (setV > 0)
        // {
        //     for (size_t i = setV; i > 0; i-=20)
        //     {
        //         dejaVu->setParams(i, 0);
        //         usleep(10000);
        //     }
        //     setV = 0;
        //     dejaVu->setParams(0, 0);
        // }
        setV = 0;
        setW = 0;
    }
    dejaVu->setParams(setV, setW);
    setV_prev = setV;
    printf("Angle = %3.2f, d = %3.2f\n", angle / M_PI * 180.0f, d);
    printf("V = %3.2f\tW = %3.2f\n", setV, setW);
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
                    setV_prev = 0;
                }
                else if (event.number == 0)
                {
                    exit(0);
                }
                else if (event.number == 4)
                {
                    dejaVu->disableDrivers();
                }
                else if (event.number == 5)
                {
                    dejaVu->enableDrivers();
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
                        setV = -(float)axes[axis].y / 32767.0 * 500.0;
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
        printf("V = %3.2f\tW = %3.2f\n", setV, setW);
    }

    isJoyStickAlive = false;

    close(js);
}