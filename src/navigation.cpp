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
PIDContorller angularPID(0.003, 0.0, 0.000, 3, -3, 100);
PIDContorller linearPID(22, 0.01, 0.00, 1000, -1000, 10);

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
            break;
        case Auto:
            if (magSen1->getTrackCount() == 1)
            {
                setV = linearPID.calculate(500, setV_prev);
                setW = angularPID.calculate(magSen1->getTrackOffset(0));
                lastOffset = magSen1->getTrackOffset(0);
            }
            else
            {
                // if (lastOffset == 80)
                // {
                //     setV = linearPID.calculate(400, setV_prev);
                //     setW = angularPID.calculate(lastOffset);
                // }
                // else if (lastOffset == -80)
                // {
                //     setV = linearPID.calculate(400, setV_prev);
                //     setW = angularPID.calculate(lastOffset);
                // }
                // else
                // {
                //     setV = 0;
                //     setW = 0;
                // }
                setV = 0;
                setW = 0;
            }
            system("clear");
            dejaVu->setParams(setV, setW);
            setV_prev = setV;
            printf("V = %3.2f\tW = %3.2f\tOffset = %2d\n", setV, setW, magSen1->getTrackOffset(0));
            usleep(10000);
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
                    angularPID.clear();
                    linearPID.clear();
                    setV_prev = 0;
                }
                else if (event.number == 0)
                {
                    exit(0);
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
    }

    isJoyStickAlive = false;

    close(js);
}