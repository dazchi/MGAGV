#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <thread>
#include <cmath>
#include <wiringSerial.h>

#include "../lib/JoyStick.h"
#include "../lib/PIDController.h"
#include "../lib/QRCode.h"
#include "../lib/Car.h"

//Car Variables
Car *myCar;
float setV = 0;
float setW = 0.0f;
float setV_js = 0;
float setW_js = 0.0f;
bool manualOverride = false;

//JoyStick Variables
int js;
std::thread *joyStickReceive_t;
bool isJoyStickAlive = false;

//QRCode Variables
QRCode *myQR;
int16_t xpos, ypos, angle;
uint32_t tagNum;

//PID Controllers
PIDContorller headingPID(0.9, 0, 3, 3, -3, 0.2);
PIDContorller linearPID(5, 0.003, 0.00, 1000, -1000, 10);

void joyStickReceive(void);

int main(int argc, char **argv)
{
    myCar = new Car();
    myQR = new QRCode();

    js = open("/dev/input/js0", O_RDONLY);
    if (js == -1)
    {
        perror("Could not open joystick");
        exit(0);
    }
    isJoyStickAlive = true;
    joyStickReceive_t = new std::thread(joyStickReceive);

    while (1)
    {
        if (!manualOverride)
        {
            if (myQR->getInformation(xpos, ypos, angle, tagNum))
            {
                setV = linearPID.calculate(-xpos);
                printf("X: %d\tY: %d\tAngle: %d\tTagNum: %d\tsetV: %4.3f\n", xpos, ypos, angle, tagNum, setV);
            }
            else
            {
                setV = 0;
            }
            myCar->setParams(setV, 0);
        }
        usleep(10000);
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
                if (event.number == 0)
                {
                    for (size_t i = setV; i > 0; i--)
                    {
                        myCar->setParams(i, 0);
                        usleep(1000);
                    }
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
                    setV_js = -(float)axes[axis].y / 32767.0 * 800.0;
                    if (fabs(setV_js) > 0.0)
                    {
                        manualOverride = true;
                    }
                    break;
                case 1:
                    setW_js = (float)axes[axis].x / 32767.0 * 1.5;
                    if (fabs(setW_js) > 0.0)
                    {
                        manualOverride = true;
                    }
                    break;
                case 2:
                    break;
                }

            if ((fabs(setV_js) < 0.1) && (fabs(setW_js) < 0.1))
            {
                setV_js = 0;
                setW_js = 0;
                manualOverride = false;
            }
            myCar->setParams(setV_js, setW_js);
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