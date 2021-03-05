#include <iostream>
#include <unistd.h>
#include <thread>
#include <modbus.h>
#include "JoyStick.h"
#include "Car.h"

#define JS_PATH "/dev/input/js0"

void heartBeat(void);
int setV = 0;
float setW = 0.0f;
Car car;

int main(int argc, char **argv)
{
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    std::thread heartBeat_t(heartBeat);

    js = open(JS_PATH, O_RDONLY);

    if (js == -1)
    {
        perror("Could not open joystick");
        exit(0);
    }

    while (read_event(js, &event) == 0)
    {
        switch (event.type)
        {
        case JS_EVENT_BUTTON:
            printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
            switch (event.number)
            {
            case 4:
                car.clearError();
                car.disableDrivers();
                break;
            case 5:
                car.enableDrivers();
                break;
            default:
                break;
            }
            break;
        case JS_EVENT_AXIS:
            axis = get_axis_state(&event, axes);
            if (axis < 3)
                printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
            switch (axis)
            {
            case 0:
                setV = -(float)axes[axis].y / 32767.0 * 100.0;
                //setW = (float)axes[axis].x / 32767.0 * 1.5;
                break;
            case 1:
                setW = (float)axes[axis].x / 32767.0 * 0.5;
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

    close(js);

    // heartBeat_t.join();

    return 0;
}

void heartBeat(void)
{
    while (1)
    {
        car.run(setV, setW);
        usleep(10000);
    }
}