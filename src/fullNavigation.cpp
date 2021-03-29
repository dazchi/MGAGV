#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <thread>
#include <cmath>
#include <wiringSerial.h>

#include "../lib/JoyStick.h"
#include "../lib/PIDController.h"
#include "../lib/MagneticSensor.h"
#include "../lib/SRampGenerator.h"
#include "../lib/MovingAverage.h"
#include "../lib/QRCode.h"
#include "../lib/Car.h"

#define _USE_MATH_DEFINES
#define JS_PATH "/dev/input/js0"
#define MS_PATH1 "/dev/ttyAMA1"
#define MS_PATH2 "/dev/ttyAMA2"
#define MAX_SPEED (1000)
#define S_RAMP_TIME (75)
#define TRACK_WIDTH (30)     //Magnetic Track Width in mm
#define FORK_WIDTH_THLD (42) //Fork Threshold Width in mm
#define FORK_MERGE_TIMEOUT (10)

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
MovingAverage offsetFrontMA(100);
MovingAverage offsetRearMA(100);
int forkSelect = 0;      //Left: 0, Right: 1
int forkStateFront = 0;  //0: No fork expected, 1:Expecting Fork, 2:Passing Fork
int forkStateRear = 0;   //0: No fork expected, 1:Expecting Fork, 2:Passing Fork
int mergeStateFront = 0; //0: No merge expected, 1:Expecting Merge from Left, 2:Expecting Merge from Right
int mergeStateRear = 0;  //0: No merge expected, 1:Expecting Merge from Left, 2:Expecting Merge from Right
int mergeTimeoutFront = 0;
int mergeTimeoutRear = 0;
int mergeFlag = 0;

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
float setW = 0.0f;
int dir = 1;

//PID Controllers
PIDContorller headingPID(2, 0.00, 4, 3, -3, 1);
PIDContorller offsetPID(0.01, 0.0, 0.005, 3, -3, 1);
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
                // forkSelect ^= 1;
                offsetPID.clear();
                headingPID.clear();
                linearPID.clear();
                mode = Auto;
                forkStateFront = 0;
                forkStateRear = 0;
                mergeStateFront = 0;
                mergeStateRear = 0;
                rampGenerator.generateVelocityProfile(dir * MAX_SPEED, S_RAMP_TIME);
                // forkSelect = rand() % 2;
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
    MagneticSensor *magSenFront, *magSenRear;
    if (dir == 1)
    {
        magSenFront = magSen1;
        magSenRear = magSen2;
    }
    else
    {
        magSenFront = magSen2;
        magSenRear = magSen1;
    }

    if (magSenFront->getTrackCount() >= 1 && magSenRear->getTrackCount() >= 1)
    {
        int16_t offsetFront, offsetRear;

        if (magSenFront->getTrackCount() == 2) //2 Tracks Detected at Front Sensor
        {
            if (forkStateFront > 0)
            {
                offsetFront = magSenFront->getTrackOffset(forkSelect);
                forkStateFront = 2;
            }
            else
            {
                // if (abs(magSenFront->getTrackOffset(0) - offsetFrontMA.getCurrentAverage()) < abs(magSenFront->getTrackOffset(1) - offsetFrontMA.getCurrentAverage()))
                if (abs(magSenFront->getTrackOffset(0)) < abs(magSenFront->getTrackOffset(1)))
                {
                    mergeStateFront = 2; //Expected Merging from Right
                    offsetFront = magSenFront->getTrackOffset(0);
                }
                else
                {
                    mergeStateFront = 1; //Expected Merging from Left
                    offsetFront = magSenFront->getTrackOffset(1);
                }
                mergeTimeoutFront = 0;
            }
        }
        else
        {
            if (magSenFront->getTrackWidth() > FORK_WIDTH_THLD)
            {
                if (mergeStateFront > 0)
                {
                    if (mergeStateFront == 1) //Merging from Left
                    {
                        offsetFront = std::max(magSenFront->getTrackOffset(0), magSenFront->getTrackOffset(0) + (magSenFront->getTrackWidth() / 2) - (TRACK_WIDTH / 2));
                    }
                    else //Merging from Right
                    {
                        offsetFront = std::min(magSenFront->getTrackOffset(0), magSenFront->getTrackOffset(0) - (magSenFront->getTrackWidth() / 2) + (TRACK_WIDTH / 2));
                    }
                    // offsetFront = magSenFront->getTrackOffset(0);
                    mergeTimeoutFront = 0;
                    mergeStateRear = mergeStateFront;
                    if (mergeFlag == 0)
                    {
                        mergeFlag = 1;
                    }
                }
                else
                {
                    if (forkSelect)
                    {
                        offsetFront = std::max(magSenFront->getTrackOffset(0), magSenFront->getTrackOffset(0) + (magSenFront->getTrackWidth() / 2) - (TRACK_WIDTH / 2));
                    }
                    else
                    {
                        offsetFront = std::min(magSenFront->getTrackOffset(0), magSenFront->getTrackOffset(0) - (magSenFront->getTrackWidth() / 2) + (TRACK_WIDTH / 2));
                    }
                    forkStateFront = 1; //Expecting Fork
                }
            }
            else
            {
                offsetFront = magSenFront->getTrackOffset(0);
                forkStateFront = 0;
                if (mergeTimeoutFront > FORK_MERGE_TIMEOUT)
                {
                    mergeStateFront = 0;
                }
            }
        }
        if (magSenRear->getTrackCount() == 2) //2 Tracks Detected at Rear Sensor
        {
            if (forkStateRear > 0)
            {
                offsetRear = magSenRear->getTrackOffset(forkSelect ^ 1);
                forkStateRear = 2;
            }
            else
            {
                // if (abs(magSenRear->getTrackOffset(0) - offsetRearMA.getCurrentAverage()) < abs(magSenRear->getTrackOffset(1) - offsetRearMA.getCurrentAverage()))
                if (abs(magSenRear->getTrackOffset(0)) < abs(magSenRear->getTrackOffset(1)))
                {
                    // mergeStateRear = 1; //Expected Merging from Left
                    offsetRear = magSenRear->getTrackOffset(0);
                }
                else
                {
                    // mergeStateRear = 2; //Expected Merging from Right
                    offsetRear = magSenRear->getTrackOffset(1);
                }
                mergeTimeoutRear = 0;
            }
        }
        else
        {
            if (magSenRear->getTrackWidth() > FORK_WIDTH_THLD)
            {
                if (mergeStateRear > 0)
                {
                    if (mergeStateRear == 1) //Merging from Left
                    {
                        offsetRear = std::min(magSenRear->getTrackOffset(0), magSenRear->getTrackOffset(0) - (magSenRear->getTrackWidth() / 2) + (TRACK_WIDTH / 2));
                    }
                    else //Merging from Right
                    {
                        offsetRear = std::max(magSenRear->getTrackOffset(0), magSenRear->getTrackOffset(0) + (magSenRear->getTrackWidth() / 2) - (TRACK_WIDTH / 2));
                    }
                    // offsetRear = magSenRear->getTrackOffset(0);
                    mergeTimeoutRear = 0;
                    mergeFlag = 2;
                }
                else
                {
                    if (forkSelect)
                    {
                        offsetRear = std::min(magSenRear->getTrackOffset(0), magSenRear->getTrackOffset(0) - (magSenRear->getTrackWidth() / 2) + (TRACK_WIDTH / 2));
                    }
                    else
                    {
                        offsetRear = std::max(magSenRear->getTrackOffset(0), magSenRear->getTrackOffset(0) + (magSenRear->getTrackWidth() / 2) - (TRACK_WIDTH / 2));
                    }
                    forkStateRear = 1; //Expecting Fork
                }
            }
            else
            {
                offsetRear = magSenRear->getTrackOffset(0);
                forkStateRear = 0;
                if (mergeTimeoutRear > FORK_MERGE_TIMEOUT)
                {
                    mergeStateRear = 0;
                    mergeFlag = 0;
                }
            }
        }
        if (mergeStateFront > 0)
        {
            mergeTimeoutFront++;
        }
        if ((mergeStateRear > 0) && (mergeFlag == 2))
        {
            mergeTimeoutRear++;
        }

        // offsetFront = magSenFront->getTrackOffset(0);
        // offsetRear = magSenRear->getTrackOffset(0);

        if ((offsetFront == 0) || (offsetRear == 0))
        {
            if (offsetFront == offsetRear)
            {
                //printf("Both offset is Zero\n");
                angle = 0;
                d = 0;
            }
            else
            {
                //printf("One offset is Zero\n");
                if (offsetFront == 0)
                {
                    angle = -atan2(offsetRear, CAR_LENGTH);
                    d = cos(angle) * offsetRear / 2.0f;
                }
                else
                {
                    angle = -atan2(offsetFront, CAR_LENGTH);
                    d = -cos(angle) * offsetFront / 2.0f;
                }
            }
        }
        else if ((offsetFront & 0x8000) ^ (offsetRear & 0x8000)) //Determine if AGV Center crosses track
        {
            //printf("Same Side\n");
            if (abs(offsetFront) == abs(offsetRear))
            {
                angle = 0;
                d = -offsetFront;
            }
            else
            {

                if (offsetFront > 0)
                {
                    angle = atan2(abs(offsetRear) - abs(offsetFront), CAR_LENGTH);
                    d = -cos(angle) * (abs(offsetFront) + abs(offsetRear)) / 2.0f;
                }
                else
                {
                    angle = -atan2(abs(offsetRear) - abs(offsetFront), CAR_LENGTH);
                    d = cos(angle) * (abs(offsetFront) + abs(offsetRear)) / 2.0f;
                }
            }
        }
        else
        {
            //printf("Diff Side\n");
            angle = -atan2(offsetFront, fabs(offsetFront * CAR_LENGTH / abs(offsetFront + offsetRear)));
            if (abs(offsetRear) > abs(offsetFront))
            {
                d = -sin(angle) * abs(offsetRear - offsetFront) * CAR_LENGTH / (2.0f * abs(offsetFront + offsetRear));
            }
            else
            {
                d = sin(angle) * abs(offsetFront - offsetRear) * CAR_LENGTH / (2.0f * abs(offsetFront + offsetRear));
            }
        }
        //printf("Angle = %3.2f, d = %3.2f\n", angle / M_PI * 180.0f, d);
        offsetFrontMA.add(offsetFront);
        offsetRearMA.add(offsetRear);
        return 1;
    }
    else
    {
        // printf("offsetFrontPrev = %d\toffsetRearPrev = %d\n", offsetFrontPrev, offsetRearPrev);
        return 0;
    }
}

void followTrack(void)
{
    // system("clear");
    float angle, d;
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
                rampGenerator.generateVelocityProfile(dir * 25, 100);
            }
            if ((tagNum == 3) && (slowFlag == false) && (setV > 0))
            {
                slowFlag = true;
                rampGenerator.generateVelocityProfile(dir * 25, 100);
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

        if (qrLocalizaiton && (abs(xpos) < 20))
        {
            if (qrDetected)
            {
                setV = linearPID.calculate(-xpos);
                setW = dir * headingPID.calculate(0, dir * angle);
                if ((abs(xpos) <= 1) && (abs(setV) < 5) && (fabs(angle) < 0.017f))
                {
                    setV = 0;
                    setW = 0;
                    dejaVu->setParams(setV, setW);
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
            setW = headingPID.calculate(offsetPID.calculate(0, d), angle);
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
        if (abs(setV) > 0)
        {
            mode = WaitingTrack;
            slowFlag = false;
            rampGenerator.generateVelocityProfile(0, 100);
            for (size_t i = 0; i < rampGenerator.getTotalTimeFrames(); i++)
            {
                setV = rampGenerator.getV();
                dejaVu->setParams(setV, setW);
                usleep(10000);
            }
            rampGenerator.generateVelocityProfile(0, -dir * 500, 80);
        }
    }
    dejaVu->setParams(setV, setW);
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
                    setV = 0;
                    setW = 0;
                    dejaVu->setParams(setV, setW);
                    offsetPID.clear();
                    headingPID.clear();
                    linearPID.clear();
                    if ((qrLocalizaiton == true) && (tagNum == 1))
                    {
                        dir *= -1;
                    }
                    rampGenerator.generateVelocityProfile(0, dir * MAX_SPEED, S_RAMP_TIME);
                    qrLocalizaiton = false;
                    slowFlag = false;
                }
                else if (event.number == 0)
                {
                    isJoyStickAlive = false;
                    dejaVu->setParams(0, 0);
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
                else if (event.number == 2)
                {
                    forkSelect ^= 1;
                    if (forkSelect)
                    {
                        printf("ForkSelect = Right");
                    }
                    else
                    {
                        printf("ForkSelect = Left");
                    }
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
        printf("V = %3.2f\tW = %3.2f\tDir = %d\tForkSelect = %s\n", setV, setW, dir, forkSelect ? "Right" : "Left");
        printf("ForkStateFront = %d\tForkStateRear = %d\n", forkStateFront, forkStateRear);
        printf("MergeStateFront = %d\tMergeStateRear = %d\tMergeTimeoutFront = %d\tMergeTimeoutRear = %d\n", mergeStateFront, mergeStateRear, mergeTimeoutFront, mergeTimeoutRear);
        printf("MergeFlag = %d\n", mergeFlag);
        printf("Angle = %3.2f, d = %3.2f\n", angle_status / M_PI * 180.0f, d_status);
        printf("X: %d\tY: %d\tAngle: %d\tTagNum: %d\n", xpos, ypos, qrAngle, tagNum);
        printf("MagSen1: of1 = %d\tof2 = %d\t width = %d\n", magSen1->getTrackOffset(0), magSen1->getTrackOffset(1), magSen1->getTrackWidth());
        printf("MagSen2: of1 = %d\tof2 = %d\t width = %d\n", magSen2->getTrackOffset(0), magSen2->getTrackOffset(1), magSen2->getTrackWidth());
        usleep(100000);
    }
}