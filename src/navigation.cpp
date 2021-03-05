#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <errno.h>
#include <modbus.h>
#include <wiringSerial.h>

#include "JoyStick.h"
#include "PIDController.h"
#include "MagneticSensor.h"
#include "Car.h"

int main(int argc, char** argv){

    MagneticSensor sensor1("/dev/ttyAMA1");

    while(1){

        system("clear");
        printf("Count = %d\n",sensor1.getTrackCount());
        printf("Width = %d\n",sensor1.getTrackWidth());
        printf("Offset0 = %d\n",sensor1.getTrackOffset(0));
        printf("Offset1 = %d\n",sensor1.getTrackOffset(1));



        usleep(1000);
    }


    return 0;
}