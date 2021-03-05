#ifndef JOYSTICK_H
#define JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>

/**
 * Current state of an axis.
 */
struct axis_state
{
    short x, y;
};

int read_event(int fd, struct js_event *event);
size_t get_axis_count(int fd);
size_t get_button_count(int fd);
size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);

#ifdef __cplusplus
}
#endif

#endif