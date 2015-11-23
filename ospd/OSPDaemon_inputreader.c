/*
 * (C) Copyright 2015 HY Research LLC for Audience.
 * Licensed under Apache Public License.
 */

/*
 * Driver for reading data from an input event device.
 */

#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

#include "OSPDaemon.h"
#include "OSPDaemon_inputreader.h"
#include "OSPDaemon_driver.h"

#define INPUT_EVENT_DIR	"/dev/input"

struct RawBuf {
    int val[5];
    unsigned long long ts;
} databuf[MAX_SENSOR];

static int OSPDaemon_inputreader_create(char const *name)
{
    int i;
    int fd = -1;
    char dname[PATH_MAX];
    char evname[80];
    int ret;

    for (i = 0; i < 99; i++) {
        snprintf(dname, PATH_MAX, INPUT_EVENT_DIR"/event%i", i);
        fd = open(dname, O_RDONLY);
        if (fd < 0) continue;
        ret = ioctl(fd, EVIOCGNAME(sizeof(evname)-1), &evname);
        if (ret < 0) {
            close(fd);
            continue;
        }
        close(fd);
        if (strncmp(evname, name, 80) == 0) {
            break;
        }
        fd = -1;
    }

    return fd;
}

static int OSPDaemon_inputreader_setup(struct OSPDaemon_SensorDetail *s, int count)
{
    int i;
    int ircount = 0, irsuccess = 0;
    DBG(DEBUG_INDRIVER, "InputReader %s:%i\n", __func__, __LINE__);

    for (i = 0; i < count; i++) {
        if (s->driver == DRIVER_INPUT) {
            s->fd = OSPDaemon_inputreader_create(s->name);
            ircount++;
            if (s->fd > 0) {
                irsuccess++;
                s->lprivate = &databuf[i];
            }
        }
        s++;
    }
    DBG(DEBUG_INDRIVER, "InputReader: %i request, %i success\n",
            ircount, irsuccess);

    return 0;
}

#define NUM_EVENT	10

static int OSPDaemon_inputreader_read(struct OSPDaemon_SensorDetail *s)
{
    int ret;
    struct input_event iev[NUM_EVENT];
    int i;
    OSP_InputSensorData_t od;
    struct RawBuf *d;
    int dirty = 0;

    ret = read(s->fd, &iev, sizeof(struct input_event)*NUM_EVENT);
    if (ret < 0) return -1;
    for  (i = 0; i < NUM_EVENT; i++) {
        if (sizeof(struct input_event)*i > ret)
            break;
        if (iev[i].type == EV_REL || iev[i].type == EV_ABS) {
            d = s->lprivate;
            switch (iev[i].code) {
                case ABS_X:
                    d->val[0] = iev[i].value;
                    break;
                case ABS_Y:
                    d->val[1] = iev[i].value;
                    break;
                case ABS_Z:
                    d->val[2] = iev[i].value;
                    break;
                    /* Missing Quat R case */
            }
        } else if (iev[i].type == EV_SYN) {
            d = s->lprivate;
            d->ts = iev[i].time.tv_usec;
            conv2OSP(s->sensor.SensorType, &od, d->ts, d->val);
            OSPDaemon_queue_put(&s->q, &od);
            dirty++;
        }
    }
    return dirty;
}
static struct OSPDaemon_driver InputReaderDriver = {
    .drvtype = DRIVER_TYPE_INPUT,
    .driver = DRIVER_INPUT,
    .setup_in = OSPDaemon_inputreader_setup,
    .read = OSPDaemon_inputreader_read,
};

void OSPDaemon_inputreader_init(void)
{
    OSPDaemon_driver_register(&InputReaderDriver);
}
