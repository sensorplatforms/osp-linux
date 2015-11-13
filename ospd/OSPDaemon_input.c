/*
 * (C) Copyright 2015 HY Research LLC for Audience.
 * Licensed under Apache Public License
 */

/*
 * Driver for input events OUTPUT.
 */


#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/uinput.h>
#include <string.h>
#include <unistd.h>

#include "OSPDaemon.h"
#include "OSPDaemon_queue.h"
#include "OSPDaemon_driver.h"

#define DBGOUT(x...) DBG(DEBUG_INDRIVER, "INPUT: "x)

static int OSPDaemon_input_create(struct OSPDaemon_output *out)
{
	struct uinput_user_dev uindev;
	int fd;
	int ret;

	DBGOUT("%s:%i\n", __func__, __LINE__);

	fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if (fd < 0) {
		ret = fd;
		DBGOUT("Failed to open with error %s", strerror(errno));
		goto error;
	}
	memset(&uindev, 0, sizeof(uindev));

	strncpy(uindev.name, out->name, UINPUT_MAX_NAME_SIZE);
	uindev.id.bustype = BUS_HOST;
	uindev.id.vendor  = 0;
	uindev.id.product = 0;
	uindev.id.version = 1;
	uindev.absmin[ABS_X] = 0;
	uindev.absmax[ABS_X] = 1023;
	uindev.absmin[ABS_Y] = 0;
	uindev.absmax[ABS_Y] = 1023;
	uindev.absmin[ABS_Z] = 0;
	uindev.absmax[ABS_Z] = 1023;
	uindev.absmin[ABS_WHEEL] = 0;
	uindev.absmax[ABS_WHEEL] = 1023;

	ret = write(fd, &uindev, sizeof(uindev));
	if (ret < 0) {
		perror("uindev write");
		goto error;
	}

	ret = ioctl(fd, UI_SET_EVBIT, EV_ABS);
	if (ret < 0) {
		perror("ioctl EV_ABS EVBIT");
		goto error;
	}
	ret = ioctl(fd, UI_SET_EVBIT, EV_SYN);
	if (ret < 0) {
		perror("ioctl EV_SYN EVBIT");
		goto error;
	}

	ret = ioctl(fd, UI_SET_ABSBIT, ABS_X);
	if (ret < 0) {
		perror("ioctl ABS_X ABSBIT");
		goto error;
	}

	ret = ioctl(fd, UI_SET_ABSBIT, ABS_Y);
	if (ret < 0) {
		perror("ioctl ABS_Y ABSBIT");
		goto error;
	}

	ret = ioctl(fd, UI_SET_ABSBIT, ABS_Z);
	if (ret < 0) {
		perror("ioctl ABS_Z ABSBIT");
		goto error;
	}
	if (out->option & INPUT_OPTION_EMBEDTS) {
		ret = ioctl(fd, UI_SET_ABSBIT, ABS_THROTTLE);
		if (ret < 0) {
			perror("ioctl ABS_THROTTLE ABSBIT");
			goto error;
		}

		ret = ioctl(fd, UI_SET_ABSBIT, ABS_RUDDER);
		if (ret < 0) {
			perror("ioctl ABS_RUDDER ABSBIT");
			goto error;
		}
	}
	if (out->option & INPUT_OPTION_QUAT4) {
		ret = ioctl(fd, UI_SET_ABSBIT, ABS_WHEEL);
		if (ret < 0) {
			perror("ioctl ABS_WHEEL ABSBIT");
			goto error;
		}
	}
	
	ret = ioctl(fd, UI_DEV_CREATE);
	if (ret < 0) {
		perror("ioctl CREATE");
		goto error;
	}
	out->fd = fd;

error:
	if (fd < 0) {
		fprintf(stderr, "Failed on %s\n", out->name);
	}
	return ret;
}

static int OSPDaemon_input_setup(struct OSPDaemon_output *out, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		if (out[i].driver == DRIVER_INPUT)
			OSPDaemon_input_create(&out[i]);
	}
	
	return 0;
}

static int OSPDaemon_input_send(struct OSPDaemon_output *out)
{
	OSP_InputSensorData_t od;
	struct input_event ev;
	int ret;
	static int noise=0;
	int val[5], vallen;
	unsigned long long ts;

	while ((ret = OSPDaemon_queue_get(&out->q, &od)) == 0) {
		vallen = extractOSP(out->type, &od, &ts, val);
		DBGOUT("Sending (INPUT) %i %i %i\n", val[0], val[1], val[2]);
		if (out->option & INPUT_OPTION_DITHER) {
			noise++;
			noise %= 2;
		}
		ev.type = EV_ABS;
		ev.code = ABS_X;
		ev.value = val[0]+noise;
		ret = write(out->fd, &ev, sizeof(ev));
		if ( ret <= 0) return 1;	

		ev.type = EV_ABS;
		ev.code = ABS_Y;
		ev.value = val[1]+noise;
		ret = write(out->fd, &ev, sizeof(ev));
		if ( ret <= 0) return 1;	

		ev.type = EV_ABS;
		ev.code = ABS_Z;
		ev.value = val[2]+noise;
		ret = write(out->fd, &ev, sizeof(ev));
		if (ret <= 0) return 1;	
		if (vallen > 3 && out->option & INPUT_OPTION_QUAT4) {
			ev.type = EV_ABS;
			ev.code = ABS_WHEEL;	/* ??? */
			ev.value = val[3]+noise;	
			ret = write(out->fd, &ev, sizeof(ev));
			if ( ret <= 0) return 1;	
		}
		if (out->option & INPUT_OPTION_EMBEDTS) {
			/* Upper 32 bit */
			ev.type = EV_ABS;
			ev.code = ABS_RUDDER;	/* ??? */
			ev.value = (ts >> 32);
			ret = write(out->fd, &ev, sizeof(ev));
			if ( ret <= 0) return 1;	
			/* Lower 32 bit */
			ev.type = EV_ABS;
			ev.code = ABS_THROTTLE;
			ev.value = (ts & ((((unsigned long long)1)<<32)-1));
			ret = write(out->fd, &ev, sizeof(ev));
			if ( ret <= 0) return 1;	
		}

		ev.type = EV_SYN;
		ev.code = 0;
		ev.value = 0;
		ret = write(out->fd, &ev, sizeof(ev));
		if ( ret <= 0) return 1;	
	}	
	if (ret == -1) return 0; else return 1;
	return 0;
}

static struct OSPDaemon_driver InputDriver = {
	.drvtype = DRIVER_TYPE_OUTPUT,
	.driver = DRIVER_INPUT,
	.setup_out = OSPDaemon_input_setup,
	.send = OSPDaemon_input_send,
};

void OSPDaemon_input_init(void)
{
	OSPDaemon_driver_register(&InputDriver);
}
