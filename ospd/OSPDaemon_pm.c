/*
 * (C) Copyright 2015 HY Research LLC for Audience, Inc.
 *
 * Apache Licensed.
 */

#include <stdio.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <limits.h>

#include "OSPDaemon.h"
#include "OSPDaemon_driver.h"
#include "OSPDaemon_pm.h"

#define DBGOUT(x...)	DBG(DEBUG_PM, "PM: "x);

int OSPDaemon_power_init(struct OSPDaemon_SensorDescription *sd)
{
	int fd;
	int ret;
	int i;
	char fname[PATH_MAX+1];

	if (sd->powerPath == NULL) {
		DBGOUT("No power path defined. Disable PM.\n");
		disablepm = 1;
		return -1;
	}

	/* Clear out the directory */
	for (i = 0; i < sd->output_count; i++) {
		snprintf(fname, PATH_MAX, "%s/%s",
			sd->powerPath, sd->output[i].name);
		unlink(fname);
	}

	fd = inotify_init();
	if (fd < 0) {
		DBGOUT("inotify_init failed\n");
		return fd;
	}
	ret = inotify_add_watch(fd, sd->powerPath, IN_CREATE | IN_DELETE);
	if (ret < 0) {
		mkdir(sd->powerPath, 0777);
		ret = inotify_add_watch(fd, sd->powerPath,
				IN_CREATE | IN_DELETE);
		if (ret < 0) {
			DBGOUT("inotify_add_watch failed\n");
			close(fd);
			fd = -1;
		}
	}
	DBGOUT("Successfully added inotify watch for PM using %s\n", sd->powerPath);

	return fd;
}

void pm_power_control(struct OSPDaemon_output *out , int enable)
{
	if (enable) {
		out->enable = 1;
		OSPDaemon_driver_enable_out(out);
		if (!out->noprocess) {
			OSP_SubscribeSensorResult(&out->ResultDesc,
					&out->handle);
			DBGOUT("%s Subscribe\n ", __func__);
		} else
			OSPDaemon_driver_enable_in(out->source);
		DBGOUT("%s Enabling %s\n", __func__, out->name);

	} else {
		out->enable = 0;
		OSPDaemon_driver_disable_out(out);
		if (!out->noprocess) {
			OSP_UnsubscribeSensorResult(out->handle);
		DBGOUT("%s UnSubscribe \n", __func__);
		} else
			OSPDaemon_driver_disable_in(out->source);
		DBGOUT("%s Disabling %s\n", __func__, out->name);
	}
}

static void pm_process_output(struct OSPDaemon_output *out, int mask)
{
	if (mask & IN_CREATE) {
	/* Enable sensor */
		pm_power_control(out, 1);
	} else if (mask & IN_DELETE) {
	/* Disable sensor */
		pm_power_control(out, 0);
    }
}

int OSPDaemon_power_process(struct OSPDaemon_SensorDescription *sd, int fd)
{
	int i;
	int ret;
	char buf[2048], *ptr;
	struct inotify_event *inptr;

	ret = read(fd, buf, 2048);
	if (ret <= 0)
		return 0;
	ptr = buf;
	do {
		inptr = (struct inotify_event *)ptr;
		for (i = 0; i < sd->output_count; i++) {
			if (strcmp(sd->output[i].name, inptr->name) == 0) {
				printf("Power event found 0x%x\n", inptr->mask);
				pm_process_output(&sd->output[i], inptr->mask);
				break;
			}
		}
		ptr += (sizeof(struct inotify_event)+inptr->len);
	} while(ptr < (buf+ret));

	return 0;
}
