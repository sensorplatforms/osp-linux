/*
 * (C) Copyright 2015 HY Research LLC for Audience, Inc.
 *
 * Apache Licensed.
 */

/* Driver framework */

#include <stdio.h>
#include <errno.h>
#include "OSPDaemon.h"
#include "OSPDaemon_driver.h"


static struct OSPDaemon_driver *inmap[] = {
	[DRIVER_INPUT] = NULL,
	[DRIVER_IIO] = NULL,
	[DRIVER_IIOEVENT] = NULL,
	[DRIVER_FILECSV] = NULL,
}, *outmap[] = {
	[DRIVER_INPUT] = NULL,
	[DRIVER_IIO] = NULL,
	[DRIVER_IIOEVENT] = NULL,
	[DRIVER_FILECSV] = NULL,
};

int OSPDaemon_driver_register(struct OSPDaemon_driver *ops)
{
	if (ops == NULL) return -EINVAL;

	if (ops->drvtype == DRIVER_TYPE_INPUT) {
		switch (ops->driver) {
		case DRIVER_INPUT:
		case DRIVER_IIO:
		case DRIVER_IIOEVENT:
		case DRIVER_FILECSV:
			break;
		default:
			return -EINVAL;
		}
		inmap[ops->driver] = ops;
	} else if (ops->drvtype == DRIVER_TYPE_OUTPUT) {
		switch (ops->driver) {
		case DRIVER_INPUT:
		case DRIVER_FILECSV:
			break;
		default:
			return -EINVAL;
		}
		outmap[ops->driver] = ops;
	} else {
		return -EINVAL;
	}
	return 0;
}

int OSPDaemon_driver_read(struct OSPDaemon_SensorDetail *s)
{
	if (s == NULL || inmap[s->driver] == NULL)
		return -EINVAL;

	return inmap[s->driver]->read(s);
}

int OSPDaemon_driver_send(struct OSPDaemon_output *s)
{
	if (s == NULL || outmap[s->driver] == NULL)
		return -EINVAL;
	DBG(DEBUG_DRIVER, "Sending to driver %i\n", s->driver);

	return outmap[s->driver]->send(s);
}

int OSPDaemon_driver_setup_in(struct OSPDaemon_SensorDetail *s, int count)
{
	if (inmap[DRIVER_IIO])
		inmap[DRIVER_IIO]->setup_in(s, count);
	if (inmap[DRIVER_INPUT])
		inmap[DRIVER_INPUT]->setup_in(s, count);
	if (inmap[DRIVER_FILECSV])
		inmap[DRIVER_FILECSV]->setup_in(s, count);

	return 0;
}

int OSPDaemon_driver_setup_out(struct OSPDaemon_output *s, int count)
{
	DBG(DEBUG_DRIVER, "Calling INPUT outbound setup\n");
	if (outmap[DRIVER_INPUT])
		outmap[DRIVER_INPUT]->setup_out(s, count);
	DBG(DEBUG_DRIVER, "Calling FileCSV outbound setup\n");
	if (outmap[DRIVER_FILECSV])
		outmap[DRIVER_FILECSV]->setup_out(s, count);
	DBG(DEBUG_DRIVER, "Done out setup\n");

	return 0;
}

int OSPDaemon_driver_enable_out(struct OSPDaemon_output *s)
{
	s->enable = 1;
	s->usage = 1;

	return 0;
}

int OSPDaemon_driver_disable_out(struct OSPDaemon_output *s)
{
	s->enable = 0;
	s->usage = 0;

	return 0;
}

int OSPDaemon_driver_enable_in(struct OSPDaemon_SensorDetail *s)
{
	if (!s) return -1;
	if (inmap[s->driver]->enable_in)
		inmap[s->driver]->enable_in(s);
	return 0;
}
int OSPDaemon_driver_disable_in(struct OSPDaemon_SensorDetail *s)
{
	if (!s) return -1;
	if (inmap[s->driver]->disable_in)
		inmap[s->driver]->disable_in(s);
	return 0;
}
