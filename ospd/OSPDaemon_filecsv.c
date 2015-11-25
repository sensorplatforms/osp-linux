/*
 * (C) 2015 HY Research LLC
 * This file is under the Apache Public License.
 */

/*
 * Driver to read and write input from a file:
 * Since a normal file is always "ready", we need to do a few
 * special things like pausing on errors.
 * This requires the rest of the system to be muxed with poll
 * to insure fairness. Otherwise, this code will monopolize the
 * system.
 */
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include "OSPDaemon.h"
#include "OSPDaemon_filecsv.h"
#include "OSPDaemon_queue.h"
#include "OSPDaemon_driver.h"

#define BUF_SZ	512
#define NUM_CSV	10
#define LINE_LEN	80

static int filecsv_count = 0;
static struct FileCSV_Sensor {
	int axis_count;
	char buf[BUF_SZ];
	int remain;
	int start;
} FileCSV[NUM_CSV];

static int OSPDaemon_filecsv_create(const char *name, struct FileCSV_Sensor *fs)
{
	int fd;

	fd = open(name, O_RDONLY);
	if (fd < 0) return -1;
	fs->remain = 0;
	fs->start = 0;

	return fd;
}

static int OSPDaemon_filecsv_setup(struct OSPDaemon_SensorDetail *s, int count)
{
	int i;
	int f_count = 0, f_success = 0;

	for (i = 0; i < count; i++) {
		if (s->driver == DRIVER_FILECSV) {
			f_count++;
			if (filecsv_count >= NUM_CSV) continue;
			s->fd = OSPDaemon_filecsv_create(s->name, &FileCSV[filecsv_count]);
			if (s->fd > 0) {
				f_success++;
				s->private = &FileCSV[filecsv_count];
				switch (s->sensor.SensorType) {
				case SENSOR_ROTATION_VECTOR:
				case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
				case SENSOR_GAME_ROTATION_VECTOR:
					FileCSV[filecsv_count].axis_count = 4;
					break;
				default:
					FileCSV[filecsv_count].axis_count = 3;
				}
				filecsv_count++;
			}
		}
		s++;
	}
	DBG(DEBUG_INDRIVER, "FileCSV: req %i success %i\n", f_count, f_success);
	return 0;
}

static void pause20(void)
{
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 20000;
	select(0, NULL, NULL, NULL, &tv);
}

static int OSPDaemon_filecsv_read(struct OSPDaemon_SensorDetail *s)
{
	char line[LINE_LEN];
	char *next;
	struct FileCSV_Sensor *fs = s->private;
	OSP_InputSensorData_t od;
	int val[5];
	int i, ret;
	int lineloc = 0;
	int gotline = 0;
	int count;

	do {
		if (fs->remain == 0) {
			ret = read(s->fd, fs->buf, BUF_SZ);
			if (ret <= 0) {
				pause20();
				mainstatus = 1;
				return 0;
			}
			fs->remain = ret;
			fs->start = 0;
		}
		count = fs->remain;
		for (i = 0; i < count && gotline == 0; i++, fs->start++, fs->remain--) {
			if (lineloc > LINE_LEN-2) {
				gotline = 1;
				line[LINE_LEN-1] = '\0';
				break;
			}
			switch(fs->buf[fs->start]) {
			case '\0':
			case '\r':
			case '\n':
				gotline = 1;
				line[lineloc] = '\0';
				break;
			default:
				line[lineloc] = fs->buf[fs->start];
				lineloc++;
				break;
			}
		}
	} while(!gotline);

	/* Assume X,Y,Z in integers */
	switch (line[0]) {
	/* Comments */
	case '#':
	case ';':
	case '/':
	case '\t':
	case ' ':
		return 0;
	}

	DBG(DEBUG_INDRIVER, "Data line: %s\n", line);

	val[0] = strtol(line, &next, 10);
	if (next == NULL) return 0;

	while(!isdigit(*next) && *next != '\0' && *next != '-') next++;
	if (*next == '\0') return 0;
	val[1] = strtol(next, &next, 10);

	while(!isdigit(*next) && *next != '\0' && *next != '-') next++;
	if (*next == '\0') return 0;
	val[2] = strtol(next, &next, 10);

	if (fs->axis_count == 4) {
		while(!isdigit(*next) && *next != '\0' && *next != '-') next++;
		if (*next == '\0') return 0;
		val[3] = strtol(next, &next, 10);
	}
	conv2OSP(s->sensor.SensorType, &od, 0, val);
	DBG(DEBUG_INDRIVER, "Data: %i, %i, %i\n", val[0], val[1], val[2]);
	OSPDaemon_queue_put(&s->q, &od);
	pause20();

	return 1;
}

static void OSPDaemon_writecsv_create(struct OSPDaemon_output *out)
{
	int fd;
	DBG(DEBUG_OUTDRIVER, "Init for %s\n", out->name);
	fd = open(out->name, O_WRONLY|O_CREAT|O_APPEND, S_IRWXU|S_IRGRP|S_IROTH);
	if (fd < 0) return;
	out->fd = fd;
}

static int OSPDaemon_writecsv_setup(struct OSPDaemon_output *out, int count)
{
	int i;
	int f_count = 0, f_success = 0;
	for (i = 0; i < count; i++) {
		if (out[i].driver == DRIVER_FILECSV) {
			f_count++;
			OSPDaemon_writecsv_create(&out[i]);
			if (out[i].fd >= 0)
				f_success++;
		}
	}
	DBG(DEBUG_OUTDRIVER, "FileCSV: req %i suc %i\n", f_count, f_success);
	return 0;
}

static int OSPDaemon_writecsv_send(struct OSPDaemon_output *out)
{
	OSP_InputSensorData_t *od;
	char line[LINE_LEN];
	int val[5], vallen;
	unsigned long long ts;
	int c;
	int count = 0;

	while ((od = OSPDaemon_queue_get(&out->q)) != NULL) {
		count++;
		vallen = extractOSP(out->type, od, &ts, val);
		DBG(DEBUG_OUTDRIVER,"Sending (FILE): %i %i %i fd = %i\n",
			val[0], val[1], val[2], out->fd);
		switch(out->ResultDesc.SensorType) {
		case SENSOR_ROTATION_VECTOR:
		case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		case SENSOR_GAME_ROTATION_VECTOR:
			c = snprintf(line, 79, "%i, %i, %i, %i, %lli\n",
				val[0], val[1], val[2], val[3], ts);
			break;
		default:
			c = snprintf(line, 79, "%i, %i, %i, %lli\n",
				val[0], val[1], val[2], ts);
			break;
		}
		if (out->fd >= 0)
			write(out->fd, line, c);
		if (count > 5) break;	/* Limit writes */
	}

	if (od == NULL) return 0; else return 1;
}

static struct OSPDaemon_driver ReadCSV = {
	.driver = DRIVER_FILECSV,
	.drvtype = DRIVER_TYPE_INPUT,
	.setup_in= OSPDaemon_filecsv_setup,
	.read = OSPDaemon_filecsv_read,
}, WriteCSV = {
	.driver = DRIVER_FILECSV,
	.drvtype = DRIVER_TYPE_OUTPUT,
	.setup_out = OSPDaemon_writecsv_setup,
	.send = OSPDaemon_writecsv_send,
};

void OSPDaemon_filecsv_init(void)
{
	OSPDaemon_driver_register(&ReadCSV);
	OSPDaemon_driver_register(&WriteCSV);
}
