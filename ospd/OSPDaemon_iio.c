/*
 * (C) Copyright 2015 HY Research LLC for Audience.
 *
 * Licensed under Apache Public License.
 */

/*
 * Driver to read data from an IIO device.
 * For now, it assumes the IIO device done in the form of the
 * OSP IIO drivers.
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
#include <sys/ioctl.h>
#include <errno.h>

#include <linux/iio/events.h>

#include "OSPDaemon.h"
#include "OSPDaemon_iio.h"
#include "OSPDaemon_driver.h"

#define IIO_DEVICE_DIR	"/sys/bus/iio/devices"

#define DBGOUT(x...) DBG(DEBUG_INDRIVER, "IIO: "x)

static struct IIO_Sensor IIOSen[MAX_IIO_SENSOR];

/* sysfs support */
static void sysfs_write_val(const char *path, const int val)
{
	FILE *f;

	f = fopen(path, "w");
	if (f) {
		fprintf(f, "%i", val);
		fclose(f);
	} else {
            DBGOUT("^^^ sysfs_write_val failed(%s) %s %d", strerror(errno), path, val);
        }
}
static void sysfs_write_str(const char *path, const char *str)
{
	FILE *f;

	f = fopen(path, "w");
	if (f) {
		fprintf(f, "%s", str);
		fclose(f);
	}
}
/* -------------- */
/* Parse a IIO type file */
static void parse_type(const char *path, struct DataDesc *dd)
{
	FILE *f;
	char desc[512];
	char *ptr = NULL, *ptr2;

	DBGOUT("Parsing %s\n", path);
	f = fopen(path, "r");
	if (f) {
		ptr = fgets(desc, 512, f);
		fclose(f);
	}
	if (ptr) {
		/* le:s64/64>>0 */
		/* Ignore endianness for now */
		if (desc[3] == 's') {
			dd->sign = 1;
		} else {
			dd->sign = 0;
		}

		dd->size = strtol(desc+4, &ptr, 0);
		dd->store = strtol(ptr+1, &ptr2, 0);
		if (*ptr2 == '>') {
			dd->shift = strtol(ptr2+2, NULL, 0);
		} else {
			dd->shift = 0;
		}
	}
}
/* Expects:
 * in_NAME_AXIS_misc
 */
static const char *tsname = "timestamp";

static int parse_axis(const char *name)
{
	int axis = axis_invalid;
	int i, j;

	if (name[0] == 'i' && name[1] == 'n' && name[2] == '_') {
		for (i = 3, j = 0; name[i] != '\0' && i < 1024; i++, j++) {
			if (name[i] == '_') break;
			if (j < 2048) {
				if (name[i] != tsname[j])
					j = 2048;
			}
		}
		if (j < 2048)
			axis = axis_timestamp;
		else if (name[i] == '_') {
			i++;
			switch (name[i]) {
			case 'x':
				axis = axis_x;
				break;
			case 'y':
				axis = axis_y;
				break;
			case 'z':
				axis = axis_z;
				break;
			case 'r':
				axis = axis_r;
				break;
			}
		}
	}

	return axis;
}

/*
 * Parse the index file.
 */
static int parse_index(const char *fname)
{
	FILE *f;
	char desc[1024];
	char *ptr = NULL;
	int idx = -1;

	f = fopen(fname, "r");
	if (f) {
		ptr = fgets(desc, 1024, f);
		desc[1023] = '\0';
		fclose(f);
	}
	if (ptr) {
		idx = atoi(desc);
	}
	return idx;
}

static OSP_InputSensorData_t * parse_iiodata(struct IIO_Sensor *is, char *buf, int len)
{
	int i, ax;
	union {
		uint8_t b[8];
		uint16_t s[4];
		uint32_t w[2];
		uint64_t ll[1];
		int8_t sb[8];
		int16_t ss[4];
		int32_t sw[2];
		int64_t sll[1];
	} conf;
	int val[6];
	unsigned long long ts = 0;
	double outval;

	for (i = 0; i < MAX_AXIS; i++) {
		ax = is->index2axis[i];
		if (ax < 0) continue;
		conf.w[0] = 0; conf.w[1] = 0;

		switch (is->IIOAxis[ax].dd.size) {
		case 8:
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.sb[0];
			else
				outval = conf.b[0];
			break;
		case 16:
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			conf.b[1] = buf[is->IIOAxis[ax].offset+1];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.ss[0];
			else
				outval = conf.s[0];
			break;
		case 32:
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			conf.b[1] = buf[is->IIOAxis[ax].offset+1];
			conf.b[2] = buf[is->IIOAxis[ax].offset+2];
			conf.b[3] = buf[is->IIOAxis[ax].offset+3];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.sw[0];
			else
				outval = conf.w[0];
			break;
		case 64:	
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			conf.b[1] = buf[is->IIOAxis[ax].offset+1];
			conf.b[2] = buf[is->IIOAxis[ax].offset+2];
			conf.b[3] = buf[is->IIOAxis[ax].offset+3];
			conf.b[4] = buf[is->IIOAxis[ax].offset+4];
			conf.b[5] = buf[is->IIOAxis[ax].offset+5];
			conf.b[6] = buf[is->IIOAxis[ax].offset+6];
			conf.b[7] = buf[is->IIOAxis[ax].offset+7];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.sll[0];
			else
				outval = conf.ll[0];
			break;
		default:
			outval = 0.0;
		}
#if 0
		if (is->IIOAxis[ax].dd.shift)
			outval /= (double)(1<<is->IIOAxis[ax].dd.shift);
#endif
#if 0
		if (ax == axis_timestamp)
			printf("%s = 0x%llx | ", axis_name[ax], conf.ll[0]);
		else
			printf("%s = %08f | ", axis_name[ax], outval);
#else
		/* DANGEROUS CODE. ASSUMES LAYOUT DETAILS */
		switch (ax) {
		case axis_x:
			val[0] = outval;
			break;
		case axis_y:
			val[1] = outval;
			break;
		case axis_z:
			val[2] = outval;
			break;
		case axis_r:
			val[3] = outval;
			break;
		case axis_timestamp:
			ts = conf.ll[0];
			break;
		}
#endif
	}
	conv2OSP(is->type, &is->data, ts, val);
	return &is->data;
}

/*
 * Parse contents of scan elements directory.
 * Enable all elements.
 */
static void setupiio(struct IIO_Sensor *is)
{
	char dname[PATH_MAX];
	char fname[PATH_MAX];
	DIR *d;
	struct dirent *dent;
	int len, ax;

	snprintf(dname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements", is->iionum);
	
	d = opendir(dname);
	if (d == NULL) return;

	while((dent = readdir(d)) != NULL) {
		if (dent->d_name[0] == '.') continue;
		len = strlen(dent->d_name);
		if (len < 6) continue;

		ax = parse_axis(dent->d_name);
		if (ax < 0) continue;

		if (strcmp(dent->d_name + len - 5, "_type") == 0) {
			snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements/%s", is->iionum, dent->d_name);

			parse_type(fname, &is->IIOAxis[ax].dd);
			DBGOUT("%s - %i, %i\n", fname,
				is->IIOAxis[ax].dd.store, ax);
		} else if (strcmp(dent->d_name + len - 3, "_en") == 0) {
			snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements/%s", is->iionum, dent->d_name);
			sysfs_write_val(fname, 1);
		} else if (strcmp(dent->d_name + len - 6, "_index") == 0) {
			snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements/%s", is->iionum, dent->d_name);
			is->IIOAxis[ax].index = parse_index(fname);
			is->index2axis[is->IIOAxis[ax].index] = ax;
		}
	}

	closedir(d);
	snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/trigger/current_trigger", is->iionum);
	DBGOUT("setting trigger name %s to %s\n", is->name, fname);
	sysfs_write_str(fname, is->name);
	snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/buffer/length", is->iionum);
	sysfs_write_val(fname, 100);
	snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/buffer/enable", is->iionum);
	sysfs_write_val(fname, 1);
}
/*
 * Find the IIO device number associate with the named sensor
 */
static int getiionum(const char *sname)
{
	int i, j;
	FILE *f;
	char fname[PATH_MAX];
	char name[1024];

	for (i = 0; i < MAX_IIO; i++) {
		snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/name", i);
		fname[PATH_MAX-1] = '\0';

		DBGOUT("^^^ %s: Now trying to open the file name %s\n", __func__, fname);
		f = fopen(fname, "r");
		if (f == NULL) {
			DBGOUT("^^^ %s: Failed to open the file\n", __func__);
			continue;
		}
		if (fgets(name, 1023, f) != NULL) {
			name[1023] = '\0';
			for (j = 0; j < 1024 && name[j] != '\0'; j++) {
				if (name[j] == '\n' ||
					name[j] == '\r') {
					name[j] = '\0';
					break;
				}
			}
		}
		fclose(f);
		if (strcmp(name, sname) == 0)
			return i;
	}
	DBGOUT("^^^ %s: Couldn't locate the valid iio for %s\n", sname);
	return -1;
}

static void OSPDaemon_iio_SetState(struct IIO_Sensor *s, int state)
{
	char name[PATH_MAX+1];

	if (state != 0) state = 1;
	name[PATH_MAX] = '\0';
	snprintf(name, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/enable",
			s->iionum);

	sysfs_write_val(name, state);
}

static int OSPDaemon_iio_enable(struct OSPDaemon_SensorDetail *s)
{
	struct IIO_Sensor *is;

	is = s->lprivate;
	OSPDaemon_iio_SetState(is, 1);

	return 0;
}

static int OSPDaemon_iio_disable(struct OSPDaemon_SensorDetail *s)
{
	struct IIO_Sensor *is;

	is = s->lprivate;
	OSPDaemon_iio_SetState(is, 0);

	return 0;
}

static int OSPDaemon_iio_create(const char *name, struct IIO_Sensor *s)
{
	int rec_sz;
	int i;
	int fd;
	char dname[PATH_MAX];

	s->iionum = getiionum(name);
	if (s->iionum < 0) return -1;
	DBGOUT("HY-DBG: %s:%i - IIOnum = %i\n", __func__, __LINE__, s->iionum);
	s->name = name;
	setupiio(s);

	rec_sz = 0;
	for (i = 0; i < MAX_AXIS; i++) {
		if (s->IIOAxis[i].index >= 0) {
			if (s->IIOAxis[i].dd.store == 0) continue;

			/* Make sure each group is aligned */
			DBGOUT("1: rec_sz = %i, s->IIOAxis[i].dd.store = %i, %i,\n", rec_sz, s->IIOAxis[i].dd.store, i);
			if (rec_sz % (s->IIOAxis[i].dd.store/8) != 0) {
				rec_sz += ((s->IIOAxis[i].dd.store/8)-(rec_sz % (s->IIOAxis[i].dd.store/8)));
			}
			s->IIOAxis[i].offset = rec_sz;
			rec_sz += s->IIOAxis[i].dd.store/8;
			DBGOUT("2: rec_sz = %i\n", rec_sz);
		}
	}
	s->rec_sz = rec_sz;

	snprintf(dname, PATH_MAX, "/dev/iio:device%i", s->iionum);

	DBGOUT("%s:%d: Name of the iio device is %s", __func__, __LINE__, dname);

	fd = open(dname, O_RDONLY);
        if (-1 == fd) {
            DBGOUT("Failed to open %s with error %s", dname, strerror(errno));
        }

	if (!disablepm) {
		OSPDaemon_iio_SetState(s, 0);
	}
	return fd;
}

static int OSPDaemon_iioevent_create(const char *name, struct IIO_Sensor *s)
{
	int fd, evfd = -1;
	char dname[PATH_MAX];
	int ret;

	s->iionum = getiionum(name);
	if (s->iionum < 0) return -1;
	DBGOUT("HY-DBG: %s:%i - IIOnum = %i\n", __func__, __LINE__, s->iionum);
	s->name = name;

	snprintf(dname, PATH_MAX, "/dev/iio:device%i", s->iionum);

	fd = open(dname, O_RDONLY);
        if (-1 == fd) {
            DBGOUT("IIOEvent failed to open %s", name);
            return -1;
        }
	/* Can the main fd be closed? */	
	ret = ioctl(fd, IIO_GET_EVENT_FD_IOCTL, &evfd);
	return evfd;
}

static int OSPDaemon_iio_setup(struct OSPDaemon_SensorDetail *s, int count)
{
	int i, j;
	int iiocount = 0;
	for (i = 0; i < count; i++) {
		DBGOUT("Driver: %i, count = %i\n", s->driver, count);
		if (s->driver == DRIVER_IIO) {
			DBGOUT("IIO Driver: %i\n", s->driver);
			for (j = 0; j < MAX_AXIS; j++) {
				IIOSen[i].IIOAxis[j].index = -1;
				IIOSen[i].index2axis[j] = -1;
			}		
			DBGOUT("%s:%i Setting up: %s\n", __func__, __LINE__,  s->name);
			s->fd = OSPDaemon_iio_create(s->name, &IIOSen[i]);
			s->lprivate = &IIOSen[i];
			IIOSen[i].type = s->sensor.SensorType;
			if (s->fd < 0) {
				fprintf(stderr, "IIO: Failed on %s\n", s->name);
			}
			iiocount++;
		} else if (s->driver == DRIVER_IIOEVENT) {
			DBGOUT("IIO Event Driver: %i\n", s->driver);
			s->fd = OSPDaemon_iioevent_create(s->name, &IIOSen[i]);
			s->lprivate = &IIOSen[i];
			iiocount++;
		}
		s++;
	}
	DBGOUT("IIO: Found %i iio requests\n", iiocount);

	return 0;
}

static int OSPDaemon_iio_read(struct OSPDaemon_SensorDetail *s)
{
	char readbuf[2048];
	struct IIO_Sensor *is;
	int ret, used;
	OSP_InputSensorData_t *od;

	is = s->lprivate;

	ret = read(s->fd, readbuf, 2048);

	if (ret > 0) {
		used = 0;
		do {
			od = parse_iiodata(is, readbuf+used, ret-used);
			if (od != NULL) {
				DBGOUT("Got data for %s\n", is->name);
				OSPDaemon_queue_put(&s->q, od);
				DBGOUT("Queued data for %s\n", is->name);
			}
			ret -= is->rec_sz;
			used += is->rec_sz;
		} while (ret > 0);
	} else {
		DBGOUT("Read error for fd %i (errno = %i)\n", s->fd, errno);
	}

	DBGOUT("Finish reading data for %s\n", is->name);
	return 0;
}

static int OSPDaemon_iioevent_read(struct OSPDaemon_SensorDetail *s)
{
	int ret;
	struct iio_event_data event;
	struct IIO_Sensor *is;

	is = s->lprivate;

	ret = read(s->fd, &event, sizeof(event));
	if (ret < 0) {
		DBGOUT("Error on event channel read %i\n", errno);
	} else {
		is->data.booldata.TimeStamp = event.timestamp;
		is->data.booldata.data = 1;
		OSPDaemon_queue_put(&s->q, &is->data);
	}

	return 0;
}
static struct OSPDaemon_driver IIODriver = {
	.drvtype = DRIVER_TYPE_INPUT,
	.driver = DRIVER_IIO,
	.setup_in = OSPDaemon_iio_setup,
	.read = OSPDaemon_iio_read,
	.enable_in = OSPDaemon_iio_enable,
	.disable_in = OSPDaemon_iio_disable,
}, IIOEventDriver = {
	.drvtype = DRIVER_TYPE_INPUT,
	.driver = DRIVER_IIOEVENT,
	.setup_in = NULL,
	.read = OSPDaemon_iioevent_read,
};


void OSPDaemon_iio_init(void)
{
	OSPDaemon_driver_register(&IIODriver);
	OSPDaemon_driver_register(&IIOEventDriver);
}
