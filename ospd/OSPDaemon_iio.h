#ifndef _OSPDAEMON_IIO_H_
#define _OSPDAEMON_IIO_H_
#include "OSPDaemon.h"

#define MAX_AXIS 	5
#define MAX_IIO		99
/* TODO: Figure this out from config file */
#define MAX_IIO_SENSOR 20

#define IIO_DEVICE_DIR	"/sys/bus/iio/devices"

enum {
	axis_x,
	axis_y,
	axis_z,
	axis_r,
	axis_timestamp,
	axis_invalid
};

struct DataDesc {
	int sign;
	int size;
	int store;
	int shift;
};

struct IIO_SensorAxis {
	struct DataDesc dd;
	int index;
	int offset;
};

struct IIO_Sensor {
	struct IIO_SensorAxis IIOAxis[MAX_AXIS];
	OSP_InputSensorData_t data;
	int index2axis[MAX_AXIS];
	int iionum;
	int rec_sz;
	char const *name;
	int type;
};

#endif
