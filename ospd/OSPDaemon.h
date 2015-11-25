#ifndef _OSPDAEMON_H_
#define _OSPDAEMON_H_	1
#include "osp-api.h"
#include "OSPDaemon_queue.h"
#include "osp-api.h"
#if defined(ANDROID_DEBUG)
#include <cutils/log.h>
#include "local_log_def.h"
#endif

/* Per sensor cal data size. Data larger then this is ignored */
#define MAX_CALSIZE	2048

struct OSPDaemon_SensorDetail;

struct OSPDaemon_output {
	ResultDescriptor_t ResultDesc;
	ResultHandle_t handle;
	struct OSPDaemon_SensorDetail *source;
	int type;
	char *name;
	int fd;
	int driver;
	int noprocess;
	int enable;
	int usage;
	int format;
	struct queue q;
	int option;
};

enum {
	OUT_FORMAT_INTEGER,
	OUT_FORMAT_FLOAT,
	OUT_FORMAT_Q24,
	OUT_FORMAT_Q16,
	OUT_FORMAT_Q15,
	OUT_FORMAT_Q12,
};

enum {
	IN_FORMAT_INTEGER,
	IN_FORMAT_FLOAT,
};

enum {
	DRIVER_INVALID,
	DRIVER_INPUT,
	DRIVER_IIO,
	DRIVER_IIOEVENT,
	DRIVER_FILECSV,
	DRIVER_UNKNOWN
};

enum {
	DRIVER_TYPE_INPUT,
	DRIVER_TYPE_OUTPUT,
};

/* Output embedded time stamps */
#define INPUT_OPTION_EMBEDTS	(1<<0)
/* Send all 4 elements of a quaternion */
#define INPUT_OPTION_QUAT4	(1<<1)
/* Dither output to defeat input event dup rejection */
#define INPUT_OPTION_DITHER	(1<<2)

static inline void conv2OSP(ASensorType_t sensor,
	OSP_InputSensorData_t *d, unsigned long long ts,
	int val[])
{
	switch (sensor) {
	case SENSOR_ACCELEROMETER:
	case SENSOR_GYROSCOPE:
	case SENSOR_GRAVITY:
	case SENSOR_LINEAR_ACCELERATION:
		d->q24data.X = val[0];
		d->q24data.Y = val[1];
		d->q24data.Z = val[2];
		d->q24data.TimeStamp = ts;
		break;

	case SENSOR_GEOMAGNETIC_FIELD:
		d->q12data.X = val[0];
		d->q12data.Y = val[1];
		d->q12data.Z = val[2];
		d->q12data.TimeStamp = ts;
		break;

	case SENSOR_ORIENTATION:
		d->orientdata.Yaw = val[0];
		d->orientdata.Pitch = val[1];
		d->orientdata.Roll = val[2];
		d->orientdata.TimeStamp = ts;
		break;
	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		d->rotvec.X = val[0];
		d->rotvec.Y = val[1];
		d->rotvec.Z = val[2];
		d->rotvec.W = val[3];
		d->rotvec.TimeStamp = ts;
		break;
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		d->uncal_q24data.X = val[0];
		d->uncal_q24data.Y = val[1];
		d->uncal_q24data.Z = val[2];
		d->uncal_q24data.TimeStamp = ts;
		break;
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		d->uncal_q12data.X = val[0];
		d->uncal_q12data.Y = val[1];
		d->uncal_q12data.Z = val[2];
		d->uncal_q12data.TimeStamp = ts;
		break;
	case SENSOR_SIGNIFICANT_MOTION:
	case SENSOR_STEP_DETECTOR:
		d->booldata.data = val[0];
		d->booldata.TimeStamp = ts;
		break;
	case SENSOR_STEP_COUNTER:
		d->stepcount.StepCount = val[0];
		d->stepcount.TimeStamp = ts;
		break;
	default:
		break;
	}
}
static inline int extractOSP(ASensorType_t sensor,
	OSP_InputSensorData_t *d, unsigned long long *ts,
	int val[])
{
	switch (sensor) {
	case SENSOR_ACCELEROMETER:
	case SENSOR_GYROSCOPE:
	case SENSOR_GRAVITY:
	case SENSOR_LINEAR_ACCELERATION:
		val[0] = d->q24data.X;
		val[1] = d->q24data.Y;
		val[2] = d->q24data.Z;
		*ts = d->q24data.TimeStamp;
		return 3;

	case SENSOR_GEOMAGNETIC_FIELD:
		val[0] = d->q12data.X;
		val[1] = d->q12data.Y;
		val[2] = d->q12data.Z;
		*ts = d->q12data.TimeStamp;
		return 3;

	case SENSOR_ORIENTATION:
		val[0] = d->orientdata.Yaw;
		val[1] = d->orientdata.Pitch;
		val[2] = d->orientdata.Roll;
		*ts = d->orientdata.TimeStamp;
		return 3;
	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		val[0] = d->rotvec.X;
		val[1] = d->rotvec.Y;
		val[2] = d->rotvec.Z;
		val[3] = d->rotvec.W;
		*ts = d->rotvec.TimeStamp;
		return 4;
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		val[0] = d->uncal_q24data.X;
		val[1] = d->uncal_q24data.Y;
		val[2] = d->uncal_q24data.Z;
		*ts = d->uncal_q24data.TimeStamp;
		return 3;
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		val[0] = d->uncal_q12data.X;
		val[1] = d->uncal_q12data.Y;
		val[2] = d->uncal_q12data.Z;
		*ts = d->uncal_q12data.TimeStamp;
		return 3;
	case SENSOR_SIGNIFICANT_MOTION:
	case SENSOR_STEP_DETECTOR:
		val[0] = d->booldata.data;
		*ts = d->booldata.TimeStamp;
		return 1;
	case SENSOR_STEP_COUNTER:
		val[0] = d->stepcount.StepCount;
		*ts = d->stepcount.TimeStamp;
		return 1;
	default:
		*ts = 0;
		return 0;
	}
	return 0;
}


struct OSPDaemon_SensorDetail {
	SensorDescriptor_t sensor;
	InputSensorHandle_t handle;
	char caldata[MAX_CALSIZE];
	char const *name;
	int fd;
	int driver;
	void *private;
	struct OSPDaemon_output *output;
	struct queue q;
	int pending;
	int noprocess;
	int format;
	OSP_InputSensorData_t pdata;
	int option;
};

struct OSPDaemon_SensorDescription {
	struct OSPDaemon_SensorDetail *sensor;
	int sensor_count;
	struct OSPDaemon_output *output;
	int output_count;	
	char *CalFile;
	char *powerPath;
	int powerfd;
};


/* Cal file format is an array of struct OSPDaemon_CalRecord */

struct OSPDaemon_CalRecord {
	ASensorType_t sensor;
	uint32_t len;
	unsigned char data[1];	/* Array length is determined by len */
};

#if !defined(ANDROID_DEBUG)
#define DBG(x, ...) if (debug_level & x) {printf(__VA_ARGS__);}
#else
#define DBG(x, ...) if (debug_level & x) {ALOGE(__VA_ARGS__);}
#endif

#define DEBUG_INIT	(1<<0)
#define DEBUG_CONF	(1<<1)
#define DEBUG_INDRIVER	(1<<2)
#define DEBUG_OUTDRIVER	(1<<3)
#define DEBUG_LOOP	(1<<4)
#define DEBUG_DATA	(1<<5)
#define DEBUG_PM	(1<<6)
#define DEBUG_DRIVER	(1<<7)

extern unsigned int debug_level;
extern unsigned int mainstatus;
extern unsigned int disablepm;

struct OSPDaemon_SensorDescription * OSPDaemon_config(char *);

void OSPDaemon_iio_init(void);
void OSPDaemon_input_init(void);
void OSPDaemon_inputreader_init(void);
void OSPDaemon_filecsv_init(void);

#endif
