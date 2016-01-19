/*
 * (C) Copyright 2015 HY Research LLC for Audience, Inc.
 *
 * Apache Licensed.
 */
/*
 * Config file parser.
 *
 * Does not explicitly use malloc.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>

#include "OSPDaemon.h"
#include "osp-api.h"

#define MAX_TAG		50
#define NAME_LEN	80
#define DBGOUT(x...) DBG(DEBUG_CONF, "CONFIG: "x)
/* Pre-allocate storate to avoid malloc.*/
static struct OSPDaemon_SensorDetail Sensor[MAX_TAG];
static char SensorName[MAX_TAG][NAME_LEN];
static int sensor_count = 0;
static struct OSPDaemon_output Output[MAX_TAG];
static int output_count = 0;
static char CalFile[PATH_MAX];
static char powerPath[PATH_MAX];
static char version[MAX_VERSION_LEN];
char output_name[MAX_TAG][NAME_LEN];
static struct OSPDaemon_SensorDescription sd;
/*
 * File format:
 * Lines beginning with '#' are comments.
 *
 * For source sensors (sensors to read from)
 * sensor0.type =
 * sensor0.driver =
 * sensor0.
 * ....
 * For output sensor (sensors to output)
 * output0.type =
 * output0.name =
 * output0.driver =
 */

#define SEN_MAP(x)	[x] = #x

static int proc_sensor_type(int linetype, int tag, char *val, int *taglist);
static int proc_sensor_noise(int linetype, int tag, char *val, int *taglist);
static int proc_sensor_convert(int linetype, int tag, char *val, int *taglist);
static int proc_sensor_swap(int linetype, int tag, char *val, int *taglist);
static int proc_name(int linetype, int tag, char *val, int *taglist);
static int proc_driver(int linetype, int tag, char *val, int *taglist);
static int proc_sensor_copy(int linetype, int tag, char *val, int *taglist);
static int proc_output_noprocess(int linetype, int tag, char *val, int *taglist);
static int proc_output_format(int linetype, int tag, char *val, int *taglist);
static int proc_sensor_format(int linetype, int tag, char *val, int *taglist);
static int proc_system_calfile(int linetype, int tag, char *val, int *taglist);
static int proc_system_pmdir(int linetype, int tag, char *val, int *taglist);
static int proc_option(int linetype, int tag, char *val, int *taglist);
static int proc_system_version(int linetype, int tag, char *val, int *taglist);

enum {
	LINE_UNKNOWN,
	LINE_SENSOR,
	LINE_OUTPUT,
	LINE_SYSTEM,
};
static const char *LineName[] = {
	[LINE_UNKNOWN] = "Unkown",
	[LINE_SENSOR] = "Input",
	[LINE_OUTPUT] = "Output",
	[LINE_SYSTEM] = "System",
};

static const char * SensorTypeMap[] = {
	SEN_MAP(SENSOR_ACCELEROMETER),
	SEN_MAP(SENSOR_GYROSCOPE),
	SEN_MAP(SENSOR_MAGNETIC_FIELD),
	SEN_MAP(SENSOR_ORIENTATION),
	SEN_MAP(SENSOR_LIGHT),
	SEN_MAP(SENSOR_PRESSURE),
	SEN_MAP(SENSOR_TEMPERATURE),
	SEN_MAP(SENSOR_PROXIMITY),
	SEN_MAP(SENSOR_GRAVITY),
	SEN_MAP(SENSOR_LINEAR_ACCELERATION),
	SEN_MAP(SENSOR_ROTATION_VECTOR),
	SEN_MAP(SENSOR_RELATIVE_HUMIDITY),
	SEN_MAP(SENSOR_AMBIENT_TEMPERATURE),
	SEN_MAP(SENSOR_MAGNETIC_FIELD_UNCALIBRATED),
	SEN_MAP(SENSOR_GAME_ROTATION_VECTOR),
	SEN_MAP(SENSOR_GYROSCOPE_UNCALIBRATED),
	SEN_MAP(SENSOR_SIGNIFICANT_MOTION),
	SEN_MAP(SENSOR_STEP_DETECTOR),
	SEN_MAP(SENSOR_STEP_COUNTER),
	SEN_MAP(SENSOR_GEOMAGNETIC_ROTATION_VECTOR),
	[NUM_ANDROID_SENSOR_TYPE] = NULL
};
static const char * SensorTypeMapP[] = {
	SEN_MAP(PSENSOR_ACCELEROMETER_RAW),
	SEN_MAP(PSENSOR_MAGNETIC_FIELD_RAW),
	SEN_MAP(PSENSOR_GYROSCOPE_RAW),
	SEN_MAP(PSENSOR_ACCELEROMETER_UNCALIBRATED),
	[NUM_PRIVATE_SENSOR_TYPE] = NULL
};

static const char *DriverTypeMap[] = {
	[DRIVER_INVALID] = "invalid",
	[DRIVER_INPUT] = "input",
	[DRIVER_IIO] = "iio",
	[DRIVER_IIOEVENT] = "iioevent",
	[DRIVER_FILECSV] = "file_csv",
	[DRIVER_QUEUE] = "queue"
};

static const struct _keymap {
	char *key;
	int (*proc)(int linetype, int tag, char *val, int *taglist);
} KeyMap[] = {
	{"type", proc_sensor_type},
	{"swap", proc_sensor_swap},
	{"convert", proc_sensor_convert},
	{"noise", proc_sensor_noise},
	{"name", proc_name},
	{"driver", proc_driver},
	{"copy", proc_sensor_copy},
	{"noprocess", proc_output_noprocess},
	{"calfile", proc_system_calfile},
	{"pmdir", proc_system_pmdir},
	{"format", proc_output_format},
	{"iformat", proc_sensor_format},
	{"option", proc_option},
	{"version", proc_system_version},
};

/* Parse a string containing 3 comma seperated values */
static int parse_3_double(char *val, double out[3])
{
	char *ptr;

	out[0] = strtod(val, &ptr);
	if (ptr && *ptr == ',') {
		out[1] = strtod(ptr+1, &ptr);
		if (ptr && *ptr == ',')
			out[2] = strtod(ptr+1, NULL);
	} else {
		out[1] = out[0];
		out[2] = out[0];
	}
	return 0;
}

static int allocate_sensor(void)
{
	int ret;

	ret = sensor_count;
	sensor_count++;
	/* Initialize descriptor to sane values */
	Sensor[ret].sensor.DataWidthMask = 0xffffffff;
	Sensor[ret].sensor.AxisMapping[0] = AXIS_MAP_POSITIVE_X;
	Sensor[ret].sensor.AxisMapping[1] = AXIS_MAP_POSITIVE_Y;
	Sensor[ret].sensor.AxisMapping[2] = AXIS_MAP_POSITIVE_Z;
	Sensor[ret].sensor.MaxValue = 0xffffffff;
	Sensor[ret].sensor.MinValue = 0xffffffff;
	Sensor[ret].sensor.ConversionScale[0] = TOFIX_PRECISE(1.0f);
	Sensor[ret].sensor.ConversionScale[1] = TOFIX_PRECISE(1.0f);
	Sensor[ret].sensor.ConversionScale[2] = TOFIX_PRECISE(1.0f);

	Sensor[ret].sensor.NominalSamplePeriodInSeconds = TOFIX_PRECISE(0.02f);
	Sensor[ret].sensor.factoryskr[0][0] = TOFIX_PRECISE(1.0f);
	Sensor[ret].sensor.factoryskr[0][1] = 0;
	Sensor[ret].sensor.factoryskr[0][2] = 0;

 	Sensor[ret].sensor.factoryskr[1][0] = 0;
 	Sensor[ret].sensor.factoryskr[1][1] = TOFIX_PRECISE(1.0f);
 	Sensor[ret].sensor.factoryskr[1][2] = 0;

	Sensor[ret].sensor.factoryskr[2][0]= 0;
	Sensor[ret].sensor.factoryskr[2][1]= 0;
	Sensor[ret].sensor.factoryskr[2][2]= TOFIX_PRECISE(1.0f);

	return ret;
}

static int allocate_output(void)
{
	int ret;

	ret = output_count;
	output_count++;
	return ret;
}

static int proc_system_calfile(int linetype, int tag, char *val, int *taglist)
{
	if (linetype != LINE_SYSTEM) return -1;

	strncpy(CalFile, val, PATH_MAX-1);
	CalFile[PATH_MAX-1] = '\0';
	DBGOUT("Cal file: %s\n", val);

	return 0;
}

static int proc_system_version(int linetype, int tag, char *val, int *taglist)
{
	if (linetype != LINE_SYSTEM) return -1;
	memset(version, 0, sizeof(version));
	strncpy(version, val, MAX_VERSION_LEN);
	DBGOUT("version: %s\n", val);
	return 0;
}
static int proc_system_pmdir(int linetype, int tag, char *val, int *taglist)
{
	if (linetype != LINE_SYSTEM) return -1;

	strncpy(powerPath, val, PATH_MAX-1);
	powerPath[PATH_MAX-1] = '\0';
	DBGOUT("power path: %s\n", val);

	return 0;
}

/*
 * sensor0.copy=output0
 */
static int proc_sensor_copy(int linetype, int tag, char *val, int *taglist)
{
	int out, ret = -1;

	if (linetype != LINE_SENSOR)  return -1;
	DBGOUT("Line %s", LineName[linetype]);

	if (taglist[tag] < 0) {
		taglist[tag] = allocate_sensor();
	}

	if (strncmp("output", val, 6) == 0) {
		out = atoi(val+6);
		DBGOUT("Linking to out tag %i\n", out);
		if (out >= 0 && out < MAX_TAG) {
			Sensor[taglist[tag]].output = &Output[out];
			Output[out].source = &Sensor[taglist[tag]];
			ret  = 0;
		}
	}

	return ret;
}
static int proc_output_noprocess(int linetype, int tag, char *val, int *taglist)
{
	if (taglist[tag] < 0) {
		if (linetype == LINE_SENSOR)
			taglist[tag] = allocate_sensor();
		else if (linetype == LINE_OUTPUT)
			taglist[tag] = allocate_output();
	}
	DBGOUT("Line %s", LineName[linetype]);
	if (linetype == LINE_SENSOR)
		Sensor[taglist[tag]].noprocess = 1;
	else if (linetype == LINE_OUTPUT)
		Output[taglist[tag]].noprocess = 1;

	return 0;
}
static int proc_option(int linetype, int tag, char *val, int *taglist)
{
	unsigned int opt;

	if (taglist[tag] < 0) {
		if (linetype == LINE_SENSOR)
			taglist[tag] = allocate_sensor();
		else if (linetype == LINE_OUTPUT)
			taglist[tag] = allocate_output();
	}
	DBGOUT("Line %s", LineName[linetype]);

	opt = strtol(val, NULL, 0);
	if (linetype == LINE_OUTPUT) {
		Output[taglist[tag]].option = opt;
	} else if (linetype == LINE_SENSOR) {
		Sensor[taglist[tag]].option = opt;
	} else {
		return -1;
	}

        return 0; // Ravi
}

static int proc_output_format(int linetype, int tag, char *val, int *taglist)
{
	int shift;

	if (linetype != LINE_OUTPUT) return 0;

	if (taglist[tag] < 0) {
		taglist[tag] = allocate_output();
	}
	DBGOUT("Line %s scale %s\n", LineName[linetype], val);

	/* scaleNN -> shift by NN */
	if (strncasecmp("shift", val, 5) == 0) {
		shift = atoi(val+5);
		DBGOUT("Line %s scale %s shift %i\n", LineName[linetype], val, shift);
		switch(shift) {
		case 24:
			Output[taglist[tag]].format = OUT_FORMAT_Q24;
			break;
		case 16:
			Output[taglist[tag]].format = OUT_FORMAT_Q16;
			break;
		case 12:
			Output[taglist[tag]].format = OUT_FORMAT_Q12;
			break;
		case 15:
			Output[taglist[tag]].format = OUT_FORMAT_Q15;
			break;
		default:
			break;
		}

	} else if (strcasecmp("integer", val) == 0) {
		Output[taglist[tag]].format = OUT_FORMAT_INTEGER;

	} else if (strcasecmp("float", val) == 0) {
		Output[taglist[tag]].format = OUT_FORMAT_FLOAT;
	}

	return 0;
}

/* Internal code (algs, etc) assume data is Qnn. If the input is
 * not in that format, this config option converts to that.
 */
static int proc_sensor_format(int linetype, int tag, char *val, int *taglist)
{
	if (linetype != LINE_SYSTEM) return 0;

	if (taglist[tag] < 0) {
		taglist[tag] = allocate_output();
	}
	DBGOUT("Line %s scale %s\n", LineName[linetype], val);

	/* scaleNN -> shift by NN */
	if (strcasecmp("integer", val) == 0) {
		Sensor[taglist[tag]].format = IN_FORMAT_INTEGER;

	} else if (strcasecmp("float", val) == 0) {
		Sensor[taglist[tag]].format = IN_FORMAT_FLOAT;
	}

	return 0;
}
static int proc_driver(int linetype, int tag, char *val, int *taglist)
{
	int i;

	for (i = 0; i < DRIVER_UNKNOWN; i++) {
		DBGOUT("Comparing %s to %s\n", DriverTypeMap[i], val);
		if (strcmp(DriverTypeMap[i], val) == 0) {
			break;
		}
	}
	DBGOUT("Line %s val = %s mapping to %i\n", LineName[linetype], val, i);
	if (taglist[tag] < 0) {
		if (linetype == LINE_SENSOR)
			taglist[tag] = allocate_sensor();
		else if (linetype == LINE_OUTPUT)
			taglist[tag] = allocate_output();
	}

	if (linetype == LINE_OUTPUT) {
		Output[taglist[tag]].driver = i;
	} else {
		Sensor[taglist[tag]].driver = i;
	}
	return 0;
}

static int proc_name(int linetype, int tag, char *val, int *taglist)
{
	if (taglist[tag] < 0) {
		if (linetype == LINE_SENSOR)
			taglist[tag] = allocate_sensor();
		else if (linetype == LINE_OUTPUT)
			taglist[tag] = allocate_output();
	}

	if (linetype == LINE_OUTPUT) {
		strncpy(output_name[taglist[tag]], val, NAME_LEN-1);
		output_name[taglist[tag]][NAME_LEN-1] = '\0';
		Output[taglist[tag]].name = output_name[taglist[tag]];
		DBGOUT("OUTPUT: tag %i - %s\n", tag, val);
	} else if (linetype == LINE_SENSOR) {
		strncpy(SensorName[taglist[tag]], val, NAME_LEN-1);
		SensorName[taglist[tag]][NAME_LEN-1] = '\0';
		Sensor[taglist[tag]].name = SensorName[taglist[tag]];
		DBGOUT("SENSOR: tag %i - %s\n", tag, val);
	} else {
		return -1;
	}

	return 0;
}

/* Mapping to values used by the descriptor */
static const int OSPaxisMap[3] = {
	[0] = AXIS_MAP_POSITIVE_X,
	[1] = AXIS_MAP_POSITIVE_Y,
	[2] = AXIS_MAP_POSITIVE_Z,
};

/* sensor0.swap=0,1,2 */
static int proc_sensor_swap(int linetype, int tag, char *val, int *taglist)
{
	int map[3];
	int ret = -1;
	int idx;

	if (linetype != LINE_SENSOR) return -1;

	if (taglist[tag] < 0)
		taglist[tag] = allocate_sensor();

	idx = 0;

	map[0] = val[idx]-'0';
	if (map[0] < 0 || map[0] > 2) goto error;

	idx++;
	while (val[idx] != '\0' && val[idx] == ' ')
		idx++;
	if (val[idx] == '\0') goto error;

	if (val[idx] != ',') goto error;
	idx++;

	while (val[idx] != '\0' && val[idx] == ' ')
		idx++;
	if (val[idx] == '\0') goto error;

	map[1] = val[idx]-'0';
	idx++;
	if (map[1] < 0 || map[1] > 2) goto error;

	while (val[idx] != '\0' && val[idx] == ' ')
		idx++;
	if (val[idx] == '\0') goto error;

	if (val[3] != ',') ret = -1;
	idx++;

	while (val[idx] != '\0' && val[idx] == ' ')
		idx++;
	if (val[idx] == '\0') goto error;

	map[2] = val[idx]-'0';
	if (map[1] < 0 || map[1] > 2) goto error;

	if (map[0] == map[1] || map[0] == map[2] ||
		map[1] == map[2]) {
		ret = -2;
		goto error;
	}
	ret = 0;

	Sensor[taglist[tag]].sensor.AxisMapping[0] = OSPaxisMap[map[0]];
	Sensor[taglist[tag]].sensor.AxisMapping[1] = OSPaxisMap[map[1]];
	Sensor[taglist[tag]].sensor.AxisMapping[2] = OSPaxisMap[map[2]];
	DBGOUT("MAP: tag %i -  %i %i %i\n", tag, map[0], map[1], map[2]);

error:
	if (ret < 0)
		printf("INVALID MAP\n");
	return ret;
}

/* sensor0.convert=-0.12345,-0.12345,-0.12345  */
/* sensor0.convert=-0.12345  */
static int proc_sensor_convert(int linetype, int tag, char *val, int *taglist)
{
	double convert[3];

	if (linetype != LINE_SENSOR) return -1;
	if (taglist[tag] < 0)
		taglist[tag] = allocate_sensor();

	parse_3_double(val, convert);
	Sensor[taglist[tag]].sensor.ConversionScale[0] = convert[0];
	Sensor[taglist[tag]].sensor.ConversionScale[1] = convert[1];
	Sensor[taglist[tag]].sensor.ConversionScale[2] = convert[2];

	DBGOUT("CONVERT: tag %i %f %f %f\n", tag, convert[0], convert[1], convert[2]);
	return 0;

}
/* sensor0.noise=-0.12345,-0.12345,-0.12345  */
/* sensor0.noise=-0.12345  */
static int proc_sensor_noise(int linetype, int tag, char *val, int *taglist)
{
	double noise[3];

	if (linetype != LINE_SENSOR) return -1;

	if (taglist[tag] < 0)
		taglist[tag] = allocate_sensor();

	parse_3_double(val, noise);
	Sensor[taglist[tag]].sensor.Noise[0] = noise[0];
	Sensor[taglist[tag]].sensor.Noise[1] = noise[1];
	Sensor[taglist[tag]].sensor.Noise[2] = noise[2];

	DBGOUT("NOISE: tag %i %f %f %f\n", tag, noise[0], noise[1], noise[2]);

	return 0;
}

/* sensor0.type=SENSOR_ACCELEROMETER */
static int proc_sensor_type(int linetype, int tag, char *val, int *taglist)
{
	int i;
	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (SensorTypeMap[i] == NULL) continue;
		if (strcmp(SensorTypeMap[i], val) == 0) {
			DBGOUT("FOUND SENSOR: %i tag %i : %s\n", linetype, tag, SensorTypeMap[i]);
			if (linetype == LINE_SENSOR) {
				taglist[tag] = sensor_count;
				sensor_count++;
				Sensor[taglist[tag]].sensor.SensorType = i;
				Sensor[taglist[tag]].sensor.SensorName = SensorTypeMap[i];
			} else if (linetype == LINE_OUTPUT) {
				taglist[tag] = output_count;
				output_count++;
				Output[taglist[tag]].type = i;
				Output[taglist[tag]].ResultDesc.SensorType = i;
				Output[taglist[tag]].ResultDesc.OutputRateInSeconds = 0;
				Output[taglist[tag]].ResultDesc.OptionData = &Output[taglist[tag]];
			}
			break;
		}
	}
	if (i != NUM_ANDROID_SENSOR_TYPE) {
		return 0;
	}
	for (i = 0; i < NUM_PRIVATE_SENSOR_TYPE; i++) {
		if (SensorTypeMapP[i] == NULL) continue;
		if (strcmp(SensorTypeMapP[i], val) == 0) {
			DBGOUT("FOUND SENSOR: %i tag %i : %s\n", linetype, tag, SensorTypeMapP[i]);
			if (linetype == LINE_SENSOR) {
				taglist[tag] = sensor_count;
				sensor_count++;
				Sensor[taglist[tag]].sensor.SensorType = M_PSensorToAndroidBase(i);
				Sensor[taglist[tag]].sensor.SensorName = SensorTypeMapP[i];
			} else if (linetype == LINE_OUTPUT) {
				taglist[tag] = output_count;
				output_count++;
				Output[taglist[tag]].type = M_PSensorToAndroidBase(i);
				Output[taglist[tag]].ResultDesc.SensorType = M_PSensorToAndroidBase(i);
				Output[taglist[tag]].ResultDesc.OutputRateInSeconds = 0;
				Output[taglist[tag]].ResultDesc.OptionData = &Output[taglist[tag]];
			}
			break;
		}
	}
	if (i == NUM_PRIVATE_SENSOR_TYPE) {
		fprintf(stderr, "Bad/invalid sensor: %s\n", val);
	}
	return 0;
}

static void remove_trail_whitespace(char *p, int crlf)
{
	char *ptr;
	ptr = p;
	while (*ptr != '\0') {
		if (*ptr == '\t') {
			*ptr = '\0';
			continue;
		}
		if (*ptr == ' ') {
			*ptr = '\0';
			continue;
		}
		if (crlf) {
			if (*ptr == '\r') {
				*ptr = '\0';
				continue;
			}
			if (*ptr == '\n') {
				*ptr = '\0';
				continue;
			}
		}
		ptr++;
	}
}
static char * remove_lead_whitespace(char *p)
{
	char *ptr;
	ptr = p;
	while (*ptr == ' ' || *ptr == '\t') ptr++;

	return ptr;
}

static void parse_line(int lineno, char *l, int *taglist1, int *taglist2)
{
	int i;
	char *eq, *ptr, *key;
	int tag;
	int linetype = LINE_UNKNOWN;

	if (l[0] == '#' ||
		l[0] == '\r' ||
		l[0] == '\n' ||
		l[0] == ' ' ||
		l[0] == '\t' ||
		l[0] == ';') return;	/* Comments */
	eq = strchr(l, '=');
	if (!eq) return;
	if (strncmp(l, "input", 5) == 0)
		linetype = LINE_SENSOR;
	else if (strncmp(l, "output", 6) == 0)
		linetype = LINE_OUTPUT;
	else if (strncmp(l, "system", 6) == 0)
		linetype = LINE_SYSTEM;

	DBGOUT("Line %i: %s\n", lineno, LineName[linetype]);
	if (linetype > 0) {
		tag = strtol(l+6, &ptr, 0);
		if (tag >= MAX_TAG) return;

		if (ptr > eq) return;
		if (*ptr != '.') return;
		key=ptr+1;
		*eq = '\0';

		remove_trail_whitespace(key, 0);
		ptr = remove_lead_whitespace(eq+1);
		remove_trail_whitespace(ptr, 1);

		if (ptr[0] == '\0') return;

		for (i = 0; i < sizeof(KeyMap)/sizeof(struct _keymap); i++) {
			if (strcmp(key, KeyMap[i].key) == 0) {
				if (KeyMap[i].proc == NULL)
					break;
				KeyMap[i].proc(linetype, tag, ptr, linetype == LINE_SENSOR?taglist1:(linetype == LINE_OUTPUT?taglist2:NULL));
			}
		}
	}
}

/* A nonzero return will abort the daemon */
struct OSPDaemon_SensorDescription *OSPDaemon_config(char *confname)
{
	FILE *f;
	char line[2048], *r;
	int i;
	int tagmap1[MAX_TAG];
	int tagmap2[MAX_TAG];
	int lineno = 1;

	f = fopen(confname, "r");
	if (!f) return NULL;
	for (i = 0; i < MAX_TAG; i++) {
		tagmap1[i] = -1;
		tagmap2[i] = -1;
		memset(&Sensor[i], 0, sizeof(Sensor[i]));
		Sensor[i].fd = -1;
		memset(&Output[i], 0, sizeof(Sensor[i]));
		Output[i].fd = -1;
	}
	sd.powerPath = NULL;
	powerPath[0] = '\0';
	DBGOUT("%s:%i\n", __func__, __LINE__);
	do {
		r = fgets(line, 2047, f);
		if (r) {
			line[2047] = '\0';
			parse_line(lineno, line, tagmap1, tagmap2);
		}
		lineno++;
	} while (r != NULL);
	DBGOUT("Processed %i lines\n", lineno);
	fclose(f);
	sd.sensor = Sensor;
	sd.sensor_count = sensor_count;
	sd.output = Output;
	sd.output_count = output_count;
	sd.CalFile = CalFile;
	sd.version = version;
	if (powerPath[0] != '\0')
		sd.powerPath = powerPath;
	return &sd;
}
