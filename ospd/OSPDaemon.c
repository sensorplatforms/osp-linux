/*
 * (C) Copyright 2015 HY Research LLC for Audience, Inc.
 *
 * Apache Licensed.
 */

#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <semaphore.h>
#include <errno.h>
#include "OSPDaemon.h"
#include "OSPDaemon_queue.h"
#include "OSPDaemon_pm.h"
#include "OSPDaemon_driver.h"

#include "osp-api.h"

extern sem_t osp_sync;

unsigned int debug_level = 255;//= 1;
unsigned int mainstatus = 0;
unsigned int disablepm = 0;

static struct OSPDaemon_SensorDescription *sd;
static struct pollfd pfd[NUM_ANDROID_SENSOR_TYPE+NUM_PRIVATE_SENSOR_TYPE];
static int nfd = NUM_ANDROID_SENSOR_TYPE+NUM_PRIVATE_SENSOR_TYPE;
static int ignorecal = 0;

static OSP_STATUS_t OSPDaemon_SensorControl_cb(SensorControl_t *cmd);

static const SystemDescriptor_t SystemDesc = {
	.TstampConversionToSeconds = 1,
	.EnterCritical = NULL,
	.ExitCritical = NULL,
	.SensorsControl = OSPDaemon_SensorControl_cb,
};

static OSP_STATUS_t OSPDaemon_SensorControl_cb(SensorControl_t *cmd)
{
	int i;

	if (cmd == NULL) return OSP_STATUS_UNSPECIFIED_ERROR;
	for (i = 0; i <sd->sensor_count; i++) {
		if (sd->sensor[i].handle == cmd->Handle)
			break;
	}

	if (i == sd->sensor_count)
		return OSP_STATUS_SENSOR_INVALID_DESCRIPTOR;

	switch (cmd->Command) {
	case SENSOR_CONTROL_SENSOR_OFF:
		OSPDaemon_driver_disable_in(&sd->sensor[i]);
		return OSP_STATUS_OK;
	case SENSOR_CONTROL_SENSOR_ON:
		OSPDaemon_driver_enable_in(&sd->sensor[i]);
		return OSP_STATUS_OK;
	default:
		return OSP_STATUS_UNSPECIFIED_ERROR;
	}
}

static unsigned char caldata[MAX_CALSIZE];

static void ReadCalData(char *name)
{
	int fd, ret;
	int i;
	struct OSPDaemon_CalRecord rec;

	if (ignorecal) return;

	fd = open(name, O_RDONLY);
	if (fd < 0) return;

	do {
		ret = read(fd, &rec, sizeof(rec));
		if (ret < sizeof(rec)) {
			close(fd);
			return;
		}
		if (rec.len > MAX_CALSIZE)
			continue;
		if (rec.len > 1) {
			ret = read(fd, caldata+1, rec.len-1);
			if (ret != rec.len-1) {
				close(fd);
				return;
			}
			caldata[0] = rec.data[0];
		}
		/* Dispatch cal data - store in pCalibrationData */
		for (i = 0; i < sd->sensor_count; i++) {
			if (sd->sensor[i].sensor.SensorType == rec.sensor)
				break;
		}
		if (i != sd->sensor_count) {
			memcpy(sd->sensor[i].caldata, caldata, rec.len);
			sd->sensor[i].sensor.pCalibrationData = sd->sensor[i].caldata;
		}
	} while(1);
}

static void updateCalFile(char *calfile, ASensorType_t s,
			void *data, uint32_t sz)
{
	char tmpfile[PATH_MAX+1];
	int oldfd, newfd;
	int ret;
	struct OSPDaemon_CalRecord rec;
	unsigned char *cdata;

	snprintf(tmpfile, PATH_MAX, "%s.update", calfile);
	tmpfile[PATH_MAX] = '\0';
	unlink(tmpfile);

	oldfd = open(calfile, O_RDONLY);
	if (oldfd < 0) return;

	newfd = open(tmpfile, O_WRONLY | O_CREAT, 0777);
	if (newfd < 0) {
		close(oldfd);
		return;
	}

	do {
		ret = read(oldfd, &rec, sizeof(rec));
		if (ret == 0) {	/* EOF */
			break;
		}
		if (ret < sizeof(rec)) {
			close(oldfd);
			close(newfd);
			unlink(tmpfile);
			return;
		}
		if (rec.len > MAX_CALSIZE)
			break;
		if (rec.len > 1) {
			ret = read(oldfd, caldata+1, rec.len-1);
			if (ret != rec.len-1) {
				close(oldfd);
				close(newfd);
				unlink(tmpfile);
				return;
			}
			caldata[0] = rec.data[0];
		}
		if (rec.sensor == s) {
			cdata = data;
			rec.len = sz;
			rec.data[0] = cdata[0];
		} else {
			cdata = caldata;
		}
		ret = write(newfd, &rec, sizeof(rec));
		if (ret != sizeof(rec)) {
			close(oldfd);
			close(newfd);
			unlink(tmpfile);
			return;
		}
		if (rec.len > 1) {
			ret = write(newfd, cdata + 1, rec.len - 1);
			if (ret != sizeof(rec)) {
				close(oldfd);
				close(newfd);
				unlink(tmpfile);
				return;
			}
		}
	} while(1);

	close(newfd);
	close(oldfd);
	unlink(calfile);
	rename(tmpfile, calfile);
}

static void OSPDaemon_write_cal_cb(InputSensorHandle_t handle,
	void *data, uint32_t sz, NTTIME ts)
{
	int i;

	for (i = 0; i < sd->sensor_count; i++) {
		if (handle == sd->sensor[i].handle) {
			break;
		}
	}
	if (i == sd->sensor_count) {
		fprintf(stderr, "Write call back with unknown handle\n");
		return;
	}
	if (sz > MAX_CALSIZE) return;
	if (sd->CalFile)
		updateCalFile(sd->CalFile, sd->sensor[i].sensor.SensorType,
				data, sz);
}

/*
 * Generic alg data output call back.
 *    Looks up the queue assocaited with the sensor and
 * places data on the queue.
 */
static void OSPDaemon_callback(ResultHandle_t handle, void *pData)
{
	int i;
	struct OSPDaemon_output *out;

	for (i = 0; i < sd->output_count; i++) {
		if (handle == sd->output[i].handle)
			break;
	}
	if (i == sd->output_count) return;
	out = &sd->output[i];

	OSPDaemon_queue_put(&out->q, pData);
	for (i = 0; i < nfd; i++) {
		if (pfd[i].fd == out->fd) {
			pfd[i].events = POLLOUT;
			break;
		}
	}
}

/* Apply modifications to the sensor data. Used with
 * config file options to modify copied sensor data.
 * Return: 0 - no mod, 1 - mod */
static int modifyOutput(struct OSPDaemon_output *out,
		OSP_InputSensorData_t *orig,
		OSP_InputSensorData_t *mod)
{
	int val[5];
	int count;
	int i;
	unsigned long long ts;
	union {
		int i;
		float f;
	} conv;

	if (out->format == OUT_FORMAT_INTEGER)
		return 0;
	count = extractOSP(out->ResultDesc.SensorType, orig, &ts, val);
	switch (out->format) {
	case OUT_FORMAT_FLOAT:
		for (i = 0; i < count; i++) {
			conv.f = val[i];
			val[i] = conv.i;
		}
		break;
	case OUT_FORMAT_Q24:
		for (i = 0; i < count; i++) {
			val[i] = val[i] << 24;
		}
		break;
	case OUT_FORMAT_Q16:
		for (i = 0; i < count; i++) {
			val[i] = val[i] << 16;
		}
		break;
	case OUT_FORMAT_Q15:
		for (i = 0; i < count; i++) {
			val[i] = val[i] << 15;
		}
		break;
	case OUT_FORMAT_Q12:
		for (i = 0; i < count; i++) {
			val[i] = val[i] << 12;
		}
		break;
	default:
		break;
	}
	conv2OSP(out->ResultDesc.SensorType, mod, ts, val);
	return 1;
}
/*
 * Processes received data:
 *    If the sensor is configured to be tee'd, queue it in the output sensor.
 *    Send data to the alg.
 */
static int OSPDaemon_senddata(struct OSPDaemon_SensorDetail *s)
{
	OSP_InputSensorData_t od;
	OSP_InputSensorData_t mod;
	OSP_STATUS_t stat;
	int ret = 0;
	int err = 0;
	if (!s->noprocess && s->pending) {
		memcpy(&od, &s->pdata, sizeof(s->pdata));
		stat = OSP_SetInputData(s->handle, &od);
		if (stat != OSP_STATUS_OK) return 1;
		s->pending = 0;
	}

	do {
		ret = OSPDaemon_queue_get(&s->q, &od);
		if (0 != ret) {
			ret = 1; // This part uses 1 as error. Keep for now
			break;
		}

		if (s->output) {
			if (disablepm || s->output->enable) {
				if (modifyOutput(s->output, &od, &mod) == 1) {
					OSPDaemon_queue_put(&s->output->q, &mod);
				} else {
					OSPDaemon_queue_put(&s->output->q, &od);
					err = sem_post(&osp_sync);
					if (-1 == err) {
						DBG(DEBUG_INIT, "Sem post failed error - %s", strerror(errno));
						return 1;
					}
				}
			}
		}
		if (!s->noprocess) {
			stat = OSP_SetInputData(s->handle, &od);
			if (stat != OSP_STATUS_OK) {
				s->pending = 1;
				memcpy(&s->pdata, &od, sizeof(s->pdata));
				ret = 1;
				break;
			}
		}
	} while(1);

	return ret;
}

static int OSPDaemon_process_inbound(struct pollfd *pfd, struct pollfd *list, int nfd, int dirty)
{
	int ret;
	int j, k;

	/* Input events */
	for (j = 0; j < sd->sensor_count; j++)
		if (sd->sensor[j].fd == pfd->fd)
			break;
	if (j < sd->sensor_count) {
		OSPDaemon_driver_read(&sd->sensor[j]);
		dirty++;
		ret = OSPDaemon_senddata(&sd->sensor[j]);
		if (sd->sensor[j].output) {
			for (k = 0; k < nfd; k++) {
				if (list[k].fd == sd->sensor[j].output->fd) {
					list[k].events = POLLOUT;
					break;
				}
			}
		}
		if (ret > 0) {
			ret++;
		}
	}
	return dirty;
}

static int OSPDaemon_process_outbound(struct pollfd *pfd, int nfd)
{
	int j;

	for (j = 0; j < sd->output_count; j++)
		if (sd->output[j].fd == pfd->fd)
			break;
	if (j < sd->output_count) {
		if (OSPDaemon_driver_send(&sd->output[j]) == 0) {
			pfd->events = 0;
		}
	}

	return 0;
}

static int OSPDaemon_flush_out(struct OSPDaemon_SensorDescription *d)
{
	int j;

	for (j = 0; j < d->output_count; j++) {
		if (!OSPDaemon_queue_isempty(&d->output[j].q)) {
			OSPDaemon_driver_send(&d->output[j]);
		}
	}

	return 0;
}

static int OSPDaemon_init_outbound(struct pollfd *pfd, int nfd, int start)
{
	int i, nstart;
	OSP_STATUS_t oret;

	nstart = start;
	for (i = 0; i < sd->output_count; i++) {
		OSPDaemon_queue_init(&sd->output[i].q);
		if (sd->output[i].fd < 0)
			continue;
		nstart++;
		pfd[i+start].fd = sd->output[i].fd;
		pfd[i+start].events = 0;

		if (sd->output[i].noprocess) continue;

		sd->output[i].ResultDesc.pResultReadyCallback = OSPDaemon_callback;
		if (sd->output[i].type != -1) {
			DBG(DEBUG_INIT, "Register with OSP for output sensor %s", sd->sensor->name);
			oret = OSP_SubscribeSensorResult(&sd->output[i].ResultDesc, &sd->output[i].handle);
			if (oret != OSP_STATUS_OK) {
				DBG(DEBUG_INIT, "Sensor subscribe failed\n");
			}
		}
	}
	return nstart;
}

/*
 * start - index to start populating the pfd array.
 *
 * Returns start + number of entries used.
 */
static int OSPDaemon_init_inbound(struct pollfd *pfd, int nfd, int start)
{
	int i;
	OSP_STATUS_t oret;
	int nstart = start;

	if (sd->CalFile)
		ReadCalData(sd->CalFile);

	for (i = 0; i < sd->sensor_count; i++) {
		nstart++;
		if (sd->sensor[i].fd < 0)
			continue;

		pfd[i+start].fd = sd->sensor[i].fd;
		pfd[i+start].events = POLLIN;
		OSPDaemon_queue_init(&sd->sensor[i].q);
		DBG(DEBUG_INIT, "Input sensor noprocess = %i\n",
				sd->sensor[i].noprocess);
		if (sd->sensor[i].noprocess) continue;
		sd->sensor[i].sensor.pOptionalWriteDataCallback =
			OSPDaemon_write_cal_cb;

		DBG(DEBUG_INIT, "Register with OSP for input sensor %s", sd->sensor->name);

		oret = OSP_RegisterInputSensor(&sd->sensor[i].sensor, &sd->sensor[i].handle);
		if (oret != OSP_STATUS_OK) {
			DBG(DEBUG_INIT, "Sensor register failed (%i)\n", oret);
		}
	}
	return nstart;
}

int OSPDaemon_get_sensor_data(int in_sen_type, struct psen_data *out_data)
{
	FUNC_LOG;
	int i, j = 0, vallen, ret = 0;
	OSP_InputSensorData_t sdata;
	for (i = 0; i < sd->sensor_count; i++) {
		if (sd->sensor[i].sensor.SensorType == in_sen_type)
			break;
	}

	if (i == sd->sensor_count) {
		DBG(DEBUG_INIT, "Failed to find the sensor of type %d", in_sen_type);
		return -1;
	}

	ret = OSPDaemon_queue_get(&sd->sensor[i].output->q, &sdata);
	if (0 != ret) {
		//DBG(DEBUG_INIT, "There is no data for sensor type %d", in_sen_type);
		return ret;
	}

	vallen = extractOSP(sd->sensor[i].output->type,
			&sdata,
			&out_data->ts,
			out_data->val);
	out_data->flush_completed = 0;
	switch (vallen) {
	case 1:
		if (!(out_data->val[0]) && !(out_data->ts))
			out_data->flush_completed = 1;
		break;
	case 2:
		if (!(out_data->val[0]) && !(out_data->val[1]) && !(out_data->ts))
			out_data->flush_completed = 1;
		break;
	case 3:
		if (!(out_data->val[0]) && !(out_data->val[1]) && !(out_data->val[2])
			&& !(out_data->ts))
			out_data->flush_completed = 1;
		break;
	case 4:
		if (!(out_data->val[0]) && !(out_data->val[1]) && !(out_data->val[2])
			&& !(out_data->val[3]) && !(out_data->ts))
			out_data->flush_completed = 1;
		break;
	default:
		out_data->flush_completed = 0;
		break;
	}
	if (out_data->flush_completed)
		LOGI("%s ::: Flush completed event for sensor : %d \n", __func__, sd->sensor[i].output->type);
	return vallen;
}

int OSPDaemon_sensor_enable(int enable, int sensor_type){
	int i, ret;
	struct OSPDaemon_output *out = NULL;
	DBG(DEBUG_INIT ,"%s :: sensortype %d enable %d\n", __func__, sensor_type, enable);
	for (i = 0; i < sd->output_count; i++) {
		if (sd->output[i].ResultDesc.SensorType == sensor_type){
			out = &sd->output[i];
			break;
		}
	}
	if (out == NULL){
		LOGE("%s :: Invalid sensor type %d\n", __func__, sensor_type);
		return -1;
	}
	if (out){
		if(enable)
			OSPDaemon_power_control(out, 1);
		else
			OSPDaemon_power_control(out, 0);
	}
	return 0;
}

int OSPDaemon_batch(int sensor_type, int64_t sampling_period_ns, int64_t max_report_latency_ns)
{
	struct OSPDaemon_SensorDetail *sensor = NULL;
	int i, ret;
	for (i = 0; i < sd->sensor_count; i++){
		if(sd->sensor[i].sensor.SensorType == sensor_type){
			sensor = &sd->sensor[i];
			break;
		}
	}
	if (sensor == NULL){
		LOGE("%s :: Invalid sensor type %d\n", __func__, sensor_type);
		return -1;
	}
	DBG(DEBUG_INIT,"%s :: sensortype :: %d sensorname : %s \n", __func__,
	sd->sensor[i].sensor.SensorType , sd->sensor[i].sensor.SensorName);
	DBG(DEBUG_INIT,"%s :: sampling period : 0x%llx MRL :: 0x%llx\n", __func__,
	sampling_period_ns, max_report_latency_ns);
	ret = OSPDaemon_driver_batch(sensor, sensor_type, sampling_period_ns, max_report_latency_ns);
	return ret;
}

int OSPDaemon_flush(int sensor_type)
{
	struct OSPDaemon_SensorDetail *sensor = NULL;
	int i, ret = 0;
	for (i = 0; i < sd->sensor_count; i++){
		if(sd->sensor[i].sensor.SensorType == sensor_type){
			sensor = &sd->sensor[i];
			break;
		}
	}
	if (sensor == NULL){
		LOGE("%s :: Invalid sensor type %d\n", __func__, sensor_type);
		return -1;
	}
	DBG(DEBUG_INIT,"%s :: sensortype :: %d sensorname : %s \n", __func__,
	sd->sensor[i].sensor.SensorType , sd->sensor[i].sensor.SensorName);
	/*To be enabled later when firmware supports*/
	ret = OSPDaemon_driver_flush(sensor, sensor_type);
	return ret;
}
static void OSPDaemon(char *confname)
{
	int i, r = 0;
	int dirty;
	int err;

	if ((sd = OSPDaemon_config(confname)) == NULL) {
		fprintf(stderr, "Bad/invalid config.\n");
		return;
	}

	OSP_Initialize(&SystemDesc);

	for (i = 0; i < nfd; i++) {
		pfd[i].fd = 0;
		pfd[i].events = 0;
	}

	OSPDaemon_driver_setup_in(sd->sensor, sd->sensor_count);
	r = OSPDaemon_init_inbound(pfd, nfd, r);
	OSPDaemon_driver_setup_out(sd->output, sd->output_count);
	nfd = OSPDaemon_init_outbound(pfd, nfd, r);
	sd->powerfd = OSPDaemon_power_init(sd);
	if (sd->powerfd > 0) {
		pfd[nfd].fd = sd->powerfd;
		pfd[nfd].events |= POLLIN;
		nfd++;
	}

	for (i = 0; i < nfd; i++) {
		DBG(DEBUG_INIT, "%i: fd %i events = %i\n", i, pfd[i].fd, pfd[i].events);
	}

	DBG(DEBUG_INIT, "Signalling on the extern osp init sem 0x%X", &osp_sync);
	err = sem_post(&osp_sync);
	if (-1 == err) {
		DBG(DEBUG_INIT, "Sem post failed error - %s", strerror(errno));
		return;
	}

	dirty = 0;
	while (!mainstatus) {
		if (poll(pfd, nfd, -1) <= 0) {
			DBG(DEBUG_LOOP, "poll failed with error %s", strerror(errno));
			continue;
		}

		for (i = 0; i < nfd; i++) {
			if (pfd[i].revents & POLLIN) {
				DBG(DEBUG_LOOP, "IN event\n");
				if (pfd[i].fd == sd->powerfd) {
					OSPDaemon_power_process(sd,
								sd->powerfd);
				} else {
					dirty = OSPDaemon_process_inbound(
								&pfd[i], pfd,
								nfd, dirty);
				}
			}
			if (pfd[i].revents & POLLOUT) {
				DBG(DEBUG_LOOP, "%s:%i OUT event\n", __func__, __LINE__);
				OSPDaemon_process_outbound(&pfd[i], nfd);
			}
		}

		OSPDaemon_flush_out(sd);

		if (dirty) {
			DBG(DEBUG_LOOP, "%s:%i PROCESS\n", __func__, __LINE__);
			if (OSP_DoForegroundProcessing() == OSP_STATUS_IDLE)
				dirty = 0;
			OSP_DoBackgroundProcessing();
		}
	}
}

static void OSPDaemon_version(void)
{
	const OSP_Library_Version_t *ver;
	OSP_GetLibraryVersion(&ver);

	printf("OSPDaemon compiled "__DATE__"@"__TIME__"\n");
	printf("Library version: %s\n", ver->VersionString);
	printf("Library build: %s\n", ver->buildTime);
}

static void OSPDaemon_help(const char *name)
{
	fprintf(stderr, "Usage %s [-c configfile|-d level|-v|-h|-p]\n", name);
	fprintf(stderr, "-c configfile: Use configfile\n");
	fprintf(stderr, "-d level:      Sets the debug level\n");
	fprintf(stderr, "-p:            Disables power management.\n");
	fprintf(stderr, "               All sensors default on\n");
	fprintf(stderr, "-v:            Dumps version info and exits.\n");
	fprintf(stderr, "-h:            Prints this message and exits.\n");
}

int OSPDaemon_looper(int argc, char **argv)
{
	char *confname = NULL;
	int ar = 1, i;
	int quiet = 0;
	int ret;
	DBG(DEBUG_INIT,"%s :: argc = %d\n", __func__, argc);
	if (argc > 1) {
		do {
			if (argv[ar][0] == '-') {
				switch (argv[ar][1]) {
				case 'c':	/* Config file */
					if (ar+1 < argc) {
						confname = argv[ar+1];
						ar++;
					}
					break;
				case 'd':	/* Debug level */
					if (ar+1 < argc) {
						debug_level = strtol(argv[ar+1],								NULL, 0);
						ar++;
					}
					break;
				case 'h':	/* Help */
					OSPDaemon_help(argv[0]);
					exit(0);
					break;
				case 'p':	/* Disable PM */
					DBG(DEBUG_INIT,"%s :: disable pm\n", __func__);
					disablepm = 1;
					break;
				case 'n':	/* Ignore cal file */
					ignorecal = 1;
					break;
				case 'q':
					quiet = 1;
					break;
				case 'v':	/* Version */
					OSPDaemon_version();
					exit(0);
					break;
				}
				ar++;
			}
		} while (ar < argc);
	}
	if (confname == NULL)
		confname = "OSPDaemon.conf";
	if (!quiet) printf("Reading config from %s\n", confname);
	if (!quiet)  printf("Debug level 0x%08x\n", debug_level);
	if (disablepm) {
		if (!quiet) printf("Disabling Power Management\n");
	}
	/* Catch SIGPIPE? */
	/* Driver inits. Replace with linker magic? */
	OSPDaemon_iio_init();
	OSPDaemon_inputreader_init();
	OSPDaemon_input_init();
	OSPDaemon_filecsv_init();
	/* End driver inits */

    /* once config and calib data read is complete, send config done command */
	OSPDaemon(confname);
	return 0;
}
