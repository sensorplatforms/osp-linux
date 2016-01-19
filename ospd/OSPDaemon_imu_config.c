/*
 * (C) Copyright 2015 HY Research LLC for Audience.
 *
 * Licensed under Apache Public License.
 */

/*
 * Configuration commands for osp compliant IMU.
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
#include "OSPDaemon_imu_config.h"
#include "OSPDaemon.h"
#define DBGOUT(x...) DBG(DEBUG_INDRIVER, "IIO: "x)

int OSPDaemon_imu_batch(int handle,
	int64_t sampling_period_ns, int64_t max_report_latency_ns)
{
	int fd, sz;
	char buff[SYSFS_CMD_SIZE];
	int seq_no = 0;
	int param_id = PARAM_ID_BATCH;
	DBGOUT("%s :: param_id : %d seq_no : %d sensor_id : %d SP: 0x%016llx MRL : 0x%016llx\n",
		__func__, param_id, seq_no, handle, sampling_period_ns, max_report_latency_ns);
	memset(buff, '\0', SYSFS_CMD_SIZE);
	fd = open(SH_CONFIG_WRITE_STORE, O_RDWR);
	if(fd > 0){
		sz = snprintf(buff, SYSFS_CMD_SIZE, "0x%02x %d 0x%02x 0x%016llx 0x%016llx",\
		handle, seq_no, param_id, sampling_period_ns, max_report_latency_ns);
		write(fd, &buff, sz);
	}
	else {
		LOGE("Failed to open the write config store");
		return -1;
	}
	close(fd);
	return 0;
}

int OSPDaemon_imu_flush(int handle)
{
	int fd, sz;
	char buff[SYSFS_CMD_SIZE];
	int seq_no = 0;
	int param_id = PARAM_ID_FLUSH;
	DBGOUT("%s :: param_id : %d seq_no : %d sensor_id :%d \n", __func__, param_id, seq_no, handle);
	memset(buff, '\0', SYSFS_CMD_SIZE);
	fd = open(SH_CONFIG_WRITE_STORE, O_RDWR);
	if(fd > 0){
		sz = snprintf(buff, SYSFS_CMD_SIZE, "0x%02x %d 0x%02x",
				handle, seq_no, param_id);
		write(fd, &buff, sz);
	}
	else {
		LOGE("Failed to open the write config store");
		return -1;
	}
	close(fd);
	return 0;
}

int OSPDaemon_imu_config_done()
{
	int fd, sz;
	char buff[SYSFS_CMD_SIZE];
	int seq_no = 0, handle = 0, data = 0;
	int param_id = 0x20;
	DBGOUT("%s :: param_id : %d seq_no : %d sensor_id : %d data : %d\n",
	__func__, param_id, seq_no, handle, data);
	memset(buff, '\0', SYSFS_CMD_SIZE);
	fd = open(SH_CONFIG_WRITE_STORE, O_RDWR);
	if(fd > 0){
		sz = snprintf(buff, SYSFS_CMD_SIZE, "0x%02x %d 0x%02x %d",
				handle, seq_no, param_id, data);
		write(fd, &buff, sz);
	}
	else {
		LOGE("Failed to open the write config store");
		return -1;
	}
	close(fd);
	return 0;
}

int OSPDaemon_imu_enable(int sensorid, int enable)
{
	int fd;
	char buff[SYSFS_CMD_SIZE];
	int seq_no = 0, sz;
	char fname[PATH_MAX];
	int param_id = PARAM_ID_ENABLE;
	DBGOUT("%s :: param_id : %d seq_no : %d sensor_id : %d  enable : %d \n",
	__func__, param_id, seq_no, sensorid, enable);
	memset(buff, '\0', SYSFS_CMD_SIZE);
	fd = open(SH_CONFIG_WRITE_STORE, O_RDWR);
	if(fd > 0){
		sz = snprintf(buff, SYSFS_CMD_SIZE, "0x%02x %d 0x%02x %d",
				sensorid, seq_no, param_id, enable);
		write(fd, &buff, sz);
	}
	else {
		LOGE("Failed to open the write config store");
		return -1;
	}
	close(fd);
	return 0;
}

void OSPDaemon_imu_config_version(uint8_t *version)
{
	int fd;
	if (!version)
		return -1;
	fd = open(SH_CONFIG_HOST_VERSION, O_RDWR);
	if(fd > 0){
		write(fd, version, MAX_VERSION_LEN);
	}
	else {
		LOGE("Failed to open the write config store");
		return -1;
	}
	close(fd);
}
