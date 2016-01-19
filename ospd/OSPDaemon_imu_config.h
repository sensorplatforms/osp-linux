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
#define SH_CONFIG_READ_STORE "/sys/bus/platform/devices/osp-output.0/sensorhub_config_read"
#define SH_CONFIG_WRITE_STORE "/sys/bus/platform/devices/osp-output.0/sensorhub_config_write"
#define SH_CONFIG_HOST_VERSION			"/sys/bus/platform/devices/osp-output.0/osp_version"
#define	SYSFS_CMD_SIZE 1024
#define PARAM_ID_ENABLE 1
#define PARAM_ID_BATCH 2
#define PARAM_ID_FLUSH 3

int OSPDaemon_imu_batch(int handle,
			int64_t sampling_period_ns,
			int64_t max_report_latency_ns);
int OSPDaemon_imu_flush(int handle);
int OSPDaemon_imu_config_done();
int OSPDaemon_imu_enable(int sensorid, int enable);
void OSPDaemon_imu_config_version(uint8_t *version);
