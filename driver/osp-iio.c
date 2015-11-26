#define DEBUG
/**
 * Copyright (c) 2015 Hunyue Yau for Audience
 *
 * Based on iio_simple_dummy by:
 *      Copyright (c) 2011 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * A reference industrial I/O driver to illustrate the functionality available.
 *
 * There are numerous real drivers to illustrate the finer points.
 * The purpose of this driver is to provide a driver with far more comments
 * and explanatory notes than any 'real' driver would have.
 * Anyone starting out writing an IIO driver should first make sure they
 * understand all of this driver except those bits specifically marked
 * as being present to allow us to 'fake' the presence of hardware.

 * Simple OSP IIO output layer.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/osp-sh.h>
#include <linux/iio/triggered_buffer.h>

#include "osp-sensors.h"
#undef KERNEL_VERSION_3_1
#define KERNEL_VERSION_3_10
/* Simulate periodic data */
//static struct timer_list osp_timer;

enum {
	axis_x,
	axis_y,
	axis_z,
	axis_r,
};

static ssize_t osp_iio_renable(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t osp_iio_wenable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len);

static IIO_DEVICE_ATTR(enable, S_IWUSR|S_IRUSR|S_IRGRP|S_IWGRP,
			&osp_iio_renable, &osp_iio_wenable, 0);

static struct attribute *osp_iio_attrs[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	NULL,
};

static const struct attribute_group osp_iio_group = {
	.attrs = osp_iio_attrs,
};
/*
 * Description of available channels
 *
 * This array of structures tells the IIO core about what the device
 * actually provides for a given channel.
 */
static struct iio_chan_spec step_channels[] = {
	{
		.type = IIO_ACCEL,	/* Nothing better defined */
		.modified = 1,
		.channel = 0,
		.address = 0,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};
static struct iio_chan_spec accel_channels[] = {
	/*
	 * 'modified' (i.e. axis specified) acceleration channel
	 * in_accel_z_raw
	 */
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel = 0,
		.address = 0,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel = 1,
		.address = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Y,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_y,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel = 2,
		.address = 2,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Z,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_z,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	/*
	 * Convenience macro for timestamps. 4 is the index in
	 * the buffer.
	 */
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static struct iio_chan_spec gyro_channels[] = {
	/*
	 * 'modified' (i.e. axis specified) acceleration channel
	 * in_accel_z_raw
	 */
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Y,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_y,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Z,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_z,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	/*
	 * Convenience macro for timestamps. 4 is the index in
	 * the buffer.
	 */
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static struct iio_chan_spec mag_channels[] = {
	/*
	 * 'modified' (i.e. axis specified) acceleration channel
	 * in_accel_z_raw
	 */
	{
		.type = IIO_MAGN,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 12),
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Y,
		.scan_type = IIO_ST('s', 32, 32, 12),
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_y,
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Z,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_z,
		.scan_type = IIO_ST('s', 32, 32, 12),
	},
	/*
	 * Convenience macro for timestamps. 4 is the index in
	 * the buffer.
	 */
	IIO_CHAN_SOFT_TIMESTAMP(3),
};
static struct iio_chan_spec linacc_channels[] = {
	/*
	 * 'modified' (i.e. axis specified) acceleration channel
	 * in_accel_z_raw
	 */
	{
		.type = IIO_ACCEL,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Y,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_y,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Z,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_z,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	/*
	 * Convenience macro for timestamps. 4 is the index in
	 * the buffer.
	 */
	IIO_CHAN_SOFT_TIMESTAMP(3),
};
static struct iio_chan_spec orient_channels[] = {
	/*
	 * 'modified' (i.e. axis specified) acceleration channel
	 * in_accel_z_raw
	 */
	{
		.type = IIO_INCLI,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 12),
	},
	{
		.type = IIO_INCLI,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Y,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_y,
		.scan_type = IIO_ST('s', 32, 32, 12),
	},
	{
		.type = IIO_INCLI,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Z,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_z,
		.scan_type = IIO_ST('s', 32, 32, 12),
	},
	/*
	 * Convenience macro for timestamps. 4 is the index in
	 * the buffer.
	 */
	IIO_CHAN_SOFT_TIMESTAMP(3),
};
static struct iio_chan_spec rotvec_channels[] = {
	/*
	 * 'modified' (i.e. axis specified) acceleration channel
	 * in_accel_z_raw
	 */
	{
		.type = IIO_QUATERNION,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_QUATERNION,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Y,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_y,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_QUATERNION,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_Z,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_z,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	{
		.type = IIO_QUATERNION,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_R,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		BIT(IIO_CHAN_INFO_RAW),
		.scan_index = axis_r,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	/*
	 * Convenience macro for timestamps. 4 is the index in
	 * the buffer.
	 */
	IIO_CHAN_SOFT_TIMESTAMP(4),
};
#if 0
static struct iio_chan_spec press_channels[] = {
	{
		.type = IIO_PRESSURE,
		.modified = 1,
		/* Channel 2 is use for modifiers */
		.channel2 = IIO_MOD_X,
		.info_mask_separate =
		/*
		 * Internal bias correction value. Applied
		 * by the hardware or driver prior to userspace
		 * seeing the readings. Typically part of hardware
		 * calibration.
		 */
		0,
		.scan_index = axis_x,
		.scan_type = IIO_ST('s', 32, 32, 24),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};
#endif
struct osp_iio_sensor {
	struct iio_dev *indio_dev;
	struct iio_trigger *trigger;
	struct mutex lock;
	int sensor;
	int private;
	int state;
	long long ts;
	union OSP_SensorData data;
};

struct osp_iio_sensor *osp_sensors_and[NUM_ANDROID_SENSOR_TYPE];
struct osp_iio_sensor *osp_sensors_prv[NUM_PRIVATE_SENSOR_TYPE];

static int osp_sensor_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask);
static int osp_sensor_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask);
/*
 * Device type specific information.
 */
static const struct iio_info osp_sensor_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &osp_sensor_read_raw,
	.write_raw = &osp_sensor_write_raw,
	.attrs = &osp_iio_group,
};

static const struct OSP_SensorDesc {
	char *name;
	char *trigname;
	struct iio_chan_spec *channels;
	int num_channels;
	struct iio_info const *info;
	int usebuffer;
	int useevent;
} and_sensor[NUM_ANDROID_SENSOR_TYPE] = {
	[SENSOR_GYROSCOPE] = {
		.name = "osp_gyro",
		.trigname = "osp_gyro",
		.channels = gyro_channels,
		.num_channels = ARRAY_SIZE(gyro_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_GYROSCOPE_UNCALIBRATED] = {
		.name = "osp_uncal_gyro",
		.trigname = "osp_uncal_gyro",
		.channels = gyro_channels,
		.num_channels = ARRAY_SIZE(gyro_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_ACCELEROMETER] = {
		.name = "osp_accel",
		.trigname = "osp_accel",
		.channels = accel_channels,
		.num_channels = ARRAY_SIZE(accel_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_GRAVITY] = {
		.name = "osp_gravity",
		.trigname = "osp_gravity",
		.channels = accel_channels,
		.num_channels = ARRAY_SIZE(accel_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_MAGNETIC_FIELD] = {
		.name = "osp_mag",
		.trigname = "osp_mag",
		.channels = mag_channels,
		.num_channels = ARRAY_SIZE(mag_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_MAGNETIC_FIELD_UNCALIBRATED] = {
		.name = "osp_uncal_mag",
		.trigname = "osp_uncal_mag",
		.channels = mag_channels,
		.num_channels = ARRAY_SIZE(mag_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_ORIENTATION] = {
		.name = "osp_compass_orientation",
		.trigname = "osp_compass_orientation",
		.channels = orient_channels,
		.num_channels = ARRAY_SIZE(mag_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_LINEAR_ACCELERATION] = {
		.name = "osp_linear_acceleration",
		.trigname = "osp_linear_acceleration",
		.channels = linacc_channels,
		.num_channels = ARRAY_SIZE(linacc_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_ROTATION_VECTOR] = {
		.name = "osp_rotation_vector",
		.trigname = "osp_rotation_vector",
		.channels = rotvec_channels,
		.num_channels = ARRAY_SIZE(rotvec_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_GAME_ROTATION_VECTOR] = {
		.name = "osp_game_rotation_vector",
		.trigname = "osp_game_rotation_vector",
		.channels = rotvec_channels,
		.num_channels = ARRAY_SIZE(rotvec_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_GEOMAGNETIC_ROTATION_VECTOR] = {
		.name = "osp_geo_rotation_vector",
		.trigname = "osp_geo_rotation_vector",
		.channels = rotvec_channels,
		.num_channels = ARRAY_SIZE(rotvec_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[SENSOR_SIGNIFICANT_MOTION] = {
		.name = "osp_sigmot",
		.trigname = "osp_sigmot",
		.channels = NULL,
		.num_channels = 0,
		.info = &osp_sensor_info,
		.usebuffer = 0,
		.useevent = 1,
	},
	[SENSOR_STEP_COUNTER] = {
		.name = "osp_step",
		.trigname = "osp_step",
		.channels = step_channels,
		.num_channels = ARRAY_SIZE(step_channels),
		.info = &osp_sensor_info,
		.usebuffer = 0,
		.useevent = 1,
	},
	[SENSOR_PRESSURE] = {
		.name = "osp_pressure",
		.trigname = "osp_pressure",
		.channels = NULL,
		.num_channels = 0,
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
}, prv_sensor[NUM_PRIVATE_SENSOR_TYPE] = {
	[PSENSOR_ACCELEROMETER_RAW] = {
		.name = "osp_accel_raw",
		.trigname = "osp_accel_raw",
		.channels = accel_channels,
		.num_channels = ARRAY_SIZE(accel_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[PSENSOR_ACCELEROMETER_UNCALIBRATED] = {
		.name = "osp_uncal_accel",
		.trigname = "osp_uncal_accel",
		.channels = accel_channels,
		.num_channels = ARRAY_SIZE(accel_channels),
		.info = &osp_sensor_info,
		.usebuffer = 1,
		.useevent = 0,
	},
	[PSENSOR_CONTEXT_DEVICE_MOTION] = {
		.name = "osp_context",
		.trigname = "osp_context",
		.usebuffer = 0,
	},
};

/* ------- sysfs ------- */

static ssize_t osp_iio_renable(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct osp_iio_sensor *osp_sensor = iio_priv(indio_dev);
	ssize_t len = 0;

	len += sprintf(buf, "%i\n", osp_sensor->state);

	return len;
}

static ssize_t osp_iio_wenable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct osp_iio_sensor *osp_sensor = iio_priv(indio_dev);
	int en;

	if (buf[0] == '1')
		en = 1;
	else
		en = 0;

	OSP_Sensor_State(osp_sensor->sensor, osp_sensor->private, en);
	osp_sensor->state = en;

	return len;
}

/* --------------------- */

/**
 * osp_sensor_read_raw() - data read function.
 * @indio_dev:	the struct iio_dev associated with this device instance
 * @chan:	the channel whose data is to be read
 * @val:	first element of returned value (typically INT)
 * @val2:	second element of returned value (typically MICRO)
 * @mask:	what we actually want to read. 0 is the channel, everything else
 *		is as per the info_mask in iio_chan_spec.
 */
static int osp_sensor_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask)
{
	struct osp_iio_sensor *osp_sensor = iio_priv(indio_dev);
	int ret = -EINVAL;
	pr_debug("%s:%i called\n", __func__, __LINE__);

	mutex_lock(&osp_sensor->lock);
	switch (mask) {
	case 0: /* magic value - channel value read */
		switch (chan->type) {
		case IIO_ACCEL:
		case IIO_ANGL_VEL:
		case IIO_MAGN:
			switch (chan->scan_index) {
			case axis_x:
				*val = osp_sensor->data.xyz.x;
				break;
			case axis_y:
				*val = osp_sensor->data.xyz.y;
				break;
			case axis_z:
				*val = osp_sensor->data.xyz.z;
				break;
			default:
				break;
			}
			ret = IIO_VAL_INT;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	mutex_unlock(&osp_sensor->lock);
	return ret;
}

/**
 * osp_sensor_write_raw() - data write function.
 * @indio_dev:	the struct iio_dev associated with this device instance
 * @chan:	the channel whose data is to be read
 * @val:	first element of returned value (typically INT)
 * @val2:	second element of returned value (typically MICRO)
 * @mask:	what we actually want to read. 0 is the channel, everything else
 *		is as per the info_mask in iio_chan_spec.
 *
 * Note that all raw writes are assumed IIO_VAL_INT and info mask elements
 * are assumed to be IIO_INT_PLUS_MICRO unless the callback write_raw_get_fmt
 * in struct iio_info is provided by the driver.
 */
static int osp_sensor_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
#if 0
	int i;
	int ret = 0;
	struct iio_dummy_state *st = iio_priv(indio_dev);
	pr_debug("%s:%i called\n", __func__, __LINE__);

	switch (mask) {
	case 0:
		if (chan->output == 0)
			return -EINVAL;

		/* Locking not required as writing single value */
		mutex_lock(&st->lock);
		st->dac_val = val;
		mutex_unlock(&st->lock);
		return 0;
	case IIO_CHAN_INFO_CALIBBIAS:
		mutex_lock(&st->lock);
		/* Compare against table - hard matching here */
		for (i = 0; i < ARRAY_SIZE(dummy_scales); i++)
			if (val == dummy_scales[i].val &&
			    val2 == dummy_scales[i].val2)
				break;
		if (i == ARRAY_SIZE(dummy_scales))
			ret = -EINVAL;
		else
			st->accel_calibscale = &dummy_scales[i];
		mutex_unlock(&st->lock);
		return ret;
	default:
		return -EINVAL;
	}
#else
	return 0;
#endif
}

/**
 * osp_iio_sensor_init() - Sets up sensor specific details.
 * @indio_dev: the iio device structure
 *
 * Most drivers have one of these to set up default values,
 * reset the device to known state etc.
 *
 * Init data to known values for debugging.
 */
static int osp_iio_sensor_init(struct iio_dev *indio_dev, int sensor, int type)
{
	struct osp_iio_sensor *osp_sensor = iio_priv(indio_dev);

	osp_sensor->indio_dev = indio_dev;
	memset(&osp_sensor->data, 0, sizeof(osp_sensor->data));
	osp_sensor->sensor = sensor;

	if (type == 0) {
		switch (sensor) {
		case SENSOR_ACCELEROMETER:
			osp_sensor->data.xyz.x = 0;
			osp_sensor->data.xyz.y = 1;
			osp_sensor->data.xyz.z = 2;
			break;
		case SENSOR_GYROSCOPE:
			osp_sensor->data.xyz.x = 2002;
			osp_sensor->data.xyz.y = 2003;
			osp_sensor->data.xyz.z = 2004;
			break;
		case SENSOR_MAGNETIC_FIELD:
			osp_sensor->data.xyz.x = 4002;
			osp_sensor->data.xyz.y = 4003;
			osp_sensor->data.xyz.z = 4004;
			break;
		case SENSOR_ROTATION_VECTOR:
			osp_sensor->data.quat.x = 6001;
			osp_sensor->data.quat.y = 6002;
			osp_sensor->data.quat.z = 6003;
			osp_sensor->data.quat.r = 1004;
			break;
		}
	}

	return 0;
}

static int osp_iio_trigger(struct iio_trigger *trig, bool state)
{
	//struct iio_dev *indio_dev = trig->private_data;
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct osp_iio_sensor *osp_sensor;
	osp_sensor = iio_priv(indio_dev);

	pr_debug("%s",__func__);
	OSP_Sensor_State(osp_sensor->sensor, osp_sensor->private, state);
	osp_sensor->state = state;

	return 0;
}

static irqreturn_t osp_iio_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct osp_iio_sensor *osp_sensor = iio_priv(indio_dev);
	if (indio_dev->buffer->scan_timestamp){
#if defined(KERNEL_VERSION_3_1)
		iio_push_to_buffer(indio_dev->buffer, (u8 *)&osp_sensor->data,
		osp_sensor->ts);
#elif defined(KERNEL_VERSION_3_10)
		*(s64 *)((u8 *)&osp_sensor->data +
			ALIGN(sizeof(osp_sensor->data), sizeof(s64))) = osp_sensor->ts;
		iio_push_to_buffers(indio_dev, (u8 *)&osp_sensor->data);
#endif
	}
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_trigger_ops osp_iio_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &osp_iio_trigger,
};

static const struct iio_buffer_setup_ops osp_iio_buffer_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

/*
 * Call back from core OSP code for data ready.
 * Store data and trigger if buffers used.
 */
static void dataready(int sensor, int prv,
                        void *private, long long ts,
                        union OSP_SensorData *sensordata)
{
	struct iio_dev *indio_dev = private;
	struct osp_iio_sensor *osp_sensor = iio_priv(indio_dev);

	osp_sensor->ts = ts;
	osp_sensor->data = *sensordata;
	if (osp_sensor->private == 0 && and_sensor[sensor].useevent) {
		switch (sensor) {
		case SENSOR_SIGNIFICANT_MOTION:
		case SENSOR_STEP_COUNTER:
			iio_push_event(indio_dev,
					IIO_MOD_EVENT_CODE(IIO_INTENSITY, 0,
						IIO_MOD_X,
						IIO_EV_TYPE_THRESH,
						IIO_EV_DIR_EITHER),
					ts);
			break;
		}
	}
	if ((osp_sensor->private == 0 && and_sensor[sensor].usebuffer) ||
		(osp_sensor->private == 1 && prv_sensor[sensor].usebuffer)) {
		iio_trigger_poll(osp_sensor->trigger, osp_sensor->ts);
		//pr_debug("iio trigger poll 0x%08x", ts);
	}
}

static int osp_iio_destroy(int sensor, int space)
{
	struct iio_dev *indio_dev;
	struct osp_iio_sensor **osp_sensors;
	if (space == 0)
		osp_sensors = osp_sensors_and;
	else
		osp_sensors = osp_sensors_prv;

 	indio_dev = osp_sensors[sensor]->indio_dev;

	/* Disable the sensor */
	OSP_Sensor_State(sensor, space, 0);
	osp_sensors[sensor]->state = 0;
	/* Unregister the device */
	iio_device_unregister(indio_dev);

	/* Device specific code to power down etc */

	/* Buffered capture related cleanup */
	iio_buffer_unregister(indio_dev);
#if 0
	iio_simple_dummy_unconfigure_buffer(indio_dev);
#endif
	/* Free all structures */
	iio_device_free(indio_dev);
	return 1;
}

/**
 * osp_iio_create() - Create a IIO sensor instance.
 * index - sensor id. enum space is determined by value of space.
 * space - enum space for sensors.
 * dev - caller used pointer.
 */
static int osp_iio_create(int index, int space, void *dev)
{
	int ret;
	struct iio_dev *indio_dev;
	struct osp_iio_sensor *st;
	const struct OSP_SensorDesc *desc;
	struct osp_iio_sensor **osp_sensor;

	if (space == 1) {	/* Private sensors */
		if (index >= NUM_PRIVATE_SENSOR_TYPE || index < 0)
			return -EINVAL;
		desc = prv_sensor;
		osp_sensor = osp_sensors_prv;
	} else if (space == 0) {	/* Android sensors */
		if (index >= NUM_ANDROID_SENSOR_TYPE || index < 0)
			return -EINVAL;
		desc = and_sensor;
		osp_sensor = osp_sensors_and;
	} else {
		return -EINVAL;
	}
	/*
	 * Allocate an IIO device.
	 *
	 * This structure contains all generic state
	 * information about the device instance.
	 * It also has a region (accessed by iio_priv()
	 * for chip specific state information.
	 */
	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	st = iio_priv(indio_dev);
	st->private = space;
	st->state = 0;
	st->data.xyz.x = 0;
	st->data.xyz.y = 0;
	st->data.xyz.z = 0;
	mutex_init(&st->lock);
	osp_iio_sensor_init(indio_dev, index, 0);

	indio_dev->dev.parent = dev;

	osp_sensor[index] = st;

	/*
	 * Set the device name.
	 *
	 * This is typically a part number and obtained from the module
	 * id table.
	 * e.g. for i2c and spi:
	 *    indio_dev->name = id->name;
	 *    indio_dev->name = spi_get_device_id(spi)->name;
	 */
	indio_dev->name = desc[index].name;

	/* Provide description of available channels */
	indio_dev->channels = desc[index].channels;
	indio_dev->num_channels = desc[index].num_channels;

	/*
	 * Provide device type specific interface functions and
	 * constant data.
	 */
	indio_dev->info = desc[index].info;

	/* Specify that device provides sysfs type interfaces */
	indio_dev->modes = INDIO_DIRECT_MODE |
				INDIO_BUFFER_TRIGGERED;
	st->trigger = iio_trigger_alloc(desc[index].trigname);

	if (st->trigger) {
		st->trigger->ops = &osp_iio_trigger_ops;
		iio_trigger_register(st->trigger);
	//	st->trigger->private_data = indio_dev;
		iio_trigger_set_drvdata(st->trigger, indio_dev);

	}

	if (desc[index].usebuffer) {
#if defined(KERNEL_VERSION_3_1)
		indio_dev->pollfunc = iio_alloc_pollfunc(
					&iio_pollfunc_store_time,
					&osp_iio_trigger_handler,
					IRQF_ONESHOT, indio_dev,
					desc[index].trigname);

		indio_dev->buffer = iio_kfifo_allocate(indio_dev);
		indio_dev->setup_ops = &osp_iio_buffer_setup_ops;

		ret = iio_buffer_register(indio_dev,
					desc[index].channels,
					desc[index].num_channels);

		indio_dev->buffer->scan_timestamp = 1;
#elif defined(KERNEL_VERSION_3_10)
		ret = iio_triggered_buffer_setup(indio_dev,
				&iio_pollfunc_store_time,
				&osp_iio_trigger_handler, NULL);
		indio_dev->buffer->scan_timestamp = 1;
		if (ret < 0)
			goto error_free_device;
#endif
	}
	OSP_Sensor_Register(index, space, dataready, indio_dev);
	ret = iio_device_register(indio_dev);
	if (ret < 0){
		goto error_unregister_buffer;
	}
	return 0;
error_unregister_buffer:
	iio_buffer_unregister(indio_dev);
#if 0
error_unconfigure_buffer:
	iio_simple_dummy_unconfigure_buffer(indio_dev);
error_unregister_events:
	iio_simple_dummy_events_unregister(indio_dev);
#endif
error_free_device:
	/* Note free device should only be called, before registration
	 * has succeeded. */
	iio_device_free(indio_dev);
error_ret:
	return ret;
}
#if 0
/* Dummy signal generator */
static void osp_poll_timer(unsigned long unused)
{
	int i;
	struct osp_iio_sensor *osp_sensor;;

	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (osp_sensors_and[i]) {
			osp_sensor = osp_sensors_and[i];
			if (i == SENSOR_ROTATION_VECTOR) {
				osp_sensor->data.quat.x++;
				osp_sensor->data.quat.y++;
				osp_sensor->data.quat.z++;
				osp_sensor->data.quat.r++;
				osp_sensor->data.quat.ts = 0x3fffffff00000000LL+jiffies;
			} else {
				osp_sensor->data.xyz.x++;
				osp_sensor->data.xyz.y++;
				osp_sensor->data.xyz.z++;
				osp_sensor->data.xyz.ts = 0x7fffffff00000000LL+jiffies;
			}
			if (and_sensor[i].usebuffer)
				iio_trigger_poll(osp_sensor->trigger, osp_sensor->ts);
		}
	}
	mod_timer(&osp_timer, jiffies+msecs_to_jiffies(500));
}
#endif
static int osp_iio_probe(struct platform_device *pdev)
{
	int ret, i;

	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		osp_sensors_and[i] = NULL;
	}

	for (i = 0; i < NUM_PRIVATE_SENSOR_TYPE; i++) {
		osp_sensors_prv[i] = NULL;
	}
#if 0
	osp_iio_create(SENSOR_GYROSCOPE, 0, NULL);
	osp_iio_create(SENSOR_GYROSCOPE_UNCALIBRATED, 0, NULL);
	osp_iio_create(SENSOR_ACCELEROMETER, 0, NULL);
	osp_iio_create(SENSOR_GRAVITY, 0, NULL);
	osp_iio_create(SENSOR_MAGNETIC_FIELD, 0, NULL);
	osp_iio_create(SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 0, NULL);
	osp_iio_create(SENSOR_ORIENTATION, 0, NULL);
	osp_iio_create(SENSOR_LINEAR_ACCELERATION, 0, NULL);
	osp_iio_create(SENSOR_ROTATION_VECTOR, 0, NULL);
	osp_iio_create(SENSOR_GAME_ROTATION_VECTOR, 0, NULL);
	osp_iio_create(SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 0, NULL);
	osp_iio_create(SENSOR_SIGNIFICANT_MOTION, 0, NULL);
	osp_iio_create(SENSOR_STEP_COUNTER, 0, NULL);
#endif
#if 1
	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (and_sensor[i].name) {
			ret = osp_iio_create(i, 0, NULL);
			if (ret < 0) {
				pr_debug("Cannot create %s\n", and_sensor[i].name);
			}
		}
	}
#if 0
	for (i = 0; i < NUM_PRIVATE_SENSOR_TYPE; i++) {
		if (prv_sensor[i].name) {
			ret = osp_iio_create(i, 1, NULL);
			if (ret < 0) {
				pr_debug("Cannot create %s\n", and_sensor[i].name);
			}
		}
	}
#endif
#endif
#if 0
	ret = osp_iio_create(SENSOR_ACCELEROMETER, 0, NULL);
	ret = osp_iio_create(SENSOR_GYROSCOPE, 0, NULL);
	ret = osp_iio_create(SENSOR_MAGNETIC_FIELD, 0, NULL);
	ret = osp_iio_create(SENSOR_ROTATION_VECTOR, 0, NULL);
	ret = osp_iio_create(SENSOR_ORIENTATION, 0, NULL);
	ret = osp_iio_create(SENSOR_LINEAR_ACCELERATION, 0, NULL);
	ret = osp_iio_create(SENSOR_GRAVITY, 0, NULL);
	ret = osp_iio_create(SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 0, NULL);
#endif
#if 0
	/* For dummy signal generator */
	if (ret >= 0) {
		setup_timer(&osp_timer, osp_poll_timer, 0);
		mod_timer(&osp_timer, jiffies + msecs_to_jiffies(1000));
	}
#endif
	return 0;
}

static int osp_iio_remove(struct platform_device *pdev)
{
	return 0;
}
static struct platform_driver osp_iio_driver = {
	.probe = osp_iio_probe,
	.remove = osp_iio_remove,
	.driver = {
		.name = "osp-output",
		.owner = THIS_MODULE,
	},
};
static __init int osp_iio_init(void)
{
	return platform_driver_register(&osp_iio_driver);
}
module_init(osp_iio_init);

/**
 * osp_iio_exit() - device driver removal
 *
 * Varies depending on bus type of the device.
 * As there is no device here, call remove directly.
 */
static __exit void osp_iio_exit(void)
{
	int i;
	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++)
		if (osp_sensors_and[i]) {
			osp_iio_destroy(i, osp_sensors_and[i]->private);
		}

	for (i = 0; i < NUM_PRIVATE_SENSOR_TYPE; i++)
		if (osp_sensors_prv[i]) {
			osp_iio_destroy(i, osp_sensors_and[i]->private);
		}
}
module_exit(osp_iio_exit);

MODULE_AUTHOR("Hunyue Yau <hy-git@hy-research.com>");
MODULE_DESCRIPTION("OSP IIO driver");
MODULE_LICENSE("GPL v2");
