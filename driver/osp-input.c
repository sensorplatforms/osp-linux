/*
 * Copyright (C) 2041 Audience, Inc.
 * Written by Hunyue Yau <hy-git@hy-research.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include "MQ_sensors.h"
#include <linux/osp-sh.h>

/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
#define G_MAX			8000
#if 0
static char *in_pname[NUM_PRIVATE_SENSOR_TYPE] = {
	[PSENSOR_ACCELEROMETER_RAW] = "acc_raw",
	[PSENSOR_ACCELEROMETER_UNCALIBRATED] = "acc_uncal",
};
#endif

struct osp_input_sensor {
	struct input_dev *input_dev;
	char *name;
} and_sensor[NUM_ANDROID_SENSOR_TYPE] = {
	[SENSOR_ACCELEROMETER] = {
		.name = "fm-accelerometer",
	},
	[SENSOR_MAGNETIC_FIELD] = {
		.name = "fm-magnetometer",
	},
	[SENSOR_GYROSCOPE] = {
		.name = "fm-gyroscope",
	},
	[SENSOR_ROTATION_VECTOR] = {
		.name = "fm-rotation-vector",
	},
	[SENSOR_MAGNETIC_FIELD_UNCALIBRATED] = {
		.name = "fm-uncalibrated-magnetometer",
	},
	[SENSOR_GYROSCOPE_UNCALIBRATED] = {
		.name = "fm-uncalibrated-gyroscope",
	},
	[SENSOR_ORIENTATION] = {
		.name = "fm-compass-orientation",
	},
	[SENSOR_LINEAR_ACCELERATION] = {
		.name = "fm-linear-acceleration",
	},
	[SENSOR_GRAVITY] = {
		.name = "fm-gravity",
	},
	[SENSOR_STEP_COUNTER] = {
		.name = "fm-step-counter",
	},
	[SENSOR_STEP_DETECTOR] = {
		.name = "fm-step-detector",
	},
	[SENSOR_SIGNIFICANT_MOTION] = {
		.name = "fm-significant-motion",
	},
	[SENSOR_PRESSURE] = {
		.name = "pressure",
	},
	[SENSOR_GEOMAGNETIC_ROTATION_VECTOR] = {
		.name = "fm-geomagnetic-rotation-vector",
	},
	[SENSOR_GAME_ROTATION_VECTOR] = {
		.name = "fm-game-rotation-vector",
	},
};

static ssize_t osp_get_enable(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return 0;
}

static ssize_t osp_set_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct osp_input_sensor *osp_input = input_get_drvdata(input);
	int i;
	unsigned long v;

	if (strict_strtoul(buf, 0, &v))
		return -EINVAL;

	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (and_sensor[i].input_dev == input) {
			break;
		}
	}

	if (i == NUM_ANDROID_SENSOR_TYPE)
		return -EINVAL;
	OSP_Sensor_State(i, 0, v);

	return count;
}
#if 0
static DEVICE_ATTR(enable_new, S_IRUGO|S_IWUSR, osp_get_enable, osp_set_enable);
#else
static DEVICE_ATTR(enable_new, S_IRUGO|S_IWUSR|S_IWUGO, osp_get_enable, osp_set_enable);
#endif

static struct attribute *osp_attributes[] = {
	&dev_attr_enable_new.attr,
	NULL
};

static struct attribute_group osp_attribute_group = {
	.attrs = osp_attributes
};

static int osp_input_open(struct input_dev *input)
{
	return 0;
}

static void osp_input_close(struct input_dev *dev)
{
}

static void dataready(int sensor, int prv,
			void *private, long long ts,
			union OSP_SensorData *sensordata)
{
	struct osp_input_sensor *sen = private;
	struct input_dev *input_dev = sen->input_dev;
	static int counter = 0;
	int PSensor;

	if (prv != 0) return;

	switch (sensor) {
	case SENSOR_STEP_DETECTOR:
	case SENSOR_SIGNIFICANT_MOTION:
		if (!input_dev) break;
		input_report_abs(input_dev, ABS_X, 1);
		input_report_abs(input_dev, ABS_GAS, counter);
		counter++;
		input_sync(input_dev);
		break;
	case SENSOR_STEP_COUNTER:
	case SENSOR_PRESSURE:
		break;
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		if (!input_dev) break;
		input_report_abs(input_dev, ABS_X, sensordata->xyz.x);
		input_report_abs(input_dev, ABS_Y, sensordata->xyz.y);
		input_report_abs(input_dev, ABS_Z, sensordata->xyz.z);
		input_sync(input_dev);
		break;
	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
	case SENSOR_ORIENTATION:
		if (!input_dev) break;
		input_report_abs(input_dev, ABS_X, sensordata->xyz.x);
		input_report_abs(input_dev, ABS_Y, sensordata->xyz.y);
		input_report_abs(input_dev, ABS_Z, sensordata->xyz.z);
		input_sync(input_dev);
		break;
	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
		if (!input_dev) break;
		input_report_abs(input_dev, ABS_GAS, sensordata->quat.r);
		input_report_abs(input_dev, ABS_X, sensordata->quat.x);
		input_report_abs(input_dev, ABS_Y, sensordata->quat.y);
		input_report_abs(input_dev, ABS_Z, sensordata->quat.z);
		input_sync(input_dev);
		break;
        case PSENSOR_ACCELEROMETER_UNCALIBRATED|SENSOR_DEVICE_PRIVATE_BASE:
                PSensor = sensor & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!input_dev) break;
                input_report_abs(input_dev, ABS_X, sensordata->xyz.x);
                input_report_abs(input_dev, ABS_Y, sensordata->xyz.y);
                input_report_abs(input_dev, ABS_Z, sensordata->xyz.z);
		input_sync(input_dev);
                break;
        case PSENSOR_ACCELEROMETER_RAW|SENSOR_DEVICE_PRIVATE_BASE:
                PSensor = sensor & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!input_dev) break;
                input_report_abs(input_dev, ABS_X, sensordata->xyz.x);
                input_report_abs(input_dev, ABS_Y, sensordata->xyz.y);
                input_report_abs(input_dev, ABS_Z, sensordata->xyz.z);
		input_sync(input_dev);
                break;
        default:
                break;
	}
}


static int osp_input_create(struct device *dev, int sensor, int private)
{
	struct input_dev *input_dev;
	struct osp_input_sensor *sen;
	int err;

	if (private == 0) {
		sen = and_sensor;
	} else {
		return -EINVAL;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	sen[sensor].input_dev = input_dev;

	input_dev->open = osp_input_open;
	input_dev->close = osp_input_close;
	input_set_drvdata(input_dev, &sen[sensor]);

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_THROTTLE, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_GAS, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = sen[sensor].name;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->relbit[0] |= BIT_MASK(REL_X);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(dev,
			"unable to register input polled device %s: %d\n",
			input_dev->name, err);
		input_free_device(input_dev);
		return err;
	}
	err = sysfs_create_group(&input_dev->dev.kobj, &osp_attribute_group);
	if (err) {
		printk("HY-DBG: %s:%i - err = %i\n", __func__, __LINE__, err);
	}	

	OSP_Sensor_Register(sensor, private, dataready, &sen[sensor]);

	return 0;
}

static int osp_input_probe(struct platform_device *pdev)
{
	osp_input_create(&pdev->dev, SENSOR_ACCELEROMETER, 0);
	osp_input_create(&pdev->dev, SENSOR_MAGNETIC_FIELD, 0);
	osp_input_create(&pdev->dev, SENSOR_GYROSCOPE, 0);
	osp_input_create(&pdev->dev, SENSOR_ROTATION_VECTOR, 0);
	osp_input_create(&pdev->dev, SENSOR_ORIENTATION, 0);
	osp_input_create(&pdev->dev, SENSOR_LINEAR_ACCELERATION, 0);
	osp_input_create(&pdev->dev, SENSOR_GRAVITY, 0);

	return 0;
}
static int osp_input_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver osp_input_driver = {
	.probe = osp_input_probe,
	.remove = osp_input_remove,
	.driver	= {
		.name = "osp-output",
		.owner = THIS_MODULE,
	},
};

static int __init osp_input_init(void)
{
	return platform_driver_register(&osp_input_driver);
}

module_init(osp_input_init);


MODULE_DESCRIPTION("OSP Input driver");
MODULE_AUTHOR("Hunyue Yau <hy-git@hy-research.com>");
MODULE_LICENSE("GPL");
