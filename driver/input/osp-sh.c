/*
 * Copyright (C) 2041 Audience, Inc.
 * Written by Hunyue Yau <hy@hy-research.com>
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
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input-polldev.h>
#include <linux/workqueue.h>
#include "MQ_sensors.h"
#include "SensorPackets.h"
#include "osp_i2c_map.h"

#define NAME			"ospsh"
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
#define G_MAX			8000

static char *in_name[NUM_ANDROID_SENSOR_TYPE] = {
	[SENSOR_ACCELEROMETER] = "fm-accelerometer",
	[SENSOR_MAGNETIC_FIELD] = "fm-magnetometer",
	[SENSOR_GYROSCOPE] = "fm-gyroscope",
	[SENSOR_ROTATION_VECTOR] = "fm-rotation-vector",
	[SENSOR_MAGNETIC_FIELD_UNCALIBRATED] = "fm-uncalibrated-magnetometer",
	[SENSOR_GYROSCOPE_UNCALIBRATED] = "fm-uncalibrated-gyroscope",
	[SENSOR_ORIENTATION] = "fm-compass-orientation",
	[SENSOR_LINEAR_ACCELERATION] = "fm-linear-acceleration",
	[SENSOR_GRAVITY] = "fm-gravity",
	[SENSOR_STEP_COUNTER] = "fm-step-counter",
	[SENSOR_STEP_DETECTOR] = "fm-step-detector",
	[SENSOR_SIGNIFICANT_MOTION] = "fm-significant-motion",
	[SENSOR_PRESSURE] = "pressure",
	[SENSOR_GEOMAGNETIC_ROTATION_VECTOR] = "fm-geomagnetic-rotation-vector",
	[SENSOR_GAME_ROTATION_VECTOR] = "fm-game-rotation-vector",
};

static char *in_pname[NUM_PRIVATE_SENSOR_TYPE] = {
	[PSENSOR_ACCELEROMETER_RAW] = "acc_raw",
	[PSENSOR_ACCELEROMETER_UNCALIBRATED] = "acc_uncal",
};
/* ------------ OSP Packet parsing code -------------------- */
/***********************************************************************
 * @fn	  ParseSensorDataPkt
 *		  Top level parser for Sensor Data Packets.
 *
 * @param   [OUT]pOut - Sensor structure that will return the parsed values
 * @param   [IN]pPacket - Packet buffer containing the packet to parse
 * @param   [IN]pktSize - Size of the packet buffer provided
 *
 * @return  0 or Error code enum corresponding to the error encountered
 *
 ************************************************************************/
static int OSP_ParseSensorDataPkt_Private(
	SensorPacketTypes_t *pOut,
	HostIFPackets_t *pHif,
	ASensorType_t sensType,
	uint8_t sensSubType,
	uint8_t dSize,
	uint8_t dFormat,
	uint8_t timeFormat,
	uint8_t	tSize,
	uint8_t hasMetaData
)
{
	int errCode = -EPROTONOSUPPORT;
	int lengthParsed;
	int i;

	switch ((PSensorType_t)sensType) {
	case PSENSOR_ACCELEROMETER_RAW:
	case PSENSOR_MAGNETIC_FIELD_RAW:
	case PSENSOR_GYROSCOPE_RAW:
		if ((sensSubType == SENSOR_SUBTYPE_UNUSED) &&
			(dSize == DATA_SIZE_16_BIT) &&
			(dFormat == DATA_FORMAT_RAW) &&
			(timeFormat == TIME_FORMAT_RAW)) {
			/* Extract Raw sensor data from packet */
			pOut->SType = (ASensorType_t)M_PSensorToAndroidBase(sensType);
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.RawSensor.Axis[0] = BYTES_TO_SHORT(
					pHif->SensPktRaw.DataRaw[0],
					pHif->SensPktRaw.DataRaw[1]);
			pOut->P.RawSensor.Axis[1] = BYTES_TO_SHORT(
					pHif->SensPktRaw.DataRaw[2],
					pHif->SensPktRaw.DataRaw[3]);
			pOut->P.RawSensor.Axis[2] = BYTES_TO_SHORT(
					pHif->SensPktRaw.DataRaw[4],
					pHif->SensPktRaw.DataRaw[5]);
				
			/* Extract time stamp */
			pOut->P.RawSensor.TStamp.TS64 = 0; //helps clear higher 32-bit
			pOut->P.RawSensor.TStamp.TS8[3] = pHif->SensPktRaw.TimeStamp[0]; //MSB
			pOut->P.RawSensor.TStamp.TS8[2] = pHif->SensPktRaw.TimeStamp[1];
			pOut->P.RawSensor.TStamp.TS8[1] = pHif->SensPktRaw.TimeStamp[2];
			pOut->P.RawSensor.TStamp.TS8[0] = pHif->SensPktRaw.TimeStamp[3]; //LSB
			//TODO: 64-bit time stamp extension??
			errCode = 0;
			lengthParsed = SENSOR_RAW_DATA_PKT_SZ;
		}
		break;
	case PSENSOR_ACCELEROMETER_UNCALIBRATED:
		if ((dSize == DATA_SIZE_32_BIT) &&
			(dFormat == DATA_FORMAT_FIXPOINT) &&
			(timeFormat == TIME_FORMAT_FIXPOINT) &&
			(tSize == TIME_STAMP_64_BIT)) {
			/* Extract uncalibrated sensor data from packet */
			pOut->SType = (ASensorType_t)M_PSensorToAndroidBase(sensType);
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.UncalFixP.Axis[0] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Data,0);
			pOut->P.UncalFixP.Axis[1] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Data, 4);
			pOut->P.UncalFixP.Axis[2] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Data, 8);

			/* Check if META_DATA is set to 0x01 then read offset */
			if (hasMetaData) {
				pOut->P.UncalFixP.Offset[0] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Offset, 0);
				pOut->P.UncalFixP.Offset[1] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Offset, 4);
				pOut->P.UncalFixP.Offset[2] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Offset, 8);

				lengthParsed = UNCALIB_FIXP_DATA_OFFSET_PKT_SZ;
			} else
				lengthParsed = UNCALIB_FIXP_DATA_PKT_SZ;
			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember
				 * that HIF packets are Big-Endian formatted */
				pOut->P.UncalFixP.TimeStamp.TS8[i] = pHif->UncalPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
			}
			errCode = 0;
		}
		break;
	case SYSTEM_REAL_TIME_CLOCK:
		printk("HY-DBG: %s:%i type %i\n", __func__, __LINE__, sensType);
		break;
	default:
		printk("HY-DBG: %s:%i type %i\n", __func__, __LINE__, sensType);
		break;
	}
	if (errCode == 0)
		return lengthParsed;
	else
		return errCode;
}

static int OSP_ParseSensorDataPkt_Android(
	SensorPacketTypes_t *pOut,
	HostIFPackets_t *pHif,
	ASensorType_t sensType,
	uint8_t sensSubType,
	uint8_t dSize,
	uint8_t dFormat,
	uint8_t timeFormat,
	uint8_t	tSize,
	uint8_t hasMetaData
)
{
	int errCode = -EPROTONOSUPPORT;
	int lengthParsed;
	int i;

	switch ((ASensorType_t)sensType) {
	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
	case SENSOR_ORIENTATION:
	case SENSOR_LINEAR_ACCELERATION:
	case SENSOR_GRAVITY:
		if ((dSize == DATA_SIZE_32_BIT) &&
			(dFormat == DATA_FORMAT_FIXPOINT) &&
			(timeFormat == TIME_FORMAT_FIXPOINT) &&
			(tSize == TIME_STAMP_64_BIT)) {
			/* Extract sensor data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.CalFixP.Axis[0] =
				BYTES_TO_LONG_ARR(pHif->CalPktFixP.Data, 0);
			pOut->P.CalFixP.Axis[1] =
				BYTES_TO_LONG_ARR(pHif->CalPktFixP.Data, 4);
			pOut->P.CalFixP.Axis[2] =
				BYTES_TO_LONG_ARR(pHif->CalPktFixP.Data, 8);

			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data -
				 * remember that HIF packets are
				 * Big-Endian formatted
				 */
				pOut->P.CalFixP.TimeStamp.TS8[i] =
					pHif->CalPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
			}
			errCode = 0;
			lengthParsed = CALIBRATED_FIXP_DATA_PKT_SZ;
		}
		break;

	case SENSOR_GAME_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_ROTATION_VECTOR:
		if ((dSize == DATA_SIZE_32_BIT) &&
			(dFormat == DATA_FORMAT_FIXPOINT) &&
			(timeFormat == TIME_FORMAT_FIXPOINT) &&
			(tSize == TIME_STAMP_64_BIT)) {
			/* Extract Quaternion data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.QuatFixP.Quat[0] = BYTES_TO_LONG_ARR(
					pHif->QuatPktFixP.Data, 0);
			pOut->P.QuatFixP.Quat[1] = BYTES_TO_LONG_ARR(
					pHif->QuatPktFixP.Data, 4);
			pOut->P.QuatFixP.Quat[2] = BYTES_TO_LONG_ARR(
					pHif->QuatPktFixP.Data, 8);
			pOut->P.QuatFixP.Quat[3] = BYTES_TO_LONG_ARR(
					pHif->QuatPktFixP.Data, 12);

			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember
				 * that HIF packets are Big-Endian formatted */
				pOut->P.QuatFixP.TimeStamp.TS8[i] = pHif->QuatPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
			}
			errCode = 0;
			lengthParsed = QUATERNION_FIXP_DATA_PKT_SZ;
		}
		break;
	case SENSOR_SIGNIFICANT_MOTION:
		printk("HY-DBG: %s:%i\n", __func__, __LINE__);
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
	case SENSOR_PRESSURE:
	case SENSOR_STEP_DETECTOR:
	case SENSOR_STEP_COUNTER:
		if ((dSize == DATA_SIZE_32_BIT) &&
			(dFormat == DATA_FORMAT_FIXPOINT) &&
			(timeFormat == TIME_FORMAT_FIXPOINT) &&
			(tSize == TIME_STAMP_64_BIT)) {
			/* Extract Quaternion data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.UncalFixP.Axis[0] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Data, 0);
			pOut->P.UncalFixP.Axis[1] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Data, 4);
			pOut->P.UncalFixP.Axis[2] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Data, 8);

			/* Check if META_DATA is set to 0x01 then read offset */
			if (hasMetaData) {
				pOut->P.UncalFixP.Offset[0] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Offset, 0);
				pOut->P.UncalFixP.Offset[1] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Offset, 4);
				pOut->P.UncalFixP.Offset[2] = BYTES_TO_LONG_ARR(
					pHif->UncalPktFixP.Offset, 8);

				lengthParsed = UNCALIB_FIXP_DATA_OFFSET_PKT_SZ;
			} else
				lengthParsed = UNCALIB_FIXP_DATA_PKT_SZ;
			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember that
				 * HIF packets are Big-Endian formatted */
				pOut->P.UncalFixP.TimeStamp.TS8[i] =
					pHif->UncalPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
			}
			errCode = 0;
		}
		break;

	default:
		{
			int i;
			unsigned char *p;
		printk("HY-DBG: %s:%i type %i: ", __func__, __LINE__, sensType);
			p = (unsigned char *)pHif;
			for (i = 0; i < 8; i++) {
				printk("%02x ", p[i]);
			}
		printk("\n");
		
		}
		break;
	}
	if (errCode == 0)
		return lengthParsed;
	else
		return errCode;
}
static int16_t OSP_ParseSensorDataPkt(SensorPacketTypes_t *pOut,
		uint8_t *pPacket, uint16_t pktSize)
{
	HostIFPackets_t *pHif = (HostIFPackets_t*)pPacket;
	int errCode = -EPROTONOSUPPORT;
	uint8_t sensType;
	uint8_t sensSubType, dSize, dFormat, timeFormat,
		tSize, isPrivateType, hasMetaData;

	/* Sanity... */
	if ((pOut == NULL) || (pPacket == NULL))
		return (-ENOMEM);

	/* Get sensor type. */
	sensType = M_SensorType(pHif->SensPktRaw.Q.SensorIdByte);
	sensSubType = M_ParseSensorSubType(pHif->SensPktRaw.Q.AttributeByte);
	isPrivateType = pHif->SensPktRaw.Q.ControlByte &
					SENSOR_ANDROID_TYPE_MASK;
	hasMetaData = M_ParseSensorMetaData (pHif->SensPktRaw.Q.SensorIdByte);
	dSize = pHif->SensPktRaw.Q.AttributeByte & DATA_SIZE_MASK;
	dFormat = pHif->SensPktRaw.Q.ControlByte & DATA_FORMAT_MASK;
	timeFormat = pHif->SensPktRaw.Q.ControlByte & TIME_FORMAT_MASK;
	tSize = pHif->SensPktRaw.Q.AttributeByte & TIME_STAMP_SIZE_MASK;

	/* Check Sensor enumeration type Android or Private */  
	if (!isPrivateType) {
		/*Sensor Enumeration type is Android*/
		errCode = OSP_ParseSensorDataPkt_Android(pOut, pHif,
					(ASensorType_t)sensType,
					sensSubType, dSize,
					dFormat, timeFormat, tSize,
					hasMetaData);
	} else {
		/*Sensor Enumeration type is Private*/
		errCode = OSP_ParseSensorDataPkt_Private(pOut, pHif,
					(ASensorType_t)sensType,
					sensSubType, dSize,
					dFormat, timeFormat, tSize,
					hasMetaData);


	}
	return errCode;
}
/* ------------ END OSP Packet parsing code -------------------- */

struct osp_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_polled_dev *poll_dev;
	struct input_dev *in_dev[NUM_ANDROID_SENSOR_TYPE];
	struct input_dev *in_pdev[NUM_PRIVATE_SENSOR_TYPE];
	struct timer_list osp_timer;
	struct work_struct osp_work;
};

static struct osp_data *gOSP;

static struct work_queue *osp_workq;

static ssize_t osp_get_enable(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return 0;
}

static ssize_t osp_set_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct osp_data *osp = input_get_drvdata(input);
	int i;
	int retval;
	unsigned long v;

	if (strict_strtoul(buf, 0, &v)) 
		return -EINVAL;

	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (osp->in_dev[i] == input) {
			break;
		}
	}

	if (i == NUM_ANDROID_SENSOR_TYPE)
		return -EINVAL;

	printk("HY-DBG: %s:%i - osp %p\n", __func__, __LINE__, osp);
	if (v) {
		printk("HY-DBG: %s:%i - enabling %i\n", __func__, __LINE__, i);
		if (i < 0x30) {
			retval = i2c_smbus_read_byte_data(osp->client, 0x20+i);
		}
	} else {
		printk("HY-DBG: %s:%i - disabling %i\n", __func__, __LINE__, i);
		if (i < 0x30) {
			retval = i2c_smbus_read_byte_data(osp->client, 0x50+i);
		}
	}

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

static int osp_i2c_read(struct osp_data *osp, u8 addr, u8 *data, int len)
{
#if 1
	struct i2c_msg msgs[] = {
		{
			.addr = osp->client->addr,
			.flags = osp->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = osp->client->addr,
			.flags = osp->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(osp->client->adapter, msgs, 2);
#else
	struct i2c_msg msgs[3];

	msgs[0].addr = osp->client->addr;
	msgs[0].flags = osp->client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &addr;

	if (len > 4096) {
		msgs[1].addr = osp->client->addr;
		msgs[1].flags = osp->client->flags | I2C_M_RD;
		msgs[1].len = len;
		msgs[1].buf = data;

		msgs[2].addr = osp->client->addr;
		msgs[2].flags = osp->client->flags | I2C_M_RD;
		msgs[2].len = len-4096;
		msgs[2].buf = data+4096;

		return i2c_transfer(osp->client->adapter, msgs, 3);
	} else {
		msgs[1].addr = osp->client->addr;
		msgs[1].flags = osp->client->flags | I2C_M_RD;
		msgs[1].len = len;
		msgs[1].buf = data;
		return i2c_transfer(osp->client->adapter, msgs, 2);
	}

#endif
}
#if 0
static irqreturn_t osp_isr(int irq, void *dev)
{
	struct osp_data *osp = dev;
	int err;

	/* data ready is the only possible interrupt type */
	osp_report_acceleration_data(tj9);

	err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
	if (err < 0)
		dev_err(&osp->client->dev,
			"error clearing interrupt status: %d\n", err);

	return IRQ_HANDLED;
}
#endif
static int osp_enable(struct osp_data *osp)
{
	return 0;
}
static void osp_disable(struct osp_data *osp)
{
}

static int osp_input_open(struct input_dev *input)
{
	struct osp_data *osp = input_get_drvdata(input);
#if 0
	int i;
	int retval;

	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (osp->in_dev[i] == input) {
			break;
		}
	}
	printk("HY-DBG: %s:%i - closing %i\n", __func__, __LINE__, i);

	if (i == NUM_ANDROID_SENSOR_TYPE)
		return 0;
	if (i < 0x30) {
		retval = i2c_smbus_read_byte_data(osp->client, 0x20+i);
	}
#endif
	return osp_enable(osp);
}

static void osp_input_close(struct input_dev *dev)
{
	struct osp_data *osp = input_get_drvdata(dev);
#if 0
	int i;
	int retval;

	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (osp->in_dev[i] == dev) {
			break;
		}
	}

	printk("HY-DBG: %s:%i - closing %i\n", __func__, __LINE__, i);

	if (i == NUM_ANDROID_SENSOR_TYPE)
		return;
	if (i < 0x30) {
		retval = i2c_smbus_read_byte_data(osp->client, 0x50+i);
	}
#endif
	osp_disable(osp);
}

static void osp_init_input_device(struct osp_data *osp,
				struct input_dev *input_dev, char *name)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_THROTTLE, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_GAS, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = name;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &osp->client->dev;
	input_dev->relbit[0] |= BIT_MASK(REL_X);
}

static int osp_setup_other_input(struct osp_data *osp)
{
	int i = 0, j = 0;
	struct input_dev *input_dev;
	int err;

	for (i = 2; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		osp->in_dev[i] = NULL;
	}
	for (i = 0; i < NUM_PRIVATE_SENSOR_TYPE; i++) {
		osp->in_pdev[i] = NULL;
	}

	for (i = 2; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (in_name[i] == NULL) 
			continue;
		input_dev = input_allocate_device();
		if (!input_dev) {
			dev_err(&osp->client->dev, "input device allocate failed\n");
			err = -ENOMEM;
			goto err_input_alloc;
		} else {
			osp->in_dev[i] = input_dev;
			input_dev->open = osp_input_open;
			input_dev->close = osp_input_close;
			input_set_drvdata(input_dev, osp);
			osp_init_input_device(osp, input_dev, in_name[i]);
		}
	}
	for (j = 0; j < NUM_PRIVATE_SENSOR_TYPE; j++) {
		if (in_pname[j] == NULL) 
			continue;
		input_dev = input_allocate_device();
		if (!input_dev) {
			dev_err(&osp->client->dev, "input device allocate failed\n");
			err = -ENOMEM;
			goto err_input_alloc;
		} else {
			osp->in_pdev[j] = input_dev;
			input_dev->open = osp_input_open;
			input_dev->close = osp_input_close;
			input_set_drvdata(input_dev, osp);
			osp_init_input_device(osp, input_dev, in_pname[j]);
		}
	}
	for (i = 2; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (in_name[i] == NULL) 
			continue;
		err = input_register_device(osp->in_dev[i]);
		if (err) {
			dev_err(&osp->client->dev,
				"unable to register input polled device %s: %d\n",
			osp->in_dev[i]->name, err);
			goto err_input_reg;
		}
		err = sysfs_create_group(&osp->in_dev[i]->dev.kobj, &osp_attribute_group);
		if (err) {
			printk("HY-DBG: %s:%i - err = %i\n", __func__, __LINE__, err);
		}	
	}
	for (j = 0; j < NUM_PRIVATE_SENSOR_TYPE; j++) {
		if (in_pname[j] == NULL) 
			continue;
		err = input_register_device(osp->in_pdev[j]);
		if (err) {
			dev_err(&osp->client->dev,
				"unable to register input polled device %s: %d\n",
			osp->in_pdev[j]->name, err);
			goto err_input_reg;
		}
	}

	return 0;
err_input_reg:
	i--;
	for (; i > 1; i--) {
		if (in_name[i] == NULL) 
			continue;
		input_unregister_device(osp->in_dev[i]);
	}
	if (j != 0) {
		j--;
		for (; j > 1; j--) {
			if (in_pname[j] == NULL) 
				continue;
			input_unregister_device(osp->in_dev[j]);
		}
	}

err_input_alloc:	
	for (i = 2; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		if (in_name[i] == NULL) 
			continue;
		if (osp->in_dev) 
			input_free_device(osp->in_dev[i]);
	}
	for (j = 0; j < NUM_PRIVATE_SENSOR_TYPE; j++) {
		if (in_name[j] == NULL) 
			continue;
		if (osp->in_dev) 
			input_free_device(osp->in_pdev[j]);
	}

	return err;
}
static int osp_setup_input_device(struct osp_data *osp)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&osp->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	osp->input_dev = input_dev;
	osp->in_dev[1] = input_dev;

	input_dev->open = osp_input_open;
	input_dev->close = osp_input_close;
	input_set_drvdata(input_dev, osp);

	osp_init_input_device(osp, input_dev, in_name[1]);

	err = input_register_device(osp->input_dev);
	if (err) {
		dev_err(&osp->client->dev,
			"unable to register input polled device %s: %d\n",
			osp->input_dev->name, err);
		input_free_device(osp->input_dev);
		return err;
	}
	err = sysfs_create_group(&osp->in_dev[1]->dev.kobj, &osp_attribute_group);
	if (err) {
		printk("HY-DBG: %s:%i - err = %i\n", __func__, __LINE__, err);
	}	

	osp_setup_other_input(osp);
	return 0;
}

static void OSP_ReportSensor(struct osp_data *osp, 
			SensorPacketTypes_t *spack)
{
	int PSensor;
	static int counter = 0;

	/* printk("HY-DBG:%s:%i(%i)\n", __func__,__LINE__, spack->SType); */
	switch(spack->SType) {
	case SENSOR_STEP_DETECTOR:
	case SENSOR_SIGNIFICANT_MOTION:
		printk("HY-DBG:%s:%i(%i) %p %i %i %i\n", __func__,__LINE__, spack->SType, osp->in_dev[spack->SType], spack->P.UncalFixP.Axis[0], spack->P.UncalFixP.Axis[1], spack->P.UncalFixP.Axis[2]);

		if (!osp->in_dev[spack->SType]) break;
		input_report_abs(osp->in_dev[spack->SType], ABS_X, 1);
		input_report_abs(osp->in_dev[spack->SType], ABS_GAS, counter);
		counter++;
		input_sync(osp->in_dev[spack->SType]);
		break;
	case SENSOR_STEP_COUNTER:
	case SENSOR_PRESSURE:
		printk("HY-DBG:%s:%i(%i) %p %i %i %i\n", __func__,__LINE__, spack->SType, osp->in_dev[spack->SType], spack->P.UncalFixP.Axis[0], spack->P.UncalFixP.Axis[1], spack->P.UncalFixP.Axis[2]);

	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		if (!osp->in_dev[spack->SType]) break;
		input_report_abs(osp->in_dev[spack->SType], ABS_X, spack->P.UncalFixP.Axis[0]);
		input_report_abs(osp->in_dev[spack->SType], ABS_Y, spack->P.UncalFixP.Axis[1]);
		input_report_abs(osp->in_dev[spack->SType], ABS_Z, spack->P.UncalFixP.Axis[2]);
		input_sync(osp->in_dev[spack->SType]);
		break;
	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
	case SENSOR_ORIENTATION:
		/* printk("HY-DBG:%s:%i(%i) %p %i %i %i\n", __func__,__LINE__, spack->SType, osp->in_dev[spack->SType], spack->P.CalFixP.Axis[0], spack->P.CalFixP.Axis[1], spack->P.CalFixP.Axis[2]); */
		if (!osp->in_dev[spack->SType]) break;
		input_report_abs(osp->in_dev[spack->SType], ABS_X, spack->P.CalFixP.Axis[0]);
		input_report_abs(osp->in_dev[spack->SType], ABS_Y, spack->P.CalFixP.Axis[1]);
		input_report_abs(osp->in_dev[spack->SType], ABS_Z, spack->P.CalFixP.Axis[2]);
		input_sync(osp->in_dev[spack->SType]);
		break;
	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
		/* printk("HY-DBG:%s:%i(%i) %p\n", __func__,__LINE__, spack->SType, osp->in_dev[spack->SType]); */
		if (!osp->in_dev[spack->SType]) break;
		input_report_abs(osp->in_dev[spack->SType], ABS_GAS, spack->P.QuatFixP.Quat[0]);
		input_report_abs(osp->in_dev[spack->SType], ABS_X, spack->P.QuatFixP.Quat[1]);
		input_report_abs(osp->in_dev[spack->SType], ABS_Y, spack->P.QuatFixP.Quat[2]);
		input_report_abs(osp->in_dev[spack->SType], ABS_Z, spack->P.QuatFixP.Quat[3]);
		input_sync(osp->in_dev[spack->SType]);
		break;
	case PSENSOR_ACCELEROMETER_UNCALIBRATED|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		/* printk("HY-DBG:%s:%i(%i) %p %i %i %i\n", __func__,__LINE__, spack->SType, osp->in_pdev[PSensor], spack->P.UncalFixP.Axis[0], spack->P.UncalFixP.Axis[1], spack->P.UncalFixP.Axis[2]); */
		if (!osp->in_pdev[PSensor]) break;
		input_report_abs(osp->in_pdev[PSensor], ABS_X, spack->P.UncalFixP.Axis[0]);
		input_report_abs(osp->in_pdev[PSensor], ABS_Y, spack->P.UncalFixP.Axis[1]);
		input_report_abs(osp->in_pdev[PSensor], ABS_Z, spack->P.UncalFixP.Axis[2]);
		input_sync(osp->in_pdev[PSensor]);
		break;
	case PSENSOR_ACCELEROMETER_RAW|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		/* printk("HY-DBG:%s:%i(%i) %p %i %i %i\n", __func__,__LINE__, spack->SType, osp->in_pdev[PSensor], spack->P.RawSensor.Axis[0], spack->P.RawSensor.Axis[1], spack->P.RawSensor.Axis[2]); */
		if (!osp->in_pdev[PSensor]) break;
		input_report_abs(osp->in_pdev[PSensor], ABS_X, spack->P.RawSensor.Axis[0]);
		input_report_abs(osp->in_pdev[PSensor], ABS_Y, spack->P.RawSensor.Axis[1]);
		input_report_abs(osp->in_pdev[PSensor], ABS_Z, spack->P.RawSensor.Axis[2]);
		input_sync(osp->in_pdev[PSensor]);
		break;
	default:
		break;
	}
	/* printk("HY-DBG:%s:%i(%i)\n", __func__,__LINE__, spack->SType); */
}

static unsigned char *osp_pack;
static unsigned char *osp_pack2;

static void osp_work_q(struct work_struct *work)
{
	struct osp_data *osp = container_of(work, struct osp_data, osp_work);
	uint16_t plen = 0, plen_old = 0;
	uint16_t plen2 = 0, plen2_old = 0;
	unsigned char *pack_ptr;
	int pack_count;
	SensorPacketTypes_t spack;
	int err;
	int ret;
	uint32_t intlen;
	int gotpack1 = 0;
	int gotpack2 = 0;

	ret = osp_i2c_read(osp, OSP_INT_LEN, (unsigned char *)&intlen, 4);
	if ((ret >= 0) && (intlen&OSP_INT_DRDY)) {
		plen = (intlen >> 4);
		plen_old = plen;
		if (plen>0 && plen < 8192) {
			ret = osp_i2c_read(osp, OSP_DATA_OUT, osp_pack, plen);
			if (ret < 0) return;
			gotpack1 = 1;
		}
	}

	ret = osp_i2c_read(osp, OSP_INT_LEN, (unsigned char *)&intlen, 4);
	if ((ret >= 0) && (intlen&OSP_INT_DRDY)) {
		plen2 = (intlen >> 4);
		plen2_old = plen2;
		if (plen2>0 && plen2 < 8192) {
			ret = osp_i2c_read(osp, OSP_DATA_OUT, osp_pack2, plen2);
			if (ret >= 0)
				gotpack2 = 1;
		}
	}

	if (gotpack1) {
		pack_count = 0;
		pack_ptr = osp_pack;
		do {
			err = OSP_ParseSensorDataPkt(&spack, pack_ptr, plen);
			if (err>0) {
				OSP_ReportSensor(osp, &spack);
			} else {
				printk("OSP packet parsing error = %i, pack_count = %i plen = %i, plen_old = %i\n", err, pack_count, plen, plen_old);
				break;
			}
			plen -= err;
			pack_ptr += err;
			pack_count++;
		} while (plen > 0 && plen2 < 8192);
	}
	if (gotpack2) {
		pack_count = 0;
		pack_ptr = osp_pack2;
		do {
			err = OSP_ParseSensorDataPkt(&spack, pack_ptr, plen2);
			if (err>0) {
				OSP_ReportSensor(osp, &spack);
			} else {
				printk("OSP packet parsing error = %i, pack_count = %i plen2 = %i, plen2_old = %i\n", err, pack_count, plen2, plen2_old);
				return;
			}
			plen2 -= err;
			pack_ptr += err;
			pack_count++;
		} while (plen2 > 0 && plen2 < 8192);
	}
		/* printk("HY-DBG: pack_count = %i\n", pack_count); */
}
static void osp_poll_timer(unsigned long _osp)
{
	struct osp_data *osp = (void *)_osp;
	queue_work(osp_workq, &osp->osp_work);
#if 0
	mod_timer(&osp->osp_timer, jiffies+msecs_to_jiffies(20));
#else
	mod_timer(&osp->osp_timer, jiffies+1);
#endif
}

static irqreturn_t osp_irq_thread(int irq, void *dev)
{
	struct osp_data *osp = dev;

	queue_work(osp_workq, &osp->osp_work);

	return IRQ_HANDLED;
}

static void osp_poll(struct input_polled_dev *dev)
{
#if 0
	struct osp_data *osp = dev->private;
	int retval;
	int maxcount = 0x1ff;
	uint16_t plen, plen_old;
	unsigned char *pack_ptr;
	int pack_count;
	SensorPacketTypes_t spack;
	int err;
	
	do {
		retval = i2c_smbus_read_byte_data(osp->client, OSP_INT_REASON);
		if (retval&OSP_INT_DRDY) {
			osp_i2c_read(osp, OSP_DATA_LEN_L, (unsigned char *)&plen, 2);
			pack_count = 0;
			if (plen>0 && plen < 8192) {
				pack_ptr = osp_pack;
				retval = osp_i2c_read(osp, OSP_DATA_OUT, osp_pack, plen);
				plen_old = plen;
				do {
					err = OSP_ParseSensorDataPkt(&spack, pack_ptr, plen);
					if (err>=0) {
						OSP_ReportSensor(osp, &spack);
					} else {
						printk("%s:%i: OSP packet parsing error = %i, pack_count = %i plen = %i, plen_old = %i\n", __func__, __LINE__, err, pack_count, plen, plen_old);
						break;
					}
					plen -= err;
					pack_ptr += err;
					pack_count++;
				} while (plen > 0);
			}
			/* printk("HY-DBG: pack_count = %i\n", pack_count); */
		} else
			break;
		maxcount--;
	} while (maxcount);
	if (maxcount < 0x3f) {
		printk("WARNING - Maxcount = %i\n", maxcount);
		if (!maxcount)
			printk("Too busy in poll, maxcount = %i\n", maxcount);	
	}
#endif
}
static void osp_polled_input_open(struct input_polled_dev *dev)
{
	struct osp_data *osp = dev->private;

	osp_enable(osp);
}

static void osp_polled_input_close(struct input_polled_dev *dev)
{
	struct osp_data *osp = dev->private;

	osp_disable(osp);
}


static int osp_setup_polled_device(struct osp_data *osp)
{
	int err;
	struct input_polled_dev *poll_dev;
	poll_dev = input_allocate_polled_device();

	printk("HY-DBG: %s:%i\n", __func__, __LINE__);
	if (!poll_dev) {
		dev_err(&osp->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	osp->poll_dev = poll_dev;
	osp->input_dev = poll_dev->input;

	poll_dev->private = osp;
	poll_dev->poll = osp_poll;
	poll_dev->open = osp_polled_input_open;
	poll_dev->close = osp_polled_input_close;
#if 1
	poll_dev->poll_interval = 20;	/* 50Hz */
#else
	poll_dev->poll_interval = 0;	/* 50Hz */
#endif

	printk("HY-DBG: %s:%i\n", __func__, __LINE__);
	osp_init_input_device(osp, poll_dev->input, in_name[1]);
	printk("HY-DBG: %s:%i\n", __func__, __LINE__);

	osp->in_dev[1] = poll_dev->input;
	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&osp->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}

	osp_setup_other_input(osp);
	printk("HY-DBG: %s:%i\n", __func__, __LINE__);

	return 0;
}

static void osp_teardown_polled_device(struct osp_data *osp)
{
	/* Need to add other input devices */
	input_unregister_polled_device(osp->poll_dev);
	input_free_polled_device(osp->poll_dev);
}

static int osp_verify(struct osp_data *osp)
{
	int retval;

	retval = i2c_smbus_read_byte_data(osp->client, OSP_WHOAMI);
	if (retval < 0) {
		dev_err(&osp->client->dev, "read err int source, 0x%x\n", retval);
		goto out;
	}

	retval = (retval != 0x54) ? -EIO : 0;

	retval = i2c_smbus_read_byte_data(osp->client, OSP_CONFIG);

out:
	return retval;
}
static int osp_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct osp_data *osp;
	int err;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	osp = kzalloc(sizeof(*osp), GFP_KERNEL);
	if (!osp) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	gOSP = osp;
	osp_pack = kzalloc(8192, GFP_KERNEL);
	if (!osp_pack) {
		dev_err(&client->dev,
			"failed to allocate memory for packet data\n");
		return -ENOMEM;
	}
	osp_pack2 = kzalloc(8192, GFP_KERNEL);
	if (!osp_pack2) {
		dev_err(&client->dev,
			"failed to allocate memory for packet data\n");
		return -ENOMEM;
	}

	osp->client = client;

	err = osp_verify(osp);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_free_mem;
	}

	i2c_set_clientdata(client, osp);

	osp_workq = create_workqueue("osp_queue");
	if (osp_workq) {
		INIT_WORK(&osp->osp_work, osp_work_q);
	}

#if 0
	osp_setup_polled_device(osp);
#else
	osp_setup_input_device(osp);
#endif

	client->irq = -1;	/* Force it for now */

	if (client->irq > 0) {
		err = request_threaded_irq(client->irq, NULL, osp_irq_thread, IRQF_TRIGGER_LOW, "osp-sh", osp);
		if (err < 0) {
			dev_err(&client->dev,
				"irq request failed %d, error %d\n",
				client->irq, err);
		}

	} else {
		err = -EINVAL;
	}
	setup_timer(&osp->osp_timer, osp_poll_timer, (unsigned long)osp);

	if (err < 0) {
		mod_timer(&osp->osp_timer, jiffies + msecs_to_jiffies(10));
	} 

	return 0;
#if 0
err_free_irq:
	free_irq(client->irq, osp);
err_destroy_input:
	input_unregister_device(osp->input_dev);
#endif
#if 0
err_pdata_exit:
	if (tj9->pdata.exit)
		tj9->pdata.exit();
#endif
err_free_mem:
	kfree(osp);
	return err;
}

static int osp_remove(struct i2c_client *client)
{
	struct osp_data *osp = i2c_get_clientdata(client);

	if (client->irq) {
#if 0
		sysfs_remove_group(&client->dev.kobj, &kxtj9_attribute_group);
#endif
		free_irq(client->irq, osp);
		input_unregister_device(osp->input_dev);
	} else {
		osp_teardown_polled_device(osp);
	}

	kfree(osp);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int osp_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct osp_data *osp = i2c_get_clientdata(client);
	struct input_dev *input_dev = osp->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		osp_disable(osp);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int osp_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct osp_data *osp = i2c_get_clientdata(client);
	struct input_dev *input_dev = osp->input_dev;
	int retval = 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		osp_enable(osp);

	mutex_unlock(&input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(osp_pm_ops, osp_suspend, osp_resume);

static const struct i2c_device_id osp_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, osp_id);

static struct i2c_driver osp_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.pm	= &osp_pm_ops,
	},
	.probe		= osp_probe,
	.remove		= osp_remove,
	.id_table	= osp_id,
};
#if 0
module_i2c_driver(osp_driver);
#else
static int __init osp_init(void)
{
	return i2c_add_driver(&osp_driver);
}
module_init(osp_init);

#endif


MODULE_DESCRIPTION("OSP driver");
MODULE_AUTHOR("Hunyue Yau <hy@hy-research.com>");
MODULE_LICENSE("GPL");
