#define DEBUG
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
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include "osp-sensors.h"
#include "SensorPackets.h"
#include "osp_i2c_map.h"
#include <linux/osp-sh.h>
#include "osp-hif-apis.h"

#define NAME			"ospsh"

/* Private sensors */
#define FEAT_PRIVATE	(1<<0)

#ifdef DUMMY_CONFIG_CMD
static u16 error_code;
static u8 dummy_read_data[4];
static u8 dummy_write_data[4];
#endif
static struct OSP_Sensor {
	void (*dataready)(int sensor, int prv,
		void *private, long long ts, union OSP_SensorData *sensordata);
	void *private;
} and_sensor[NUM_ANDROID_SENSOR_TYPE], prv_sensor[NUM_PRIVATE_SENSOR_TYPE];

struct osp_data {
	struct i2c_client *client;
	struct timer_list osp_timer;
	struct work_struct osp_work;
	struct mutex lock;
	unsigned int features;
	struct hif_write_data resp_data;
};

static struct osp_data *gOSP;
static struct work_queue *osp_workq;

/* Number of packets to parse */
#define NUM_PACK	2
static unsigned char *osp_pack[NUM_PACK];

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
			pOut->SType = (ASensorType_t)M_PSensorToAndroidBase(
				sensType);
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
			pOut->P.RawSensor.TStamp.TS64 = 0; //helps clear higher
			pOut->P.RawSensor.TStamp.TS8[3] =
				pHif->SensPktRaw.TimeStamp[0];          //MSB
			pOut->P.RawSensor.TStamp.TS8[2] =
				pHif->SensPktRaw.TimeStamp[1];
			pOut->P.RawSensor.TStamp.TS8[1] =
				pHif->SensPktRaw.TimeStamp[2];
			pOut->P.RawSensor.TStamp.TS8[0] =
				pHif->SensPktRaw.TimeStamp[3];          //LSB
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
			pOut->SType = (ASensorType_t)M_PSensorToAndroidBase(
				sensType);
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
				pOut->P.UncalFixP.TimeStamp.TS8[i] =
					pHif->UncalPktFixP.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
		}
		break;
	case SYSTEM_REAL_TIME_CLOCK:
		break;
	default:
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
	uint8_t tSize,
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
					pHif->CalPktFixP.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
			lengthParsed = CALIBRATED_FIXP_DATA_PKT_SZ;
		}
		break;

	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
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
				pOut->P.QuatFixP.TimeStamp.TS8[i] =
					pHif->QuatPktFixP.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
			lengthParsed = QUATERNION_FIXP_DATA_PKT_SZ;
		}
		break;

	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
	case SENSOR_PRESSURE:
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
					pHif->UncalPktFixP.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
		}
		break;

	case SENSOR_SIGNIFICANT_MOTION:
		if ((dSize == DATA_SIZE_8_BIT) &&
		    (dFormat == DATA_FORMAT_RAW) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
			/* Extract SignificantMotion data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.SigMotion.MotionDetected =
				pHif->SignificantMotion.significantMotionDetected;
			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember that HIF
				  packets are Big-Endian formatted */
				pOut->P.SigMotion.TimeStamp.TS8[i] =
					pHif->SignificantMotion.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
			lengthParsed = SIGNIFICANTMOTION_FIXP_DATA_PKT_SZ;
		}
		break;

	case SENSOR_STEP_DETECTOR:
		if ((dSize == DATA_SIZE_8_BIT) &&
		    (dFormat == DATA_FORMAT_RAW) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
			/* Extract StepDetector data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.StepDetector.StepDetected =
				pHif->StepDetector.stepDetected;

			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember that HIF
				  packets are Big-Endian formatted */
				pOut->P.StepDetector.TimeStamp.TS8[i] =
					pHif->StepDetector.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
			lengthParsed = STEPDETECTOR_DATA_PKT_SZ;
		}
		break;

	case SENSOR_STEP_COUNTER:
		if ((dSize == DATA_SIZE_64_BIT) &&
		    (dFormat == DATA_FORMAT_RAW) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
			/* Extract StepCounter data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.StepCount.NumStepsTotal = BYTES_TO_LONGLONG(
				pHif->StepCounter.NumStepsTotal[0],
				pHif->StepCounter.NumStepsTotal[1],
				pHif->StepCounter.NumStepsTotal[2],
				pHif->StepCounter.NumStepsTotal[3],
				pHif->StepCounter.NumStepsTotal[4],
				pHif->StepCounter.NumStepsTotal[5],
				pHif->StepCounter.NumStepsTotal[6],
				pHif->StepCounter.NumStepsTotal[7]);
			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember that HIF
				  packets are Big-Endian formatted */
				pOut->P.StepCount.TimeStamp.TS8[i] =
					pHif->StepCounter.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
			lengthParsed = STEPCOUNTER_DATA_PKT_SZ;
		}
		break;

	case SENSOR_ORIENTATION:
		if ((dSize == DATA_SIZE_32_BIT) &&
		    (dFormat == DATA_FORMAT_FIXPOINT) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
			/* Extract OrientationFixP data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.OrientFixP.Pitch = BYTES_TO_LONG_ARR(
				pHif->OrientationFixP.Pitch, 0);
			pOut->P.OrientFixP.Roll = BYTES_TO_LONG_ARR(
				pHif->OrientationFixP.Roll, 0);
			pOut->P.OrientFixP.Yaw = BYTES_TO_LONG_ARR(
				pHif->OrientationFixP.Yaw, 0);
			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember that HIF
				  packets are Big-Endian formatted */
				pOut->P.OrientFixP.TimeStamp.TS8[i] =
					pHif->OrientationFixP.TimeStamp[
						sizeof(uint64_t) - i-1];
			}
			errCode = 0;
			lengthParsed = ORIENTATION_FIXP_DATA_PKT_SZ;
		}
		break;

	case SENSOR_LINEAR_ACCELERATION:
	case SENSOR_GRAVITY:
		if ((dSize == DATA_SIZE_32_BIT) &&
		    (dFormat == DATA_FORMAT_FIXPOINT) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
			/* Extract ThreeAxisFixp data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.ThreeAxisFixP.Axis[0] = BYTES_TO_LONG_ARR(
				pHif->ThreeAxisFixp.Data, 0);

			pOut->P.ThreeAxisFixP.Axis[1] = BYTES_TO_LONG_ARR(
				pHif->ThreeAxisFixp.Data, 4);

			pOut->P.ThreeAxisFixP.Axis[2] = BYTES_TO_LONG_ARR(
				pHif->ThreeAxisFixp.Data, 8);
			//TODO: How to handle the Accuracy here ? Leave it in
			// the Metadeta ?
			/* Extract fixed point time stamp */
			for (i = 0; i < sizeof(uint64_t); i++) {
				/* Copy LSB to MSB data - remember that HIF
				  packets are Big-Endian formatted */
				pOut->P.ThreeAxisFixP.TimeStamp.TS8[i] =
					pHif->ThreeAxisFixp.TimeStamp[sizeof(uint64_t) - i-1];
			}
			errCode = 0;
			lengthParsed = THREEAXIS_FIXP_DATA_PKT_SZ;
		}
		break;

	default:
	{
		int i;
		unsigned char *p;
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

static int OSP_Sensor_enable(struct osp_data *osp, int sensor, int private)
{
	int retval = -EINVAL;

	if (private != 0 && !(osp->features & FEAT_PRIVATE)) return -EINVAL;

	if (sensor < 0x30)
		retval = i2c_smbus_read_byte_data(osp->client, 0x20+sensor);

	return retval;
}

static int OSP_Sensor_disable(struct osp_data *osp, int sensor, int private)
{
	int retval = -EINVAL;

	if (private != 0 && !(osp->features & FEAT_PRIVATE)) return -EINVAL;

	if (sensor < 0x30)
		retval = i2c_smbus_read_byte_data(osp->client, 0x50+sensor);

	return retval;
}

static int osp_i2c_read(u8 addr, u8 *data, int len)
{
#if 1
	struct i2c_msg msgs[] = {
		{
			.addr = gOSP->client->addr,
			.flags = gOSP->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = gOSP->client->addr,
			.flags = gOSP->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(gOSP->client->adapter, msgs, 2);
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

static int osp_i2c_write(u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = gOSP->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = gOSP->client->addr,
			.flags = 0,
			.len = len,
			.buf = data,
		},
	};
	return i2c_transfer(gOSP->client->adapter, msgs, 2);
}


static void osp_disable(struct osp_data *osp)
{
}

/* ------------ Call back management code -------------- */
static void OSP_CB_init(void)
{
	int i;

	for (i = 0; i < NUM_ANDROID_SENSOR_TYPE; i++) {
		and_sensor[i].dataready = NULL;
	}
	for (i = 0; i < NUM_PRIVATE_SENSOR_TYPE; i++) {
		prv_sensor[i].dataready = NULL;
	}
}

/* API Call: Register a callback for a sensor */
int OSP_Sensor_Register(int sensor, int private,
	void (*dataready)(int sensor, int prv,
		void *private, long long ts, union OSP_SensorData *sensordata),
	void *prv)
{
	struct OSP_Sensor *sen;

	if (private == 1)
		sen = prv_sensor;
	else if (private == 0)
		sen = and_sensor;
	else
		return -EINVAL;

	if (sen[sensor].dataready != NULL)
		return -EBUSY;

	sen[sensor].dataready = dataready;
	sen[sensor].private = prv;

	return 0;
}
EXPORT_SYMBOL_GPL(OSP_Sensor_Register);

/* API Call: Release a sensor */
int OSP_Sensor_UnRegister(int sensor, int private)
{
	struct OSP_Sensor *sen;

	if (private == 1)
		sen = prv_sensor;
	else if (private == 0)
		sen = and_sensor;
	else
		return -EINVAL;

	sen->dataready = NULL;

	return 0;
}

EXPORT_SYMBOL_GPL(OSP_Sensor_UnRegister);
/* API Call: Activates/suspends a sensor */
int OSP_Sensor_State(int sensor, int private, int state)
{
	if (private != 1 && private != 0)
		return -EINVAL;

	if (state == 1) {
		OSP_Sensor_enable(gOSP, sensor, private);
	} else {
		OSP_Sensor_disable(gOSP, sensor, private);
	}

	return 0;
}

EXPORT_SYMBOL_GPL(OSP_Sensor_State);

static void OSP_ReportSensor(struct osp_data *osp,
	SensorPacketTypes_t *spack)
{
	int PSensor;
	static int sig_counter = 0, step_counter = 0;
	union OSP_SensorData data;

	switch(spack->SType) {
	case SENSOR_STEP_DETECTOR:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.StepDetector.StepDetected;
		data.xyz.y = step_counter;
		step_counter++;
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.StepDetector.TimeStamp.TS64, &data);
		break;

	case SENSOR_SIGNIFICANT_MOTION:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.SigMotion.MotionDetected;
		data.xyz.y = sig_counter;
		sig_counter++;
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.SigMotion.TimeStamp.TS64, &data);
		break;

	case SENSOR_PRESSURE:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.CalFixP.Axis[0];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.CalFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_STEP_COUNTER:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = (uint32_t)spack->P.StepCount.NumStepsTotal;
		data.xyz.y = (uint32_t)(spack->P.StepCount.NumStepsTotal >> 32);
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.StepCount.TimeStamp.TS64, &data);
		break;

	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.UncalFixP.Axis[0];
		data.xyz.y = spack->P.UncalFixP.Axis[1];
		data.xyz.z = spack->P.UncalFixP.Axis[2];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.UncalFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.CalFixP.Axis[0];
		data.xyz.y = spack->P.CalFixP.Axis[1];
		data.xyz.z = spack->P.CalFixP.Axis[2];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.CalFixP.TimeStamp.TS64, &data);
		break;
	case SENSOR_LINEAR_ACCELERATION:
	case SENSOR_GRAVITY:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.ThreeAxisFixP.Axis[0];
		data.xyz.y = spack->P.ThreeAxisFixP.Axis[1];
		data.xyz.z = spack->P.ThreeAxisFixP.Axis[2];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.ThreeAxisFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_ORIENTATION:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.y = spack->P.OrientFixP.Pitch;
		data.xyz.z = spack->P.OrientFixP.Roll;
		data.xyz.x = spack->P.OrientFixP.Yaw;
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.OrientFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
		if (!and_sensor[spack->SType].dataready) break;
		data.quat.r = spack->P.QuatFixP.Quat[0];
		data.quat.x = spack->P.QuatFixP.Quat[1];
		data.quat.y = spack->P.QuatFixP.Quat[2];
		data.quat.z = spack->P.QuatFixP.Quat[3];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.QuatFixP.TimeStamp.TS64, &data);
		break;

	case PSENSOR_ACCELEROMETER_UNCALIBRATED|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!prv_sensor[PSensor].dataready) break;

		data.xyz.x = spack->P.UncalFixP.Axis[0];
		data.xyz.y = spack->P.UncalFixP.Axis[1];
		data.xyz.z = spack->P.UncalFixP.Axis[2];
		prv_sensor[PSensor].dataready(PSensor, 1,
			prv_sensor[PSensor].private,
			spack->P.UncalFixP.TimeStamp.TS64, &data);
		break;

	case PSENSOR_ACCELEROMETER_RAW|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!prv_sensor[PSensor].dataready) break;
		data.xyz.x = spack->P.RawSensor.Axis[0];
		data.xyz.y = spack->P.RawSensor.Axis[1];
		data.xyz.z = spack->P.RawSensor.Axis[2];
		prv_sensor[PSensor].dataready(PSensor, 1,
			prv_sensor[PSensor].private,
			spack->P.RawSensor.TStamp.TS64, &data);
		break;

	default:
		break;
	}
}

/* Attempt to grab 2 packets in a row to maximize through put.
 * I2C is a slow bus (can sleep during xfers) and may take longer then
 * the time it takes to create the data on the hub if we wait too long.
 */
static void osp_work_q(struct work_struct *work)
{
	struct osp_data *osp = container_of(work, struct osp_data, osp_work);
	SensorPacketTypes_t spack;
	unsigned char *pack_ptr;
	int pack_count;
	int ret;
	uint32_t intlen;
	int i;
	int gotpack[NUM_PACK] = {0,0};
	uint16_t plen[NUM_PACK] = {0,0};

	/* Grab buffers as quickly as possible */
	for (i = 0; i < NUM_PACK; i++) {
		ret = osp_i2c_read(OSP_INT_LEN,
			(unsigned char *)&intlen, 4);
		if ((ret >= 0) && (intlen&OSP_INT_DRDY)) {
			plen[i] = (intlen >> 4);
			if (plen[i] > 0 && plen[i] < 8192) {
				ret = osp_i2c_read(OSP_DATA_OUT,
					osp_pack[i], plen[i]);
				if (ret < 0) return;
				gotpack[i] = 1;
			}
		}
	}
	for (i = 0; i < NUM_PACK; i++) {
		if (gotpack[i]) {
			pack_count = 0;
			pack_ptr = osp_pack[i];
			do {
				ret = OSP_ParseSensorDataPkt(&spack, pack_ptr,
					plen[i]);
				if (ret>0) {
					OSP_ReportSensor(osp, &spack);
				} else {
					dev_err(
						&osp->client->dev,
						"OSP packet parsing error = %i, pack_count = %i plen = %i\n",
						ret, pack_count, plen[i]);
					break;
				}
				pr_debug("OSP Read data len %d, packet_count %d\n",
				plen[i], pack_count);
				plen[i] -= ret;
				pack_ptr += ret;
				pack_count++;
			} while (plen[i] > 0);
		}
	}
}

static void osp_poll_timer(unsigned long _osp)
{
	struct osp_data *osp = (void *)_osp;
	queue_work(osp_workq, &osp->osp_work);
	mod_timer(&osp->osp_timer, jiffies+1);
}

static irqreturn_t osp_irq_thread(int irq, void *dev)
{
	struct osp_data *osp = dev;
	pr_debug("%s:%d", __func__, __LINE__);

	queue_work(osp_workq, &osp->osp_work);
	return IRQ_HANDLED;
}

/* HY-DBG: Verify return code with latest firmwrae. */
static int osp_verify(struct osp_data *osp)
{
	int retval;

	pr_debug("%s:%d", __func__, __LINE__);
	retval = i2c_smbus_read_byte_data(osp->client, OSP_WHOAMI);
	if (retval < 0) {
		dev_err(&osp->client->dev,
			"read err int source, 0x%x\n", retval);
		goto out;
	}

	retval = i2c_smbus_read_byte_data(osp->client, OSP_CONFIG);

	retval = (retval != 0x54) ? -EIO : 0;
	retval = 0;
out:
	return retval;
}

static ssize_t sensorhub_config_read_request(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int param_id, sensor_id, seq_no;
	struct hif_data buff;
	memset(buff.buffer, 0, sizeof(buff.buffer));
	sscanf(buf, "0x%02x %d 0x%02x", &sensor_id, &seq_no, &param_id);
	pr_debug(" %s setconfig param = %d, sensor id = %d, seq no = %d\n",\
		__func__, param_id, sensor_id, seq_no);
	osp_read_packet(param_id, sensor_id, seq_no, &buff);
	pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
	/*write Hif read request packet to firmware
	mutex_lock(&gOSP->lock);
	osp_i2c_write(OSP_DATA_OUT, &buff.buffer, buff.size);
	mutex_unlock(&gOSP->lock);*/
	return count;
}
 static ssize_t sensorhub_config_write_request(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
 {
	int param_id, sensor_id, seq_no;
	struct hif_data buff;
	sscanf(buf, "0x%02x %d 0x%02x", &sensor_id, &seq_no, &param_id);
	gOSP->resp_data.param_id = param_id;
	gOSP->resp_data.sensor_id = sensor_id;
	gOSP->resp_data.seq_no = seq_no;
	memset(buff.buffer, 0 , sizeof(buff.buffer));
	/* parse remaining buffer depending upon setconfig parameters*/
	switch (param_id) {
		/*sensor enable 0x01*/
	case PARAM_ID_ENABLE:
	{
		u8 enable;
		sscanf(buf + HIF_PARAM_STR_LEN, "%c", &enable);
		enable = enable - '0';
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d,\
			enable = %d\n", __func__, param_id, sensor_id, seq_no,\
			enable);
		osp_sensor_enable(param_id, sensor_id, seq_no, enable, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_BATCH:
	{
		u64 data[2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%16llx 0x%16llx", &data[0], &data[1]);
		pr_debug("%s param_id = %d sensor id : 0x%02x, seq_no : %d,\
			data[0] = %llu, data[1] = %llu \n", __func__,\
			param_id, sensor_id, seq_no, data[0], data[1]);
		osp_set_batch(param_id, sensor_id, seq_no, data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_FLUSH:
	{
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d,\
				\n", __func__, param_id, sensor_id, seq_no);
		osp_set_flush(param_id, sensor_id, seq_no, &buff);
		pr_debug("%s param_id = %d sensor id : 0x%02x, seq_no : %d\n",\
			__func__, param_id, sensor_id, seq_no);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_AXIS_MAPPING:
	{
		s8 data[3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hhx 0x%hhx 0x%hhx",
			&data[0], &data[1], &data[2]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d,\
			data[0] = %hhd, data[1] = %hhd, data[2] = %hhd \n",\
			__func__, param_id, sensor_id, seq_no, data[0], data[1], data[2]);
		osp_set_axis_mapping(param_id, sensor_id, seq_no,
			data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_CONVERSION_OFFSET:
	{
		s32 data[3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x 0x%08x 0x%08x", &data[0], &data[1], &data[2]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d,\
			data[0] = %d, data[1] = %d, data[2] = %d \n",\
			 __func__, param_id, sensor_id, seq_no, data[0], data[1], data[2]);
		osp_set_conversion_offset(param_id, sensor_id, seq_no,
			data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_TIMESTAMP_OFFSET:
	case PARAM_ID_EXPECTED_NORM:
	{
		s32 data;
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x", &data);
		pr_debug("%s param id = %d sensor id : 0x%02x, seq_no : %d,\
				data = %d \n", __func__, param_id, sensor_id, seq_no, data);
		osp_set_time_offset_normal_mag(param_id, sensor_id,
		seq_no, data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_ONTIME_WAKETIME:
	{
		u32 data[2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x 0x%08x", &data[0], &data[1]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d, \
				data[0] = %d data[1] = %d \n", __func__,\
				param_id, sensor_id, seq_no, data[0], data[1]);
		osp_set_on_wake_time(param_id, sensor_id, seq_no, data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_HPF_LPF_CUTOFF:
	{
		u16 data[2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx", &data[0], &data[1]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d, \
				data[0] = %hu data[1] = %hu \n", __func__,\
				param_id, sensor_id, seq_no, data[0], data[1]);
		osp_set_hpf_lpf_cutoff(param_id, sensor_id, seq_no,
					data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_F_SKOR_MATRIX:
	{
		u32 data[3][3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x", \
			&data[0][0], &data[0][1], &data[0][2], &data[1][0], &data[1][1],\
			&data[1][2], &data[2][0], &data[2][1], &data[2][2]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d,\
				\n", __func__, param_id, sensor_id, seq_no);
		osp_set_skor_matrix(param_id, sensor_id, seq_no, data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_F_NONLINEAR_EFFECTS:
	{
		u16 data[4][3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx",\
			&data[0][0], &data[0][1], &data[0][2], &data[1][0],\
			&data[1][1], &data[1][2], &data[2][0], &data[2][1],\
			&data[2][2], &data[3][0], &data[3][1], &data[3][2]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d,\
				\n", __func__, param_id, sensor_id, seq_no);
		osp_set_non_linear_effects(param_id, sensor_id, seq_no,
		data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_TEMP_COEFF:
	{
		u16 data[3][2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx", &data[0][1],\
			&data[0][1], &data[1][0], &data[1][2],\
			&data[2][0], &data[2][1]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d,\
				\n", __func__, param_id, sensor_id, seq_no);
		osp_set_temp_coeff(param_id, sensor_id, seq_no, data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_SHAKE_SUSCEPTIBILIY:
	{
		u16 data[3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx 0x%hx", &data[0], &data[1], &data[2]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d, \
			data[0] = %hu data[1] = %hu data[2] = %hu\n", \
			__func__, param_id, sensor_id, seq_no, data[0],\
			data[1], data[2]);
		osp_set_shake_suscept(param_id, sensor_id, seq_no,
				data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_DYNAMIC_CAL_SCALE:
	case PARAM_ID_DYNAMIC_CAL_SKEW:
	case PARAM_ID_DYNAMIC_CAL_OFFSET:
	case PARAM_ID_DYNAMIC_CAL_ROTATION:
	case PARAM_ID_DYNAMIC_CAL_QUALITY:
	case PARAM_ID_CONVERSION_SCALE:
	case PARAM_ID_SENSOR_NOISE:
	case PARAM_ID_XYZ_OFFSET:
	case PARAM_ID_F_CAL_OFFSET:
	case PARAM_ID_BIAS_STABILITY:
	case PARAM_ID_REPEATABILITY:
	{
		u32 data[3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x 0x%08x 0x%08x", &data[0],\
		&data[1], &data[2]);
		pr_debug("%s param id : %d sensor id : 0x%02x, seq_no : %d, data[0] = 0x%08x,\
		data[1] = 0x%08x data[2] = 0x%08x\n", __func__, param_id, sensor_id,\
		seq_no, data[0], data[1], data[2]);
		osp_set_calib_param_fix32(param_id, sensor_id,
		seq_no, data, &buff);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	case PARAM_ID_DYNAMIC_CAL_SOURCE:
	{
		s8 data;
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hhx", &data);
		osp_set_dynamic_calib_source(param_id, sensor_id, seq_no, data, &buff);
		pr_debug("%s param id = %d sensor id : 0x%02x,\
			seq_no : %d data = %d\n",\
			__func__, param_id, sensor_id, seq_no, data);
		pr_debug("%s ::: hif data : 0x%04x \n", __func__, *(u32 *)(buff.buffer + SET_CONFIG_SZ));
		break;
	}
	default:
		pr_debug("%s wrong parameter entered for sending \
				write request packet\n", __func__);
		break;
	}
	/*write Hif write request packet to firmware
	mutex_lock(&gOSP->lock);
	osp_i2c_write(OSP_DATA_OUT, &buff.buffer, buff.size);
	mutex_unlock(&gOSP->lock);
	*/
	return count;
 }
 static ssize_t sensorhub_config_read_response(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	/*Need to add read response data*/
	return 0;
}
 static ssize_t sensorhub_config_write_response(struct device *dev,
					  struct device_attribute *attr, char *buf)
 {
	/*Need to add read response data*/
	return 0;
 }
#ifdef DUMMY_CONFIG_CMD
 static ssize_t sensorhub_dummy_config_read_request(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
 {
	int sensor_id, seq_no, param_id;
	u8 buffer[4];
	memset(buffer, 0, sizeof(buffer));
	memset(dummy_read_data, 0, sizeof(dummy_read_data));
	sscanf(buf, "0x%02x %d 0x%02x", &sensor_id, &seq_no, &param_id);
	pr_debug("%s : dummy param id : %d sensor id : %d seq no : %d\n",
		 __func__, param_id, sensor_id, seq_no);
	FormatSensorReadWriteReq(buffer, param_id, sensor_id, 0, seq_no, \
		 HIF_READ_CONTROL_REQ);
	memcpy(dummy_read_data, buffer, 4);
	pr_debug("formatted dummy data : 0x%08x dummy data : 0x%08x\n",
		cpu_to_be32(*(u32 *)buffer), *(u32 *)dummy_read_data);
	return count;
 }
 static ssize_t sensorhub_dummy_config_write_request(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
 {
	 int sensor_id, seq_no, param_id;
	 u8 buffer[4];
	 memset(dummy_write_data, 0, sizeof(dummy_write_data));
	 memset(buffer, 0, sizeof(buffer));
	 sscanf(buf, "0x%02x %d 0x%02x", &sensor_id, &seq_no, &param_id);
	 pr_debug("%s : dummy param id : %d sensor id : %d seq no : %d\n",
		 __func__, param_id, sensor_id, seq_no);
	 FormatSensorReadWriteReq(buffer, param_id, sensor_id, 0, seq_no, \
		 HIF_WRITE_CONTROL_REQ);
	 memcpy(dummy_write_data, buffer, 4);
	 pr_debug("formatted dummy data : 0x%08x dummy data : 0x%08x\n",
		 cpu_to_be32(*(u32 *)buffer), *(u32 *)dummy_write_data);
	 return count;
 }
 static ssize_t sensorhub_dummy_config_write_response(struct device *dev,
					  struct device_attribute *attr, char *buf)
 {
	int ret = 0;
	u8 param_id = M_ParseConfigParam(*(u8 *)(dummy_write_data + 3));
	u8 sensor_id = M_SensorType(*(u8 *)(dummy_write_data + 1));
	u8 seq_no = M_SequenceNum(*(u8 *)(dummy_write_data + 2));
	pr_debug("%s :", __func__);
	if (seq_no == 0) {
		pr_debug("%s , seq_no : %d, response suppressed\n", __func__,
		seq_no);
		return ret;
	 }
	ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d",
		sensor_id, seq_no, param_id, error_code);
	pr_debug("%s : %d\n", __func__, ret);
	return ret;
 }

int format_dummy_read_response(char *buf, u8 paramid, u8 sensorid, u8 seqno)
{
	int ret = 0;
	pr_debug("%s : response :\n", __func__);
	switch (paramid) {
	case PARAM_ID_ERROR_CODE_DATA:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d",
			sensorid, seqno, paramid, error_code);
		break;
	}
	case PARAM_ID_AXIS_MAPPING:
	{
		s8 data[3] = {0x0A, 0x0B, 0x0c};
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%02x 0x%02x 0x%02x", sensorid,
				seqno, paramid, error_code, data[0],
				data[1], data[2]);
		break;
	}
	case PARAM_ID_CONVERSION_OFFSET:
	{
		s32 data[3] = {0x01020304, 0x0A0B0C0D, 0xA2ABCDA0};
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x 0x%08x", sensorid, seqno,
				paramid, error_code, data[0], data[1], data[2]);
		break;
	}
	case PARAM_ID_DYNAMIC_CAL_SCALE:
	case PARAM_ID_DYNAMIC_CAL_SKEW:
	case PARAM_ID_DYNAMIC_CAL_OFFSET:
	case PARAM_ID_DYNAMIC_CAL_ROTATION:
	case PARAM_ID_DYNAMIC_CAL_QUALITY:
	case PARAM_ID_CONVERSION_SCALE:
	case PARAM_ID_SENSOR_NOISE:
	case PARAM_ID_XYZ_OFFSET:
	case PARAM_ID_F_CAL_OFFSET:
	case PARAM_ID_BIAS_STABILITY:
	case PARAM_ID_REPEATABILITY:
	{
		u32 data[3] = {0x01020304, 0x0A0B0C0D, 0xA2ABCDA0};
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x 0x%08x", sensorid, seqno,
				paramid, error_code, data[0], data[1], data[2]);
	}
	case PARAM_ID_TIMESTAMP_OFFSET:
	case PARAM_ID_EXPECTED_NORM:
	case PARAM_ID_POWER:
	{
		u32 data = 0x01020304;
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d 0x%08x",
				sensorid, seqno, paramid, error_code, data);
		break;
	}
	case PARAM_ID_MINMAX_DELAY:
	{
		s32 data[2] = {0x0A0B0C0D, 0x2AF08000};
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x", sensorid, seqno, paramid,
				error_code, data[0], data[1]);
		break;
	}
	case PARAM_ID_ONTIME_WAKETIME:
	case PARAM_ID_RANGE_RESOLUTION:
	case PARAM_ID_FIFO_EVT_CNT:
	{
		u32 data[2] = {0x0A0B0C0D, 0x2AF08000};
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x", sensorid, seqno, paramid,
				error_code, data[0], data[1]);
		break;
	}
	case PARAM_ID_HPF_LPF_CUTOFF:
	{
		u16 data[2] = {0x0A0B, 0x2AF0};
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%04x 0x%04x", sensorid, seqno , paramid,
				error_code, data[0], data[1]);
		break;
	}
	case PARAM_ID_SENSOR_NAME:
	{
		char SENSOR_NAME[32] = "Accelerometer";
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d %s",
			sensorid, seqno, paramid, error_code, SENSOR_NAME);
		break;
	}
	case PARAM_ID_VERISON:
	{
		char motionq_version[32] = "version1";
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d %s",
			sensorid, seqno, paramid, error_code, motionq_version);
		break;
	}
	case PARAM_ID_F_SKOR_MATRIX:
	{
		u32 data[3][3] = {{0x0A0B020D, 0x12430808, 0x20FA8000},
				{0x20FA8000, 0x12430808, 0x0A0B020D},
				{0x0A0B020D, 0x20FA8000, 0x20FA8000} };
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
			"0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x "
			"0x%08x 0x%08x", sensorid, seqno, paramid, error_code,
			data[0][0], data[0][1], data[0][2], data[1][0],
			data[1][1], data[1][2], data[2][0], data[2][1],
			data[2][2]);
		break;
	}
	case PARAM_ID_F_NONLINEAR_EFFECTS:
	{
		u16 data[4][3] = {{0x0A0B, 0x0B0C, 0x0102},
				{0x0203, 0x0123, 0x0121},
				{0xA0B0, 0x1020, 0xF080},
				{0x8000, 0xF010, 0x0B0C} };
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
			"0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x "
			"0x%04x 0x%04x 0x%04x 0x%04x 0x%04x", sensorid, seqno,
			paramid, error_code, data[0][0],
			data[0][1], data[0][2], data[1][0], data[1][1],
			data[1][2], data[2][0], data[2][1], data[2][2],
			data[3][0], data[3][1], data[3][2]);
		break;
	}
	case PARAM_ID_TEMP_COEFF:
	{
		u16 data[3][2] = {{0x0A0B, 0x0C0D},
				{0x0201, 0x0302},
				{0x0201, 0x03} };
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
			"0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x",
			sensorid, seqno, paramid, error_code, data[0][0],
			data[0][1], data[1][0], data[1][1], data[2][0],
			data[2][1]);
		break;
	}
	case PARAM_ID_SHAKE_SUSCEPTIBILIY:
	{
		u16 data[3] = {0x0A0B, 0x0102, 0x0406};
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
			"0x%04x 0x%04x 0x%04x", sensorid, seqno, paramid,
			error_code, data[0], data[1], data[2]);
		break;
	}
	case PARAM_ID_DYNAMIC_CAL_SOURCE:
	{
		s8 data = 0x1A;
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d 0x%02x",
			sensorid, seqno, paramid, error_code, data);
		break;
	}
	default:
	{
		pr_debug("Setconfig parameter is write only\n");
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "%d %d %d %d",
		sensorid, seqno, paramid, error_code);
		break;
	}
	}
	return ret;
}
 static ssize_t sensorhub_dummy_config_read_response(struct device *dev,
					  struct device_attribute *attr, char *buf)
 {
	int ret = 0;
	u8 param_id = M_ParseConfigParam(*(u8 *)(dummy_read_data + 3));
	u8 sensor_id = M_SensorType(*(u8 *)(dummy_read_data + 1));
	u8 seq_no = M_SequenceNum(*(u8 *)(dummy_read_data + 2));
	pr_debug("%s :", __func__);
	if (seq_no == 0) {
		pr_debug("%s , seq_no : %d, response suppressed\n", __func__,
		seq_no);
		return ret;
	}
	ret = format_dummy_read_response(buf, param_id, sensor_id, seq_no);
	pr_debug("%s : %d\n", __func__, ret);
	return ret;
 }

static DEVICE_ATTR(sensorhub_config_read, 0666, sensorhub_config_read_response,
		sensorhub_config_read_request);
static DEVICE_ATTR(sensorhub_config_write, 0666, sensorhub_config_write_response,
		sensorhub_config_write_request);
static DEVICE_ATTR(sensorhub_dummy_config_read, 0666,
		sensorhub_dummy_config_read_response,
		sensorhub_dummy_config_read_request);
static DEVICE_ATTR(sensorhub_dummy_config_write, 0666,
		sensorhub_dummy_config_write_response,
		sensorhub_dummy_config_write_request);
#endif

static struct attribute *core_sysfs_attrs[] = {
	&dev_attr_sensorhub_config_write.attr,
	&dev_attr_sensorhub_config_read.attr,
#ifdef DUMMY_CONFIG_CMD
	&dev_attr_sensorhub_dummy_config_write.attr,
	&dev_attr_sensorhub_dummy_config_read.attr,
#endif
	NULL
};

static struct attribute_group core_sysfs = {
	.attrs = core_sysfs_attrs
};

static int OSP_add_child(struct osp_data *osp)
{
	struct platform_device *pdev;
	int err;
	pdev = platform_device_alloc("osp-output", 0);
	if (!pdev) {
		printk("Cannot allocate dev\n");
		return -ENOMEM;
	}
	pdev->dev.parent = &osp->client->dev;
	err =  platform_device_add(pdev);
	if (!err) {
		err = sysfs_create_group(&pdev->dev.kobj, &core_sysfs);
		if (err) {
			pr_debug("%s not able to create sysfs %d \n", __func__, err);
		}
	}
	else
		pr_debug("%s failed to register platform device\n", __func__ );
	return err;
}

static int osp_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct osp_data *osp;
	int err;
	int i;

	pr_debug("%s:%d", __func__, __LINE__);
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
	mutex_init(&osp->lock);
	for (i = 0; i < NUM_PACK; i++) {
		osp_pack[i] = kzalloc(8192, GFP_KERNEL);
		if (!osp_pack[i]) {
			dev_err(&client->dev,
				"failed to allocate memory for packet data\n");
			i--;
			for (; i >= 0; i--) {
				kfree(osp_pack[i]);
			}
			return -ENOMEM;
		}
	}

	osp->client = client;

	err = osp_verify(osp);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_free_mem;
	}
	pr_debug("%s:%d", __func__, __LINE__);

	i2c_set_clientdata(client, osp);

	osp_workq = create_workqueue("osp_queue");
	if (osp_workq) {
		INIT_WORK(&osp->osp_work, osp_work_q);
	}

	client->irq = -1;	/* Force it for now */

	if (client->irq > 0) {
		err = request_threaded_irq(client->irq, NULL, osp_irq_thread, IRQF_TRIGGER_LOW, "osp-sh", osp);
		if (err < 0) {
			dev_err(&client->dev,
				"irq request failed %d, error %d\n",
				client->irq, err);
		}

	}

	if (client->irq <= 0 || err < 0) {
		setup_timer(&osp->osp_timer, osp_poll_timer, (unsigned long)osp);
		mod_timer(&osp->osp_timer, jiffies + msecs_to_jiffies(10));
	}
	pr_debug("%s:%d", __func__, __LINE__);

	OSP_CB_init();
	/* Create child device */
	OSP_add_child(osp);
	pr_debug("%s:%d", __func__, __LINE__);

	return 0;
err_free_mem:
	kfree(osp);
	return err;
}

static int osp_remove(struct i2c_client *client)
{
	struct osp_data *osp = i2c_get_clientdata(client);

	if (client->irq) {
		free_irq(client->irq, osp);
	}

	kfree(osp);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int osp_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct osp_data *osp = i2c_get_clientdata(client);

	mutex_lock(&osp->lock);
	osp_disable(osp);
	mutex_unlock(&osp->lock);
	return 0;
}

static int osp_resume(struct device *dev)
{
	return 0;
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
	pr_debug("%s:%d", __func__, __LINE__);
	return i2c_add_driver(&osp_driver);
}
module_init(osp_init);
#endif


MODULE_DESCRIPTION("OSP driver");
MODULE_AUTHOR("Hunyue Yau <hy-git@hy-research.com>");
MODULE_LICENSE("GPL");
