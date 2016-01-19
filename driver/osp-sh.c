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
#include <linux/gpio.h>
#define NAME			"ospsh"
/* Private sensors */
#define FEAT_PRIVATE	(1<<0)
#define MAX_BUF_SZ 		20
#define MAX_VERSION_LEN	64

/*static struct completion hif_response_complete;*/
static u8 sdata[PAGE_SIZE];
static u16 error_code;
static u8 rxbuf[MAX_BUF_SZ];
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
	u8 version[MAX_VERSION_LEN];
	bool setv;
	u8 isFlushcompleted;
};

static struct osp_data *gOSP;
static struct work_queue *osp_workq;
/* Number of packets to parse */
#define NUM_PACK	2
static unsigned char *osp_pack[NUM_PACK];
#define u8_to_be64(byte, val) \
	for(i = sizeof(u64) -1; i >= 0; i--) { \
		val = (val << 8) + byte[i];};

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
		    pOut->P.CalFixP.TimeStamp.TS64 = 0;
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
			u8_to_be64(pHif->CalPktFixP.TimeStamp, pOut->P.CalFixP.TimeStamp.TS64);
			pOut->P.CalFixP.TimeStamp.TS64 = be64_to_cpu(pOut->P.CalFixP.TimeStamp.TS64);
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
		    pOut->P.QuatFixP.TimeStamp.TS64 = 0;
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
			u8_to_be64(pHif->QuatPktFixP.TimeStamp , pOut->P.QuatFixP.TimeStamp.TS64);
			pOut->P.QuatFixP.TimeStamp.TS64 = be64_to_cpu(pOut->P.QuatFixP.TimeStamp.TS64);
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
		    pOut->P.UncalFixP.TimeStamp.TS64 = 0;
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
			u8_to_be64(pHif->UncalPktFixP.TimeStamp , pOut->P.UncalFixP.TimeStamp.TS64);
			pOut->P.UncalFixP.TimeStamp.TS64 = be64_to_cpu(pOut->P.UncalFixP.TimeStamp.TS64);
			errCode = 0;
		}
		break;

	case SENSOR_SIGNIFICANT_MOTION:
		if ((dSize == DATA_SIZE_8_BIT) &&
		    (dFormat == DATA_FORMAT_RAW) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
		    pOut->P.SigMotion.TimeStamp.TS64 = 0;
			/* Extract SignificantMotion data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.SigMotion.MotionDetected =
				pHif->SignificantMotion.significantMotionDetected;
			/* Extract fixed point time stamp */
			u8_to_be64(pHif->SignificantMotion.TimeStamp , pOut->P.UncalFixP.TimeStamp.TS64);
			pOut->P.SigMotion.TimeStamp.TS64 = be64_to_cpu(pOut->P.SigMotion.TimeStamp.TS64);
			errCode = 0;
			lengthParsed = SIGNIFICANTMOTION_FIXP_DATA_PKT_SZ;
		}
		break;

	case SENSOR_STEP_DETECTOR:
		if ((dSize == DATA_SIZE_8_BIT) &&
		    (dFormat == DATA_FORMAT_RAW) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
		    pOut->P.StepDetector.TimeStamp.TS64 = 0;
			/* Extract StepDetector data from packet */
			pOut->SType = (ASensorType_t)sensType;
			pOut->SubType = SENSOR_SUBTYPE_UNUSED;
			pOut->P.StepDetector.StepDetected =
				pHif->StepDetector.stepDetected;

			/* Extract fixed point time stamp */
			u8_to_be64(pHif->StepDetector.TimeStamp , pOut->P.StepDetector.TimeStamp.TS64);
			pOut->P.StepDetector.TimeStamp.TS64 = be64_to_cpu(pOut->P.StepDetector.TimeStamp.TS64);
			errCode = 0;
			lengthParsed = STEPDETECTOR_DATA_PKT_SZ;
		}
		break;

	case SENSOR_STEP_COUNTER:
		if ((dSize == DATA_SIZE_64_BIT) &&
		    (dFormat == DATA_FORMAT_RAW) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
		    pOut->P.StepCount.TimeStamp.TS64 = 0;
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
			u8_to_be64(pHif->StepCounter.TimeStamp, pOut->P.StepCount.TimeStamp.TS64);
			pOut->P.StepCount.TimeStamp.TS64 = be64_to_cpu(pOut->P.StepCount.TimeStamp.TS64);
			errCode = 0;
			lengthParsed = STEPCOUNTER_DATA_PKT_SZ;
		}
		break;

	case SENSOR_ORIENTATION:
		if ((dSize == DATA_SIZE_32_BIT) &&
		    (dFormat == DATA_FORMAT_FIXPOINT) &&
		    (timeFormat == TIME_FORMAT_FIXPOINT) &&
		    (tSize == TIME_STAMP_64_BIT)) {
		    pOut->P.OrientFixP.TimeStamp.TS64 = 0;
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
			u8_to_be64(pHif->OrientationFixP.TimeStamp, pOut->P.OrientFixP.TimeStamp.TS64);
			pOut->P.OrientFixP.TimeStamp.TS64 = be64_to_cpu(pOut->P.OrientFixP.TimeStamp.TS64);
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
		    pOut->P.ThreeAxisFixP.TimeStamp.TS64 = 0;
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
			u8_to_be64(pHif->ThreeAxisFixp.TimeStamp, pOut->P.ThreeAxisFixP.TimeStamp.TS64);
			pOut->P.ThreeAxisFixP.TimeStamp.TS64 = be64_to_cpu(pOut->P.ThreeAxisFixP.TimeStamp.TS64);
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
	gOSP->isFlushcompleted = 0;
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
	gOSP->isFlushcompleted = M_FLUSH_COMPLETED(pHif->SensPktRaw.Q.AttributeByte);
	if(gOSP->isFlushcompleted)
		pr_debug("%s :: %d Flush completed %d for sensor : %d\n",
				__func__, __LINE__, gOSP->isFlushcompleted, sensType);

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

static int osp_i2c_write(u8 reg_addr, u8 *data, int len)
{
	u8 message[len+1];
	struct i2c_msg msgs[] = {
		{
			.addr = gOSP->client->addr,
			.flags = gOSP->client->flags,
			.len = len + 1,
			.buf = message,
		},
	};
	message[0] = reg_addr;
	memcpy(message + 1, data, len);
	return i2c_transfer(gOSP->client->adapter, msgs, 1);
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
	int param_id = 0x01, ret;
	struct hif_data buff;
	memset(buff.buffer, 0, sizeof(buff.buffer));
	if (private != 1 && private != 0)
		return -EINVAL;
	pr_debug("%s sensor : %d state : %d \n",__func__, sensor, state);
	osp_sensor_enable(param_id, sensor, 0, state, &buff);
	mutex_lock(&gOSP->lock);
	ret = osp_i2c_write(OSP_SET_CONFIG, buff.buffer, buff.size);
	mutex_unlock(&gOSP->lock);
	return 0;
}

EXPORT_SYMBOL_GPL(OSP_Sensor_State);

static void OSP_ReportSensor(struct osp_data *osp,
	SensorPacketTypes_t *spack)
{
	int PSensor;
	static int sig_counter = 0, step_counter = 0;
	union OSP_SensorData data;
	memset(&data, 0, sizeof(data));
	pr_debug("%s ::: sensor type : %d \n", __func__, spack->SType);
	switch(spack->SType) {
	case SENSOR_STEP_DETECTOR:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.StepDetector.StepDetected;
			data.xyz.y = step_counter;
			data.xyz.ts = spack->P.StepDetector.TimeStamp.TS64;
		}
		step_counter++;
		pr_debug("%s:: step detector data.x = %d, data.y : %d\n", __func__,
			data.xyz.x, data.xyz.y);
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.StepDetector.TimeStamp.TS64, &data);
		break;

	case SENSOR_SIGNIFICANT_MOTION:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.SigMotion.MotionDetected;
			data.xyz.y = sig_counter;
			data.xyz.ts = spack->P.SigMotion.TimeStamp.TS64;
		}
		sig_counter++;
		pr_debug("%s:: significant motion data.x = %d, data.y : %d\n", __func__,
			data.xyz.x, data.xyz.y);
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.SigMotion.TimeStamp.TS64, &data);
		break;

	case SENSOR_PRESSURE:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.CalFixP.Axis[0];
			data.xyz.ts = spack->P.CalFixP.TimeStamp.TS64;
		}
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.CalFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_STEP_COUNTER:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = (uint32_t)spack->P.StepCount.NumStepsTotal;
			data.xyz.y = (uint32_t)(spack->P.StepCount.NumStepsTotal >> 32);
			data.xyz.ts = spack->P.StepCount.TimeStamp.TS64;
		}
		pr_debug("%s:: step counter data.x = %d, data.y : %d\n", __func__,
				data.xyz.x, data.xyz.y);
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.StepCount.TimeStamp.TS64, &data);
		break;

	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.UncalFixP.Axis[0];
			data.xyz.y = spack->P.UncalFixP.Axis[1];
			data.xyz.z = spack->P.UncalFixP.Axis[2];
			data.xyz.ts = spack->P.UncalFixP.TimeStamp.TS64;
		}
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.UncalFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.CalFixP.Axis[0];
			data.xyz.y = spack->P.CalFixP.Axis[1];
			data.xyz.z = spack->P.CalFixP.Axis[2];
			data.xyz.ts = spack->P.CalFixP.TimeStamp.TS64 ;
		}
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.CalFixP.TimeStamp.TS64, &data);
		break;
	case SENSOR_LINEAR_ACCELERATION:
	case SENSOR_GRAVITY:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.ThreeAxisFixP.Axis[0];
			data.xyz.y = spack->P.ThreeAxisFixP.Axis[1];
			data.xyz.z = spack->P.ThreeAxisFixP.Axis[2];
			data.xyz.ts = spack->P.ThreeAxisFixP.TimeStamp.TS64;
		}
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.ThreeAxisFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_ORIENTATION:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.y = spack->P.OrientFixP.Pitch;
			data.xyz.z = spack->P.OrientFixP.Roll;
			data.xyz.x = spack->P.OrientFixP.Yaw;
			data.xyz.ts = spack->P.OrientFixP.TimeStamp.TS64;
		}
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.OrientFixP.TimeStamp.TS64, &data);
		break;

	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
		if (!and_sensor[spack->SType].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.quat.r = spack->P.QuatFixP.Quat[0];
			data.quat.x = spack->P.QuatFixP.Quat[1];
			data.quat.y = spack->P.QuatFixP.Quat[2];
			data.quat.z = spack->P.QuatFixP.Quat[3];
			data.quat.ts = spack->P.QuatFixP.TimeStamp.TS64;
		}
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private,
			spack->P.QuatFixP.TimeStamp.TS64, &data);
		break;

	case PSENSOR_ACCELEROMETER_UNCALIBRATED|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!prv_sensor[PSensor].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.UncalFixP.Axis[0];
			data.xyz.y = spack->P.UncalFixP.Axis[1];
			data.xyz.z = spack->P.UncalFixP.Axis[2];
		}
		prv_sensor[PSensor].dataready(PSensor, 1,
			prv_sensor[PSensor].private,
			spack->P.UncalFixP.TimeStamp.TS64, &data);
		break;

	case PSENSOR_ACCELEROMETER_RAW|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!prv_sensor[PSensor].dataready) break;
		if (!(gOSP->isFlushcompleted)) {
			data.xyz.x = spack->P.RawSensor.Axis[0];
			data.xyz.y = spack->P.RawSensor.Axis[1];
			data.xyz.z = spack->P.RawSensor.Axis[2];
		}
		prv_sensor[PSensor].dataready(PSensor, 1,
			prv_sensor[PSensor].private,
			spack->P.RawSensor.TStamp.TS64, &data);
		break;

	default:
		break;
	}
}

int process_sensor_data(u8 *buf, int len)
{
	int err;
	u8 *pack_ptr;
	struct _SensorPacketTypes spack;
	int pack_count = 0;
	int plen = len;
	pack_ptr = buf;
	pr_debug("%s %i\n", __func__, __LINE__);
	do {
		err = OSP_ParseSensorDataPkt(&spack, pack_ptr, plen);
		if (err > 0) {
			OSP_ReportSensor(gOSP, &spack);
		} else {
			pr_debug("OSP packet parsing error = %d,\
				pack_count = %d, plen = %d \n", err,\
				pack_count + 1, plen);
			break;
		}
		plen -= err;
		pack_ptr += err;
		pack_count++;
	} while (plen > 0);
	pr_debug("OSP packet count : %d \n", pack_count);
	return 0;
}

/* Attempt to grab 2 packets in a row to maximize through put.
 * I2C is a slow bus (can sleep during xfers) and may take longer then
 * the time it takes to create the data on the hub if we wait too long.
 */
static void osp_work_q(struct work_struct *work)
{
	u8 cause = 0;
	u16 size;
	int ret = 0;
	mutex_lock(&gOSP->lock);
read_again:
	cause = 0;
	size = 0;
	error_code = 0;
	memset(rxbuf, 0, sizeof(rxbuf));
	memset(sdata, 0, sizeof(sdata));
	/*Read Get cause value */
	ret = osp_i2c_read(OSP_GET_CAUSE, rxbuf, sizeof(u8)*3);
	cause = *((u8 *)rxbuf);
	pr_debug("%s ::: cause : 0x%02x\n", __func__, cause);
	switch (cause) {
	case RESERVED:
	/* TBD: reserved for some purpose */
		break;
	case SENSOR_DATA_READY:
	{
		size = be16_to_cpu(*(u16 *)(rxbuf + sizeof(u8)));
		pr_debug("%s: cause= 0x%02x  size : 0x%04x \n", __func__, cause, size);
		/* Read Sensor data */
		if(size > 0 && size < PAGE_SIZE) {
			ret = osp_i2c_read(OSP_GET_DATA, sdata, size);
			/* Process sensor data and send it to userspace */
			process_sensor_data((u8 *)sdata, size);
		}
		break;
	}
	case MAG_CALIB_DT_UPDATE:
	/* Send notification to userspace */
		break;
	case ACCEL_CALIB_DT_UPDATE:
	/* Send notification to userspace */
		break;
	case GYRO_CALIB_DT_UPDATE:
	/* Send notification to userspace */
		break;
	case TEMP_CHANGE_DETECT:
	/* TBD: Need to check with firmware on what actions to take */
		break;
	case CONFIG_CMD_RESP:
	/*Implement Config Command Response*/
	{
		error_code = *(u8 *)(rxbuf + sizeof(u8));
		size = *(u8 *)(rxbuf + sizeof(u8)*2);
		pr_debug("%s: cause= 0x%02x error code = 0x%02x size = : 0x%02x \n",\
			__func__, cause, error_code, size);

		switch(error_code){
		case 0: /* success*/
		{
			if(size > 0){ /*response for sensor read control request packet*/
				ret = osp_i2c_read(OSP_GET_DATA, sdata, size);
				pr_debug("sdata packet :: 0x%08x \n", cpu_to_be32(*(u32 *)(sdata)));
				/* wait_for_completion(&hif_response_complete); */
			}
			break;
		}
		default : /*other failure code*/
			break;
		}
		break;
	}
	case UNRECOVER_ERR_MOTIONQ:
	/* TBD */
		break;
	default:
		break;
	}
	if (gpio_get_value(MPU_GYRO_IRQ_GPIO))
		goto read_again;
	mutex_unlock(&gOSP->lock);
}

static irqreturn_t osp_irq_thread(int irq, void *dev)
{
	struct osp_data *osp = dev;
	pr_debug("%s\n", __func__);
	queue_work(osp_workq, &osp->osp_work);
	return IRQ_HANDLED;
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
		pr_debug("%s param_id : %d, sensor id : %d, seq : %d\n", __func__,
				param_id, sensor_id, seq_no);
		osp_read_packet(param_id, sensor_id, seq_no, &buff);
		/*write Hif read request packet to firmware */
		mutex_lock(&gOSP->lock);
		osp_i2c_write(OSP_SET_CONFIG, buff.buffer, buff.size);
		mutex_unlock(&gOSP->lock);
		return count;
}

static ssize_t sensorhub_config_write_request(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct hif_data buff;
	bool IsParamIDCorrect;
	int ret;
	memset(buff.buffer, 0, sizeof(buff.buffer));
	buff.size = 0;
	IsParamIDCorrect = format_config_write_request(buf, &buff);
	if(!IsParamIDCorrect){
		pr_debug("%s:: param_id incorrect \n", __func__);
		return count;
	}
	pr_debug("%s :: cmd : 0x%04x HIF Packet : 0x%08x \n", __func__,
	*(u16 *)(buff.buffer), *(u32 *)(buff.buffer + 2));
	/*write Hif write request packet to firmware */
	mutex_lock(&gOSP->lock);
	ret = osp_i2c_write(OSP_SET_CONFIG, buff.buffer, buff.size);
	mutex_unlock(&gOSP->lock);
	return count;
}

int format_read_response(char *buf, u8 paramid, u8 sensorid, u8 seqno)
{
	int ret = 0;
	pr_debug("%s : response :\n", __func__);
	switch (paramid) {
	case PARAM_ID_ERROR_CODE_DATA:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d", \
				sensorid, seqno, paramid, error_code);
		break;
	}
	case PARAM_ID_AXIS_MAPPING:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%02x 0x%02x 0x%02x", sensorid, seqno, paramid,
				error_code, *(s8 *)(sdata + 4), *(s8 *)(sdata + 5),
				*(s8 *)(sdata + 5));
		break;
	}
	case PARAM_ID_CONVERSION_OFFSET:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d 0x%08x 0x%08x"
				"0x%08x", sensorid, seqno, paramid, error_code, *(s32 *)(sdata + 4),
				*(s32 *)(sdata + 8), *(s32 *)(sdata + 12));
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
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x 0x%08x", sensorid, seqno, paramid, error_code,
				*(u32 *)(sdata + 4), *(u32 *)(sdata + 8), *(u32 *)(sdata + 12));
		break;
	}
	case PARAM_ID_TIMESTAMP_OFFSET:
	case PARAM_ID_EXPECTED_NORM:
	case PARAM_ID_POWER:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d 0x%08x",
				sensorid, seqno, paramid, error_code, *(u32 *)(sdata + 4));
		break;
	}
	case PARAM_ID_MINMAX_DELAY:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x", sensorid, seqno, paramid,
				error_code, *(s32 *)(sdata + 4), *(s32 *)(sdata + 8));
		break;
	}
	case PARAM_ID_ONTIME_WAKETIME:
	case PARAM_ID_RANGE_RESOLUTION:
	case PARAM_ID_FIFO_EVT_CNT:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x", sensorid, seqno, paramid,
				error_code, *(u32 *)(sdata + 4), *(u32 *)(sdata + 8));
		break;
	}
	case PARAM_ID_HPF_LPF_CUTOFF:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%04x 0x%04x", sensorid, seqno , paramid, error_code,
				*(u16 *)(sdata + 4), *(u16 *)(sdata + 6));
		break;
	}
	case PARAM_ID_SENSOR_NAME:
	case PARAM_ID_VERISON:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d %s",
				sensorid, seqno, paramid, error_code, (sdata + 4));
		break;
	}
	case PARAM_ID_F_SKOR_MATRIX:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x",
				sensorid, seqno, paramid, error_code, *(u32 *)(sdata + 4),
				*(u32 *)(sdata + 8), *(u32 *)(sdata + 12), *(u32 *)(sdata + 16),
				*(u32 *)(sdata + 20), *(u32 *)(sdata + 24), *(u32 *)(sdata + 28),
				*(u32 *)(sdata + 32), *(u32 *)(sdata + 36));
		break;
	}
	case PARAM_ID_F_NONLINEAR_EFFECTS:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x "
				"0x%04x 0x%04x 0x%04x 0x%04x 0x%04x", sensorid, seqno, paramid,
				error_code, *(u16 *)(sdata + 4), *(u16 *)(sdata + 6),
				*(u16 *)(sdata + 8), *(u16 *)(sdata + 10), *(u16 *)(sdata + 12),
				*(u16 *)(sdata + 14), *(u16 *)(sdata + 16), *(u16 *)(sdata + 18),
				*(u16 *)(sdata + 20), *(u16 *)(sdata + 22), *(u16 *)(sdata + 24),
				*(u16 *)(sdata + 26));
		break;
	}
	case PARAM_ID_TEMP_COEFF:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x",
				sensorid, seqno, paramid, error_code, *(u16 *)(sdata + 4),
				*(u16 *)(sdata + 6), *(u16 *)(sdata + 8), *(u16 *)(sdata + 10),
				*(u16 *)(sdata + 12), *(u16 *)(sdata + 14));
		break;
	}
	case PARAM_ID_SHAKE_SUSCEPTIBILIY:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d "
				"0x%04x 0x%04x 0x%04x", sensorid, seqno, paramid, error_code,
				*(u16 *)(sdata + 4), *(u16 *)(sdata + 6), *(u16 *)(sdata + 8));
		break;
	}
	case PARAM_ID_DYNAMIC_CAL_SOURCE:
	{
		ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d 0x%02x",
				sensorid, seqno, paramid, error_code, *(s8 *)(sdata + 4));
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

static ssize_t sensorhub_config_read_response(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 param_id, sensor_id, seq_no;
	/* complete(&hif_response_complete); */
	pr_debug("%s ::\n", __func__);
	param_id = M_ParseConfigParam(*(u8 *)(sdata + 3));
	sensor_id = M_SensorType(*(u8 *)(sdata + 1));
	seq_no = M_SequenceNum(*(u8 *)(sdata + 2));
	if (seq_no == 0) {
		pr_debug("%s , seq_no : %d, response suppressed\n", __func__, seq_no);
		return ret;
	}
	ret = format_read_response(buf, param_id, sensor_id, seq_no);
	return ret;
}

static ssize_t sensorhub_config_write_response(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 param_id, sensor_id, seq_no;
	/* complete(&hif_response_complete); */
	pr_debug("%s ::\n", __func__);
	param_id = M_ParseConfigParam(*(u8 *)(sdata + 3));
	sensor_id = M_SensorType(*(u8 *)(sdata + 1));
	seq_no = M_SequenceNum(*(u8 *)(sdata + 2));
	if (seq_no == 0) {
		pr_debug("%s , seq_no : %d, response suppressed\n", __func__, seq_no);
		return ret;
	}

	ret = snprintf(buf, HIF_RESP_DATA_SZ, "0x%02x %d 0x%02x %d",
		sensor_id, seq_no, param_id, error_code);
	return ret;
}

static ssize_t sensorhub_reset_response(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int ret;
	u8 reset_data = 1;
	struct hif_data buff;
	memset(buff.buffer, 0 , sizeof(buff.buffer));
	ret = i2c_smbus_write_byte_data(gOSP->client, OSP_RESET_REG, 1);
	pr_debug("%s :: %d osp reset done ret ::%d \n", __func__, __LINE__, ret);
	/* wait for sensorhub to initialize before sending config command*/
	msleep(300);
	osp_set_config_done(PARAM_ID_CONFIG_DONE, 0, 0, &buff);
	ret = osp_i2c_write(OSP_SET_CONFIG, buff.buffer, buff.size);
	pr_debug("%s :: %d config done command sent \n", __func__, __LINE__, ret);
	return snprintf(buf, PAGE_SIZE, "sensorhub reset done\n");
}

static ssize_t sensorhub_osp_versionr(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int ret;
	return snprintf(buf, MAX_VERSION_LEN, gOSP->version);
}

static ssize_t sensorhub_osp_versionw(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	if(!gOSP || gOSP->setv) {
		pr_err("OSP version can't be set\n");
	}
	else {
		sprintf(gOSP->version, "%s", buf);
		gOSP->setv = true;
	}
	return count;
}

static DEVICE_ATTR(sensorhub_config_read, 0666, sensorhub_config_read_response,
		sensorhub_config_read_request);
static DEVICE_ATTR(sensorhub_config_write, 0666, sensorhub_config_write_response,
		sensorhub_config_write_request);
static DEVICE_ATTR(sensorhub_reset, 0666, sensorhub_reset_response,
		NULL);
static DEVICE_ATTR(osp_version, 0666, sensorhub_osp_versionr,
		sensorhub_osp_versionw);

static struct attribute *core_sysfs_attrs[] = {
	&dev_attr_sensorhub_config_write.attr,
	&dev_attr_sensorhub_config_read.attr,
	&dev_attr_sensorhub_reset.attr,
	&dev_attr_osp_version.attr,
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
		pr_debug("Cannot allocate dev\n");
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
	i2c_set_clientdata(client, osp);
	/*reset sensorhub chip*/
	err = i2c_smbus_write_byte_data(osp->client, OSP_RESET_REG, 1);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized err : %d\n", err);
		goto err_free_mem;
	}
	osp_workq = create_workqueue("osp_queue");
	if (osp_workq) {
		INIT_WORK(&osp->osp_work, osp_work_q);
	}
	if (client->irq > 0) {
		err = request_irq(client->irq, osp_irq_thread, IRQF_TRIGGER_RISING, "osp-sh", osp);
		if (err < 0)
			dev_err(&client->dev, "irq request failed %d, error %d\n",
				client->irq, err);
	}
	pr_debug("%s:%d", __func__, __LINE__);
	OSP_CB_init();
	/* Create child device */
	OSP_add_child(osp);
/*	init_completion(&hif_response_complete); */
	memset(gOSP->version, 0, sizeof(gOSP->version));
	gOSP->setv = false;
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
