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
#include "MQ_sensors.h"
#include "SensorPackets.h"
#include "osp_i2c_map.h"
#include <linux/osp-sh.h>

#define NAME			"ospsh"

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

	/* Private not supported yet */
	if (private != 0) return -EINVAL;

	if (sensor < 0x30)
		retval = i2c_smbus_read_byte_data(osp->client, 0x20+sensor);

	return retval;
}

static int OSP_Sensor_disable(struct osp_data *osp, int sensor, int private)
{
	int retval = -EINVAL;

	/* Private not supported yet */
	if (private != 0) return -EINVAL;

	if (sensor < 0x30)
		retval = i2c_smbus_read_byte_data(osp->client, 0x50+sensor);

	return retval;
}

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
	struct OSP_Sensor *sen;

	if (private == 1) {
		sen = prv_sensor;
		return -EINVAL;	/* Private not supported yet */
	} else if (private == 0)
		sen = and_sensor;
	else
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
	static int counter = 0;
	union OSP_SensorData data;

	switch(spack->SType) {
	case SENSOR_STEP_DETECTOR:
	case SENSOR_SIGNIFICANT_MOTION:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = 1;
		data.xyz.y = counter;
		counter++;
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private, jiffies, &data);
		break;
	case SENSOR_STEP_COUNTER:
	case SENSOR_PRESSURE:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.CalFixP.Axis[0];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private, jiffies, &data);
		break;

	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.UncalFixP.Axis[0];
		data.xyz.y = spack->P.UncalFixP.Axis[1];
		data.xyz.z = spack->P.UncalFixP.Axis[2];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private, jiffies, &data);
		break;
	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
	case SENSOR_ORIENTATION:
		if (!and_sensor[spack->SType].dataready) break;
		data.xyz.x = spack->P.CalFixP.Axis[0];
		data.xyz.y = spack->P.CalFixP.Axis[1];
		data.xyz.z = spack->P.CalFixP.Axis[2];
		and_sensor[spack->SType].dataready(spack->SType, 0,
			and_sensor[spack->SType].private, jiffies, &data);
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
			and_sensor[spack->SType].private, jiffies, &data);
		break;
	case PSENSOR_ACCELEROMETER_UNCALIBRATED|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!prv_sensor[PSensor].dataready) break;

		data.xyz.x = spack->P.UncalFixP.Axis[0];
		data.xyz.y = spack->P.UncalFixP.Axis[1];
		data.xyz.z = spack->P.UncalFixP.Axis[2];
		prv_sensor[PSensor].dataready(PSensor, 1,
			prv_sensor[PSensor].private, jiffies, &data);
		break;
	case PSENSOR_ACCELEROMETER_RAW|SENSOR_DEVICE_PRIVATE_BASE:
		PSensor = spack->SType & ~(SENSOR_DEVICE_PRIVATE_BASE);
		if (!prv_sensor[PSensor].dataready) break;
		data.xyz.x = spack->P.RawSensor.Axis[0];
		data.xyz.y = spack->P.RawSensor.Axis[1];
		data.xyz.z = spack->P.RawSensor.Axis[2];
		prv_sensor[PSensor].dataready(PSensor, 1,
			prv_sensor[PSensor].private, jiffies, &data);
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
		ret = osp_i2c_read(osp, OSP_INT_LEN,
				(unsigned char *)&intlen, 4);
		if ((ret >= 0) && (intlen&OSP_INT_DRDY)) {
			plen[i] = (intlen >> 4);
			if (plen[i] > 0 && plen[i] < 8192) {
				ret = osp_i2c_read(osp, OSP_DATA_OUT,
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
				ret = OSP_ParseSensorDataPkt(&spack, pack_ptr, plen[i]);
				if (ret>0) {
					OSP_ReportSensor(osp, &spack);
				} else {
					dev_err(&osp->client->dev,
						 "OSP packet parsing error = %i, pack_count = %i plen = %i\n", ret, pack_count, plen[i]);
					break;
				}
				pr_debug("OSP Read data len %d, packet_count %d\n", plen[i], pack_count);
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

static int OSP_add_child(struct osp_data *osp)
{
	struct platform_device *pdev;

	pdev = platform_device_alloc("osp-output", 0);
	if (!pdev) {
		printk("Cannot allocate dev\n");
		return -ENOMEM;
	}
	pdev->dev.parent = &osp->client->dev;

	return platform_device_add(pdev);
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
