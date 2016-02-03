/*
 * osp-hif-apis.c  --  Audience Sensorhub HIF interface apis definitions.
 *
 * Copyright 2011 Audience, Inc.
 *
 * Author: Amit Jain
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/sched.h>
#include <linux/delay.h>
#include "osp-hif-apis.h"
#include "osp-sensors.h"
#include "SensorPackets.h"

#define TS_DRIFT_THRESHOLD 		(150*1000000)

extern atomic_t enable_count;
extern void osp_timesync_start(void);
/* functions */
int FormatSensorReadWriteReq(u8 *pDest,
		int paramid,
		int sType,
		u8 subType,
		u8 seqNum,
		u8 read_write)
{
	struct _HifSensorReadWriteReqPacket *pOut =
	(struct _HifSensorReadWriteReqPacket *)pDest;
	/* Sanity checks... */
	if (pOut == NULL)
		return -MQ_BAD_BUFFER;
	/* Check Sensor enumeration type Android or User defined */
	if (!(M_GetSensorType(sType))) {
		pOut->Q.ControlByte = SENSOR_TYPE_ANDROID;
	} else {
		pOut->Q.ControlByte = SENSOR_TYPE_PRIVATE;
	}
	/* Setup Control Byte */
	if (read_write)
		pOut->Q.ControlByte |= PKID_CONTROL_REQ_WR;
	else
		pOut->Q.ControlByte |= PKID_CONTROL_REQ_RD;
	/* Setup Sensor ID Byte */
	pOut->Q.SensorIdByte = M_SensorType(sType);

	/* Setup Attribute Byte 1 */
	pOut->Q.AttributeByte = M_SensorSubType(subType) |\
				M_SequenceNum(seqNum);
	/* Attribute Byte 2 */
	pOut->AttrByte2 = M_SetParamId(paramid);
	/* Return the length of the packet */
	return SENSOR_READ_WRITE_REQ_PKT_SZ;
}

inline void format_hif_packet(struct hif_data *buff, u16 size,
						int paramid, int sensorid, int seqno)
{
	u16 cmd = cpu_to_be16(0x0000 | size);
	buff->size = size + SET_CONFIG_SZ;
	memcpy(buff->buffer, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(buff->buffer + SET_CONFIG_SZ,
						paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	pr_debug("%s:%d size 0x%04x buffer size : %d \n",
		__func__, __LINE__, cmd, buff->size);
}

void osp_sensor_enable(int paramid, int sensorid, int seqno,
		u8 enable, struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ , sizeof(u8) , 1);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	buff->buffer[SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ] = enable;
	pr_debug("%s:%d Sending sensor enable command sensor id : %d enable : %d  \n",
			__func__, __LINE__, sensorid, enable);
}

void osp_sensorhub_ts_init(struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ ,
				sizeof(u64), 1);
	u64 tsinit = time_get_ns();
	pr_debug("%s :: tsinit: %llu\n",	__func__, tsinit);
	format_hif_packet(buff, size, PARAM_ID_TSYNC_SET_TIME, 0, 0);
	tsinit = cpu_to_be64(tsinit);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
			&tsinit, sizeof(u64));
	return;
}

u64 osp_sensorhub_ts_start(struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, 0, 0);
	u64 ts = time_get_ns();
	pr_debug("%s :: ts-start\n", __func__);
	format_hif_packet(buff, size, PARAM_ID_TSYNC_START, 0, 0);
	return ts;
}

void osp_sensorhub_ts_followup(u64 ts, struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ ,
				sizeof(u64), 1);
	pr_debug("%s :: ts-send: %llu\n",	__func__, ts);
	ts = cpu_to_be64(ts);
	format_hif_packet(buff, size, PARAM_ID_TSYNC_FOLLOWUP, 0, 0);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
			&ts, sizeof(u64));
	return;
}

void osp_sensorhub_ts_end(u64 ts, struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ ,
				sizeof(u64), 1);
	pr_debug("%s :: ts-send: %llu\n",	__func__, ts);
	format_hif_packet(buff, size, PARAM_ID_TSYNC_END, 0, 0);
	ts = cpu_to_be64(ts);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
			&ts, sizeof(u64));
	return;
}

void osp_set_batch(int paramid, int sensorid, int seqno,
		u64 data[2], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ ,
				sizeof(unsigned long long int) , 2);
	u64 sampling_freq = cpu_to_be64(data[0]);
	u64 maximum_latency = cpu_to_be64(data[1]);
	pr_debug("%s :: sampling freq : 0x%016llx MRL: 0x%016llx \n",
	__func__, sampling_freq, maximum_latency);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&sampling_freq, sizeof(unsigned long long int));
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(sampling_freq),
	&maximum_latency, sizeof(unsigned long long int));
}

void osp_set_flush(int paramid, int sensorid, int seqno, struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ , 0, 0);
	pr_debug("%s :: sensor id : %d \n", __func__, sensorid);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
}

void osp_set_axis_mapping(int paramid, int sensorid, int seqno,
	s8 data[3], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(s8), 3);
	int i = 0;
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	for(i = 0; i < 3; i++)
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + i,
	&data[i], sizeof(u8));
}

void osp_set_conversion_offset(int paramid, int sensorid, int seqno,
	s32 data[3], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(s32), 3);
	s32 off_data[3];
	int i;
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	for(i = 0; i < 3; i++)
		off_data[i] = cpu_to_be32(data[i]);
	for(i = 0; i < 3; i++)
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + (sizeof(s32) * i),
	&data[i], sizeof(s32));
}

void osp_set_on_wake_time(int paramid, int sensorid, int seqno,
	u32 data[2], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(u32), 2);
	u32 on_time = cpu_to_be32(data[0]), wakeup_time = cpu_to_be32(data[1]);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&on_time, sizeof(u32));
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u32),
	&wakeup_time, sizeof(u32));
}

void osp_set_hpf_lpf_cutoff(int paramid, int sensorid, int seqno,
	u16 data[2], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(u16), 2);
	u16 hpf_cutoff = cpu_to_be16(data[0]);
	u16 lpf_cutoff = cpu_to_be16(data[1]);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&hpf_cutoff, sizeof(u16));
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u16),
	&lpf_cutoff, sizeof(u16));
}

void osp_set_shake_suscept(int paramid, int sensorid, int seqno,
	u16 data[3], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(u16), 3);
	int i = 0;
	u16 shake_suscept[3];
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	for(i = 0; i < 3; i++)
		shake_suscept[i] = cpu_to_be16(data[i]);
	for(i = 0; i < 3; i++)
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u16) * i,
	&shake_suscept[i], sizeof(u16));
}

void osp_set_skor_matrix(int paramid, int sensorid, int seqno,
	u32 data[3][3], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(u32), 9);
	int i,j;
	u32 skor_matrix[3][3];
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	for (i = 0; i < 3; i++) {
		for(j = 0; j < 3; j++)
			skor_matrix[i][j] = cpu_to_be32(data[i][j]);
	}
	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++)
		memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ
			+ (i * sizeof(u32)) + (j * sizeof(u32)),
		&skor_matrix[i][j], sizeof(u32));
	}
}

void osp_set_non_linear_effects(int paramid, int sensorid, int seqno,
	u16 data[4][3], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(u16), 12);
	int i,j;
	u16 non_lin_effect[4][3];
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	for (i = 0; i < 4; i++){
		for(j = 0; j < 3; j++)
			non_lin_effect[i][j] = cpu_to_be16(data[i][j]);
	}
	for(i = 0; i < 4; i++){
		for(j = 0; j < 3; j++)
		memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ +
			(i * sizeof(u16)) + (j * sizeof(u16)),
		&non_lin_effect, sizeof(u16));
	}
}

void osp_set_temp_coeff(int paramid, int sensorid, int seqno,
	u16 data[3][2], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(u16), 6);
	int i,j;
	u16 temp_coeff[3][2];
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	for(i = 0; i < 3; i++){
		for(j = 0; j < 2; j++)
			temp_coeff[i][j] = cpu_to_be16(data[i][j]);
	}
	for(i = 0; i < 3 ; i++){
		for(j = 0; j < 2; j++)
		memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ +
		(i * sizeof(u16)) + (j * sizeof(u16)),
		&data, sizeof(data));
	}
}

void osp_set_time_offset_normal_mag(int paramid, int sensorid, int seqno,
				s32 data, struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(s32), 1);
	s32 time_offset = cpu_to_be32(data);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&time_offset, sizeof(s32));
}

void osp_set_calib_param_fix32(int paramid,
	int sensorid, int seqno, u32 data[3], struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(u32), 3);
	u32 fixp32[3];
	int i;
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	for(i = 0; i < 3; i++)
		fixp32[i] = cpu_to_be32(data[i]);
	for(i = 0; i < 3; i++)
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + (i * sizeof(u32)),
	&fixp32[i], sizeof(u32));
}

void osp_set_dynamic_calib_source(int paramid,
	int sensorid, int seqno, int8_t data, struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, sizeof(int8_t), 1);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
	memcpy(buff->buffer + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&data, sizeof(data));
}

void osp_read_packet(int paramid, int sensorid, int seqno,
		struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, 0, 0);
	u16 cmd = cpu_to_be16(0x0000 | size);
	pr_debug("%s param id = %d sensorid %d\n", __func__, paramid, sensorid);
	buff->size = size + SET_CONFIG_SZ;
	memcpy(buff->buffer, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(buff->buffer + SET_CONFIG_SZ,
				paramid, sensorid, 0, seqno, HIF_READ_CONTROL_REQ);
}

void osp_set_config_done(int paramid, int sensorid, int seqno,
		struct hif_data *buff)
{
	u16 size = HIF_PACKET_SIZE(SENSOR_READ_WRITE_REQ_PKT_SZ, 0, 0);
	format_hif_packet(buff, size, paramid, sensorid, seqno);
}

bool format_config_write_request(const char *buf, struct hif_data *buff)
{
	int param_id, sensor_id, seq_no;
	bool IsParamIDCorrect = true;
	sscanf(buf, "0x%02x %d 0x%02x", &sensor_id, &seq_no, &param_id);
	pr_debug("%s param_id : %d, sensor id : %d, seq : %d\n", __func__,
		param_id, sensor_id, seq_no);
	/* parse remaining buffer depending upon setconfig parameters*/
	switch (param_id) {
		/*sensor enable 0x01*/
	case PARAM_ID_ENABLE:
	{
		u8 enable;
		sscanf(buf + HIF_PARAM_STR_LEN, "%c", &enable);
		enable = enable - '0';
		pr_debug("%s enable = %d\n", __func__, enable);
		if(enable) {
			if((atomic_read(&enable_count) == 0)) {
					osp_timesync_start();
			}
			atomic_inc(&enable_count);
		}
		else {
			if(atomic_read(&enable_count) > 0) {
				atomic_dec(&enable_count);
			}
		}
		osp_sensor_enable(param_id, sensor_id, seq_no, enable, buff);
		break;
	}

	case PARAM_ID_BATCH:
	{
		u64 data[2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%16llx 0x%16llx", &data[0], &data[1]);
		pr_debug("%s data[0] = %llu, data[1] = %llu \n", __func__,
		data[0], data[1]);
		osp_set_batch(param_id, sensor_id, seq_no, data, buff);
		break;
	}
	case PARAM_ID_FLUSH:
	{
		osp_set_flush(param_id, sensor_id, seq_no, buff);
		break;
	}
	case PARAM_ID_AXIS_MAPPING:
	{
		s8 data[3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hhx 0x%hhx 0x%hhx",
			&data[0], &data[1], &data[2]);
		pr_debug("%s data[0] = %hhd, data[1] = %hhd, data[2] = %hhd \n",
			__func__, data[0], data[1], data[2]);
		osp_set_axis_mapping(param_id, sensor_id, seq_no,
			data, buff);
		break;
	}
	case PARAM_ID_CONVERSION_OFFSET:
	{
		s32 data[3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x 0x%08x 0x%08x",
			&data[0], &data[1], &data[2]);
		pr_debug("%s data[0] = %d, data[1] = %d, data[2] = %d \n",
			 __func__, data[0], data[1], data[2]);
		osp_set_conversion_offset(param_id, sensor_id, seq_no,
			data, buff);
		break;
	}
	case PARAM_ID_TIMESTAMP_OFFSET:
	case PARAM_ID_EXPECTED_NORM:
	{
		s32 data;
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x", &data);
		pr_debug("%s data = 0x%08x \n", __func__, data);
		osp_set_time_offset_normal_mag(param_id, sensor_id,
		seq_no, data, buff);
		break;
	}
	case PARAM_ID_ONTIME_WAKETIME:
	{
		u32 data[2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x 0x%08x", &data[0], &data[1]);
		pr_debug("%s data[0] = %d data[1] = %d \n", __func__,
			data[0], data[1]);
		osp_set_on_wake_time(param_id, sensor_id, seq_no, data, buff);
		break;
	}
	case PARAM_ID_HPF_LPF_CUTOFF:
	{
		u16 data[2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx", &data[0], &data[1]);
		pr_debug("%s data[0] = %hu data[1] = %hu \n", __func__,
			data[0], data[1]);
		osp_set_hpf_lpf_cutoff(param_id, sensor_id, seq_no,
					data, buff);
		break;
	}
	case PARAM_ID_F_SKOR_MATRIX:
	{
		u32 data[3][3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x", \
			&data[0][0], &data[0][1], &data[0][2], &data[1][0], &data[1][1],\
			&data[1][2], &data[2][0], &data[2][1], &data[2][2]);
		osp_set_skor_matrix(param_id, sensor_id, seq_no, data, buff);
		break;
	}
	case PARAM_ID_F_NONLINEAR_EFFECTS:
	{
		u16 data[4][3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx",\
			&data[0][0], &data[0][1], &data[0][2], &data[1][0],\
			&data[1][1], &data[1][2], &data[2][0], &data[2][1],\
			&data[2][2], &data[3][0], &data[3][1], &data[3][2]);
		osp_set_non_linear_effects(param_id, sensor_id, seq_no,
		data, buff);
		break;
	}
	case PARAM_ID_TEMP_COEFF:
	{
		u16 data[3][2];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx 0x%hx 0x%hx 0x%hx 0x%hx", &data[0][1],\
			&data[0][1], &data[1][0], &data[1][2],\
			&data[2][0], &data[2][1]);
		osp_set_temp_coeff(param_id, sensor_id, seq_no, data, buff);
		break;
	}
	case PARAM_ID_SHAKE_SUSCEPTIBILIY:
	{
		u16 data[3];
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hx 0x%hx 0x%hx", &data[0], &data[1], &data[2]);
		pr_debug("%s data[0] = %hu data[1] = %hu data[2] = %hu\n", __func__,
			data[0], data[1], data[2]);
		osp_set_shake_suscept(param_id, sensor_id, seq_no,
				data, buff);
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
		pr_debug("%s data[0] = 0x%08x, data[1] = 0x%08x data[2] = 0x%08x\n",
			 __func__, data[0], data[1], data[2]);
		osp_set_calib_param_fix32(param_id, sensor_id,
		seq_no, data, buff);
		break;
	}
	case PARAM_ID_DYNAMIC_CAL_SOURCE:
	{
		s8 data;
		sscanf(buf + HIF_PARAM_STR_LEN, "0x%hhx", &data);
		osp_set_dynamic_calib_source(param_id, sensor_id, seq_no, data, buff);
		pr_debug("%s data = %d\n", __func__, data);
		break;
	}
	case PARAM_ID_CONFIG_DONE:
	{
		osp_set_config_done(param_id, sensor_id, seq_no, buff);
		break;
	}
	default:
		pr_debug("%s wrong parameter entered for sending \
				write request packet\n", __func__);
		IsParamIDCorrect = false;
		break;
	}
	return IsParamIDCorrect;
}
