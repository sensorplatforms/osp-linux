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
#define DEBUG
#include "osp-hif-apis.h"
#include "osp-sensors.h"
#include "SensorPackets.h"
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

void osp_sensor_enable(int paramid, int sensorid, int seqno, u8 enable, struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_ENABLE_REQ_PKT_SZ;
	int i;
	memset(send_buf, 0, sizeof(send_buf));
	cmd = cpu_to_be16(0x0000 | size);
	buff->size = size + SET_CONFIG_SZ;
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
		paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&enable, sizeof(u8));
	pr_debug("%s:%d Sending sensor enable command sensor id : %d enable : %d  \n",
			__func__, __LINE__, sensorid, enable);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_batch(int paramid, int sensorid, int seqno, u64 data[2], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(unsigned long long int)*2;
	u64 sampling_freq, maximum_latency;
	int i;
	pr_debug("sampling_freq : 0x%016llx maximum_latency : 0x%016llx \n",
	sampling_freq, maximum_latency);
	sampling_freq = cpu_to_be64(data[0]);
	maximum_latency = cpu_to_be64(data[1]);
	memset(send_buf, 0, sizeof(send_buf));
	cmd = cpu_to_be16(0x0000 | size);
	buff->size = size + SET_CONFIG_SZ;
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
		paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&sampling_freq, sizeof(unsigned long long int));
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(sampling_freq),
	&maximum_latency, sizeof(unsigned long long int));
	pr_debug("%s:%d Sending sensor batch command size 0x%04x buffer size : %d \n",
		__func__, __LINE__, cmd, buff->size);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_flush(int paramid, int sensorid, int seqno, struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ;
	int i;
	memset(send_buf, 0, sizeof(send_buf));
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	pr_debug("%s:%d Sending sensor flush command sensor : %d \n", __func__, __LINE__, sensorid);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_axis_mapping(int paramid, int sensorid, int seqno,
	s8 data[3], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(s8) *3;
	int i = 0;
	memset(send_buf, 0, sizeof(send_buf));
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	for(i = 0; i < 3; i++)
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + i,
	&data[i], sizeof(u8));
	pr_debug("%s:%d Sending sensor set axis command 0x%04x \n",
		__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_conversion_offset(int paramid, int sensorid, int seqno,
	s32 data[3], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(s32) * 3;
	s32 off_data[3];
	int i = 0;
	memset(send_buf, 0, sizeof(send_buf));
	buff->size = size + SET_CONFIG_SZ;
	for(i = 0; i < 3; i++)
	off_data[i] = cpu_to_be32(data[i]);
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	for(i = 0; i < 3; i++)
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + (sizeof(s32) * i),
	&data[i], sizeof(s32));
	pr_debug("%s:%d Sending sensor set conversion offset command 0x%04x \n",
			__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_on_wake_time(int paramid, int sensorid, int seqno,
	u32 data[2], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u32)*2;
	u32 on_time, wakeup_time;
	int i;
	on_time = cpu_to_be32(data[0]);
	wakeup_time = cpu_to_be32(data[1]);
	memset(send_buf, 0, sizeof(send_buf));
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&on_time, sizeof(u32));
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u32),
	&wakeup_time, sizeof(u32));
	pr_debug("%s:%d Sending sensor set on time wake time command 0x%04x \n",
			__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_hpf_lpf_cutoff(int paramid, int sensorid, int seqno,
	u16 data[2], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u16)*2;
	u16 hpf_cutoff = cpu_to_be16(data[0]);
	u16 lpf_cutoff = cpu_to_be16(data[1]);
	int i;
	memset(send_buf, 0, sizeof(send_buf));
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&hpf_cutoff, sizeof(u16));
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u16),
	&lpf_cutoff, sizeof(u16));
	pr_debug("%s:%d Sending sensor set hpf lpf cutoff command 0x%04x \n",
			__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_shake_suscept(int paramid, int sensorid, int seqno,
	u16 data[3], struct hif_data *buff){
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u16) * 3;
	int i = 0;
	u16 shake_suscept[3];
	for(i = 0; i < 3; i++)
	shake_suscept[i] = cpu_to_be16(data[i]);
	memset(send_buf, 0, sizeof(send_buf));
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	for(i = 0; i < 3; i++)
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u16) * i,
	&shake_suscept[i], sizeof(u16));
	pr_debug("%s:%d Sending sensor shake suscept command size 0x%04x \n",
			__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_skor_matrix(int paramid, int sensorid, int seqno,
	u32 data[3][3], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u32) * 9;
	int i,j;
	u32 skor_matrix[3][3];
	for (i = 0; i < 3; i++) {
		for(j = 0; j < 3; j++)
		skor_matrix[i][j] = cpu_to_be32(data[i][j]);
	}
	memset(send_buf, 0, sizeof(send_buf));
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++)
		memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + (i * sizeof(u32)) + (j * sizeof(u32)),
		&skor_matrix[i][j], sizeof(u32));
	}
		pr_debug("%s:%d Sending sensor set skor matrix command size 0x%04x \n",
			__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_non_linear_effects(int paramid, int sensorid, int seqno,
	u16 data[4][3], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(u16) * 12;
	int i,j;
	u16 non_lin_effect[4][3];
	for (i = 0; i < 4; i++){
		for(j = 0; j < 3; j++)
		non_lin_effect[i][j] = cpu_to_be16(data[i][j]);
	}
	cmd = cpu_to_be16(0x0000 | size);
	buff->size = size + SET_CONFIG_SZ;
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
		paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	for(i = 0; i < 4; i++){
		for(j = 0; j < 3; j++)
		memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + (i * sizeof(u16)) + (j * sizeof(u16)),
		&non_lin_effect, sizeof(u16));
	}
	pr_debug("%s:%d Sending sensor set non linear effects command size 0x%04x \n",
		__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_temp_coeff(int paramid, int sensorid, int seqno,
	u16 data[3][2], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(data);
	int i,j;
	u16 temp_coeff[3][2];
	for(i = 0; i < 3; i++){
		for(j = 0; j < 2; j++)
		temp_coeff[i][j] = cpu_to_be16(data[i][j]);
	}
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
		paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	for(i = 0; i < 3 ; i++){
		for(j = 0; j < 2; j++)
		memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + (i * sizeof(u16)) + (j * sizeof(u16)),
		&data, sizeof(data));
	}
	pr_debug("%s:%d Sending sensor set temp coeff command size 0x%04x \n",
			__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_time_offset_normal_mag(int paramid, int sensorid, int seqno,
				s32 data, struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(data);
	s32 time_offset = cpu_to_be32(data);
	int i;
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
		paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&time_offset, sizeof(s32));
	pr_debug("%s:%d Sending sensor time offset command size 0x%04x \n",
		__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_calib_param_fix32(int paramid,
	int sensorid, int seqno, u32 data[3], struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(data);
	u32 fixp32[3];
	int i;
	for(i = 0; i < 3; i++)
		fixp32[i] = cpu_to_be32(data[i]);
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
		paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	for(i = 0; i < 3; i++)
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ + (i * sizeof(u32)),
	&fixp32[i], sizeof(u32));
	pr_debug("%s:%d Sending sensor set fixp32 calib command size 0x%04x \n",
		__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_dynamic_calib_source(int paramid,
	int sensorid, int seqno, int8_t data, struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ + sizeof(data);
	int i;
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
			paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	memcpy(send_buf + SET_CONFIG_SZ + SENSOR_READ_WRITE_REQ_PKT_SZ,
	&data, sizeof(data));
	pr_debug("%s:%d Sending sensor set dynamic calib source command size 0x%04x \n",
		__func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_read_packet(int paramid, int sensorid, int seqno, struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ;
	int i;
	pr_debug("%s param id = %d sensorid %d\n", __func__, paramid, sensorid);
	buff->size = size + SET_CONFIG_SZ;
	cmd = cpu_to_be16(0x0000 | size);
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
				paramid, sensorid, 0, seqno, HIF_READ_CONTROL_REQ);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}

void osp_set_config_done(int paramid, int sensorid, int seqno, u8 enable, struct hif_data *buff)
{
	u8 send_buf[HIF_RW_REQ_MAX_SZ];
	u16 cmd;
	u16 size = SENSOR_READ_WRITE_REQ_PKT_SZ;
	int i;
	memset(send_buf, 0, sizeof(send_buf));
	cmd = cpu_to_be16(0x0000 | size);
	buff->size = size + SET_CONFIG_SZ ;
	memcpy(send_buf, &cmd, SET_CONFIG_SZ);
	FormatSensorReadWriteReq(send_buf + SET_CONFIG_SZ,
		paramid, sensorid, 0, seqno, HIF_WRITE_CONTROL_REQ);
	pr_debug("%s:%d size : 0x%04x \n", __func__, __LINE__, cmd);
	for(i = 0; i < buff->size; i++)
		buff->buffer[i] = send_buf[i];
}
