/* osp-hif-apis.h  --  Audience Sensorhub HIF interface apis
 *
 * Copyright 2011 Audience, Inc.
 *
 * Author: Amit Jain
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* macros/ data structures */
#include <linux/skbuff.h>
#define ES_SET_CONFIG 0x80060000
#define HIF_WRITE_CONTROL_REQ 1
#define HIF_READ_CONTROL_REQ 0
#define SET_CONFIG_SZ sizeof(u32)
#define HIF_RW_REQ_MAX_SZ 44
#define HIF_RESP_DATA_SZ 150
#define SENSOR_DUMMY_RESPONSE 1
#define SENSOR_ACTUAL_RESPONSE 0
#define HIF_PARAM_STR_LEN 12
struct hif_data{
	u8 buffer[HIF_RW_REQ_MAX_SZ];
	u32 size;
};
struct hif_write_data{
	u8 param_id;
	u8 sensor_id,
	u8,seq_no;
};
/* set config parameters format functions for creating read
    control request packet*/
struct hif_data osp_read_packet(int paramid, int sensorid, int seqno);
/*Format function to create hif read write request packets*/
int FormatSensorReadWriteReq(u8 *pDest, int paramid, int sType, u8 subType,
u8 seqNum, u8 read_write);
/* set config parameters format functions for creating write
    control request packet*/
struct hif_data osp_sensor_enable(int paramid, int sensorid, int seqno, u8 enable);
struct hif_data osp_set_batch(int paramid, int sensorid, int seqno, u64 data[2]);
struct hif_data osp_set_flush(int paramid, int sensorid, int seqno);
struct hif_data osp_set_axis_mapping(int paramid, int sensorid, int seqno,
	s8 data[3]);
struct hif_data osp_set_conversion_offset(int paramid, int sensorid, int seqno,
	s32 data[3]);
struct hif_data osp_set_on_wake_time(int paramid, int sensorid, int seqno,
	u32 data[2]);
struct hif_data osp_set_hpf_lpf_cutoff(int paramid, int sensorid, int seqno,
	u16 data[2]);
struct hif_data osp_set_shake_suscept(int paramid, int sensorid, int seqno,
	u16 data[3]);
struct hif_data osp_set_skor_matrix(int paramid, int sensorid, int seqno,
	u32 data[3][3]);
struct hif_data osp_set_non_linear_effects(int paramid, int sensorid, int seqno,
	u16 data[4][3]);
struct hif_data osp_set_temp_coeff(int paramid, int sensorid, int seqno,
	u16 data[3][2]);
struct hif_data osp_set_time_offset_normal_mag(int paramid, int sensorid, int seqno,
				s32 data);
struct hif_data osp_set_calib_param_fix32(int paramid,
	int sensorid, int seqno, u32 data[3]);
struct hif_data osp_set_dynamic_calib_source(int paramid,
	int sensorid, int seqno, s8 data);
