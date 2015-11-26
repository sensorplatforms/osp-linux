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
#define HIF_WRITE_CONTROL_REQ 1
#define HIF_READ_CONTROL_REQ 0
#define SET_CONFIG_SZ sizeof(u16)
#define HIF_RW_REQ_MAX_SZ 44
#define HIF_RESP_DATA_SZ 150
#define HIF_PARAM_STR_LEN 12
#define DUMMY_CONFIG_CMD 1
#define SENSOR_DUMMY_RESPONSE 1
#define SENSOR_ACTUAL_RESPONSE 0
struct hif_data{
	u8 buffer[HIF_RW_REQ_MAX_SZ];
	u32 size;
};
struct hif_write_data{
	u8 param_id;
	u8 sensor_id;
	u8 seq_no;
};

#define SENSORHUB_GET_CAUSE 0x80050000
#define I2C_READ_MAX_RETRY	64
#define WORD_ALIGN(len, sz)	(((len+(sz-1))/sz)*sz)
#define MPU_GYRO_IRQ_GPIO 185

/* Cause and Error Macros */
#define RESERVED			0x00
#define SENSOR_DATA_READY		0x01
#define MAG_CALIB_DT_UPDATE		0x02
#define ACCEL_CALIB_DT_UPDATE		0x03
#define GYRO_CALIB_DT_UPDATE		0x04
#define TEMP_CHANGE_DETECT		0x05
#define CONFIG_CMD_RESP			0x06
#define UNRECOVER_ERR_MOTIONQ		0x07
#define SENSOR_EVT 0x08

/*Error code macros*/
#define ERROR_CODE_SUCCESS 0
#define ERROR_CODE_CMD_FAILED -1
#define ERROR_CODE_CMD_NOT_SUPPORTED -2
#define ERROR_CODE_INVALID_CONFIG_PKT -3
#define ERROR_CODE_PKT_CHECKSUM_FAIL -4

/* Set aside for future use */
#define MOTIONQ_EVENTS			0x08
#define VOICEQ_EVENTS			0x10

/* set config parameters format functions for creating read
    control request packet*/
void osp_read_packet(int paramid, int sensorid, int seqno, struct hif_data *buff);
/*Format function to create hif read write request packets*/
int FormatSensorReadWriteReq(u8 *pDest, int paramid, int sType, u8 subType,
u8 seqNum, u8 read_write);
/* set config parameters format functions for creating write
    control request packet*/
void osp_sensor_enable(int paramid, int sensorid, int seqno, u8 enable, struct hif_data *buff);
void osp_set_batch(int paramid, int sensorid, int seqno, u64 data[2], struct hif_data *buff);
void osp_set_flush(int paramid, int sensorid, int seqno, struct hif_data *buff);
void osp_set_axis_mapping(int paramid, int sensorid, int seqno,
	s8 data[3], struct hif_data *buff);
void osp_set_conversion_offset(int paramid, int sensorid, int seqno,
	s32 data[3], struct hif_data *buff);
void osp_set_on_wake_time(int paramid, int sensorid, int seqno,
	u32 data[2], struct hif_data *buff);
void osp_set_hpf_lpf_cutoff(int paramid, int sensorid, int seqno,
	u16 data[2], struct hif_data *buff);
void osp_set_shake_suscept(int paramid, int sensorid, int seqno,
	u16 data[3], struct hif_data *buff);
void osp_set_skor_matrix(int paramid, int sensorid, int seqno,
	u32 data[3][3], struct hif_data *buff);
void osp_set_non_linear_effects(int paramid, int sensorid, int seqno,
	u16 data[4][3], struct hif_data *buff);
void osp_set_temp_coeff(int paramid, int sensorid, int seqno,
	u16 data[3][2], struct hif_data *buff);
void osp_set_time_offset_normal_mag(int paramid, int sensorid, int seqno,
				s32 data, struct hif_data *buff);
void osp_set_calib_param_fix32(int paramid,
	int sensorid, int seqno, u32 data[3], struct hif_data *buff);
void osp_set_dynamic_calib_source(int paramid,
	int sensorid, int seqno, s8 data, struct hif_data *buff);
void osp_set_config_done(int paramid,
	int sensorid, int seqno, u8 enable, struct hif_data *buff);
