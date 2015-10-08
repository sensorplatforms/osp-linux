/*****************************************************************************
 * @file  SensorPackets.h
 *
 * Definitions and macros for host inteface packet format
 *
 * < Detailed description of subsystem >
 *
 * @seealso < Links to relevant files, references, etc. >
 *
 * @info
 * $Author: rverma $
 * $DateTime: 2014/11/13 17:24:40 $
 * $Revision: #8 $
 *
 * Copyright 2012 Audience, Incorporated. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * Audience Inc. ("Confidential Information"). You shall not disclose
 * such Confidential Information and shall use it only in accordance
 * with the Terms of Sale of Audience products and the terms of any
 * license agreement you entered into with Audience for such products.
 *
 * AUDIENCE SOURCE CODE STRICTLY "AS IS" WITHOUT ANY WARRANTY
 * WHATSOEVER, AND AUDIENCE EXPRESSLY DISCLAIMS ALL WARRANTIES,
 * EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE, TITLE OR NON-NFRINGEMENT OF THIRD PARTY RIGHTS. AUDIENCE
 * SHALL NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF
 * USING, MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
 *
 ****************************************************************************/
#if !defined (SENSOR_PACKETS_H)
#define   SENSOR_PACKETS_H

/*--------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
 \*--------------------------------------------------------------------------*/
#if 0
#include <stdint.h>
#include <stddef.h>
#endif
#include "MQ_sensors.h"

/*--------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*--------------------------------------------------------------------------*/
/********************************************************/
/*              COMMON PACKET DEFINITION                */
/********************************************************/
/* Packet Identifier for version 0 packets */
#define PKID_SENSOR_DATA	0x00
#define PKID_CONTROL_REQ	0x10
#define PKID_CONTROL_RESP	0x20
#define PKID_TEST_DATA		0x30
#define PKID_MASK_VER0		0xF0

/** =============== CONTROL ID BYTE =============== */
#define M_GetSensorType(i)	((uint8_t)((i >> 16 ) & 0x01))
#define SENSOR_TYPE_ANDROID	0x0
#define SENSOR_TYPE_PRIVATE	0x1
/** =============== SENSOR ID BYTE =============== */
#define M_SensorMetaData(i)	((uint8_t)((i << 6) & 0xC0))
#define M_ParseSensorMetaData(i)	((uint8_t)((i >> 6) & 0x03))
#define M_SensorType(s)		((uint8_t)(s & 0x3F))

/** =============== ATTRIBUTE BYTE =============== */
#define M_SensorSubType(st)	((uint8_t)((st << 4) & 0xF0))
#define M_ParseSensorSubType(st)	((uint8_t)((st >> 4) & 0x0F))

/* Checksum option (check sum is always 16-bit CRC if enabled) */
#define CHECK_SUM_PRESENT	0x01

/********************************************************/
/*              SENSOR DATA PACKET                      */
/********************************************************/
/** =============== CONTROL BYTE =============== */
/* CRC option */
#define CRC_ENABLE		0x08

/*Enumeration type of sensor*/
#define SENSOR_ANDROID_TYPE_MASK	0x01

/* Format for data values */
#define DATA_FORMAT_RAW		0x00    /* Raw counts (no units) from sensor values */
#define DATA_FORMAT_FIXPOINT	0x04    /* Fixed point format in application defined units */
#define DATA_FORMAT_MASK	0x04

/* Time Format option */
#define TIME_FORMAT_RAW		0x00    /* RAW count values for time stamp base */
#define TIME_FORMAT_FIXPOINT	0x02    /* Fixed point format specified in seconds */
#define TIME_FORMAT_MASK	0x02

/** ============ SENSOR IDENTIFIER BYTE ========== */
#define META_DATA_UNUSED	0x00    /* Meta Data Identifier  no used*/
#define META_DATA_OFFSET_CHANGE	0x01    /* Meta Data Identifier */

/** =============== ATTRIBUTE BYTE =============== */
/* Data size option */
#define DATA_SIZE_8_BIT		0x00
#define DATA_SIZE_16_BIT	0x02
#define DATA_SIZE_32_BIT	0x04
#define DATA_SIZE_64_BIT	0x06
#define DATA_SIZE_MASK		0x06

/* Time Stamp Size */
#define TIME_STAMP_32_BIT	0x00    /* Uncompressed 32-bit time stamp */
#define TIME_STAMP_64_BIT	0x01    /* Uncompressed 64-bit time stamp */
#define TIME_STAMP_SIZE_MASK	0x01

/********************************************************/
/*          SENSOR CONTROL REQ/RESP PACKET              */
/********************************************************/
/** =============== CONTROL BYTE =============== */
/* First 5 MS bits are same as Sensor Data Packet */
/* Data Format */
#define CTRL_PKT_DF_INTEGER	0x00
#define CTRL_PKT_DF_FIXPOINT	0x02
#define CTRL_PKT_DF_FLOAT	0x04
#define CTRL_PKT_DF_DOUBLE	0x06

/* A/P Definition same as Sensor Data Packet */

/** =============== ATTRIBUTE BYTE 1 =============== */
/* Sequence Number for request/response */
#define M_SequenceNum(sNum)	((sNum) & 0x0F)

/** =============== ATTRIBUTE BYTE 2 =============== */
/* Parameter ID */
#define M_GetParamId(AttrByte2)	((AttrByte2) >> 3)
#define M_SetParamId(id)	(((id) & 0x1F) << 3)

/* Parameter Size */
#define M_GetParamSize(AttrByte2)	((AttrByte2) & 0x07)

/* Param data size */
#define PARAM_DATA_SZ_8_BIT	0x00
#define PARAM_DATA_SZ_16_BIT	0x01
#define PARAM_DATA_SZ_32_BIT	0x02
#define PARAM_DATA_SZ_64_BIT	0x03
#define PARAM_DATA_SZ_BOOL_FALSE	0x04
#define PARAM_DATA_SZ_BOOL_TRUE	0x05
#define PARAM_DATA_SZ_UNKNOWN	0x07


/* Byte extraction macros */
#define BYTE3(x)	((uint8_t)((x >> 24) & 0xFF))
#define BYTE2(x)	((uint8_t)((x >> 16) & 0xFF))
#define BYTE1(x)	((uint8_t)((x >> 8) & 0xFF))
#define BYTE0(x)	((uint8_t)(x & 0xFF))

/* 16-bit & 32-bit data combine macros */
#define BYTES_TO_SHORT(b1,b0)	((int16_t)(((int16_t)b1 << 8) | b0))
#define BYTES_TO_LONG_ARR(arr,ind)	BYTES_TO_LONG(arr[ind],arr[ind+1],arr[ind+2],arr[ind+3])
#define BYTES_TO_LONG(b3,b2,b1,b0)	\
    ((int32_t)(((int32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0))

/*--------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*--------------------------------------------------------------------------*/
/* Definition for quaternion data packets for internal usage */
typedef struct _QuaternionFixP {
	TimeStamp_t	TimeStamp;
	int32_t		Quat[4]; //W,X,Y,Z order
} QuaternionFixP_t;

typedef struct _UncalibratedFixP {
	TimeStamp_t	TimeStamp;
	int32_t		Axis[3];   //X,Y,Z order
	int32_t		Offset[3]; //XOFF,YOFF,ZOFF order
} UncalibratedFixP_t;

typedef struct _CalibratedFixP {
	TimeStamp_t	TimeStamp;
	int32_t		Axis[3]; //X,Y,Z,
} CalibratedFixP_t;

/* Union of the structures that can be parsed out of Sensor Data packet */
typedef struct _SensorPacketTypes {
	union {
		TriAxisRawData_t	RawSensor;
		UncalibratedFixP_t	UncalFixP;
		CalibratedFixP_t	CalFixP;
		QuaternionFixP_t	QuatFixP;
	} P;
	ASensorType_t SType;
	uint8_t SubType;
} SensorPacketTypes_t;

/* ========== Host Inteface Packet definitions ========== */
/*
 * WARNING - Do not change structure definition without changing the handlers
 * and the document describing the Host Interface Protocol
 */

/* Sensor Packet Type qualifier */
typedef struct _HifSensorPktQualifier {
	uint8_t	ControlByte;
	uint8_t	SensorIdByte;
	uint8_t	AttributeByte;
} HifSnsrPktQualifier_t;

/* Basic packet: Raw 16-bit data, 32-bit integer time stamp; No checksum */
typedef struct _HifSensorDataRaw {
	HifSnsrPktQualifier_t	Q;
	uint8_t			TimeStamp[4];   //32-bit Time Stamp
	uint8_t			DataRaw[6];     //3-Axis Raw 16-bit sensor data
} HifSensorDataRaw_t;

#define SENSOR_RAW_DATA_PKT_SZ	sizeof(HifSensorDataRaw_t)

/* Uncalibrated data packet:
 *   32-bit Fixedpoint uncalibrated;
 *   64-bit Fixedpoint time stamp;
 *   No Checksum */
typedef struct _HifUncalibratedFixPoint {
	HifSnsrPktQualifier_t	Q;
	uint8_t	TimeStamp[8];   //64-bit Time Stamp in fixed point format
	uint8_t	Data[12];       //3-Axis Fixed point 32-bit uncalibrated data
	uint8_t	Offset[12];     //3-Axis Fixed point 32-bit Offset
} HifUncalibratedFixPoint_t;

#define UNCALIB_FIXP_DATA_PKT_SZ	(offsetof(HifUncalibratedFixPoint_t, Offset))
#define UNCALIB_FIXP_DATA_OFFSET_PKT_SZ	sizeof(HifUncalibratedFixPoint_t)

/* Calibrated data packet: 32-bit Fixedpoint Calibrated; 64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifCalibratedFixPoint {
	HifSnsrPktQualifier_t	Q;
	uint8_t	TimeStamp[8];	//64-bit Time Stamp in fixed point format
	uint8_t	Data[12];	//3-Axis Fixed point 32-bit calibrated data
} HifCalibratedFixPoint_t;

#define CALIBRATED_FIXP_DATA_PKT_SZ	sizeof(HifCalibratedFixPoint_t)

/* Quaternion data packet: 32-bit Fixedpoint quaternion; 64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifQuaternionFixPoint {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8];   //64-bit Time Stamp in fixed point format
	uint8_t Data[16];       //4-Axis Fixed point 32-bit quaternion data
} HifQuaternionFixPoint_t;

#define QUATERNION_FIXP_DATA_PKT_SZ	sizeof(HifQuaternionFixPoint_t)

/* Sensor control packet for enable/disable sensor */
typedef struct _HifSensorEnable {
	HifSnsrPktQualifier_t	Q;
	uint8_t			AttrByte2;
} HifSensorEnable_t;

#define SENSOR_ENABLE_REQ_PKT_SZ	sizeof(HifSensorEnable_t)

/* Define union for all the host interface packet types */
typedef union _HostIFPackets {
	HifSensorDataRaw_t		SensPktRaw;
	HifUncalibratedFixPoint_t	UncalPktFixP;
	HifCalibratedFixPoint_t		CalPktFixP;
	HifQuaternionFixPoint_t		QuatPktFixP;
	HifSensorEnable_t		Enable;
} HostIFPackets_t;

/*--------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*--------------------------------------------------------------------------*/
int16_t FormatSensorDataPktRaw(uint8_t *pDest,
			const TriAxisRawData_t *pSensData,
			uint8_t metaData, ASensorType_t sType,
			uint8_t subType);
int16_t FormatQuaternionPktFixP(uint8_t *pDest,
			const QuaternionFixP_t *pQuatData,
			ASensorType_t sType);
int16_t FormatUncalibratedPktFixP(uint8_t *pDest,
			const UncalibratedFixP_t *pUncalData,
			uint8_t metaData, ASensorType_t sType);
int16_t FormatCalibratedPktFixP(uint8_t *pDest,
			const CalibratedFixP_t *pCalData,
			ASensorType_t sType);
int16_t ParseHostIntefacePkt(SensorPacketTypes_t *pOut,
			uint8_t *pPacket, uint16_t pktSize);

#endif /* SENSOR_PACKETS_H */
/*--------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*--------------------------------------------------------------------------*/
