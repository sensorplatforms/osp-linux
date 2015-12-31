/*****************************************************************************
 * @file  SensorPackets.h
 *
 * Definitions and macros for host interface packet format
 *
 * < Detailed description of subsystem >
 *
 * @seealso < Links to relevant files, references, etc. >
 *
 * @info
 * $Author: rverma $
 * $DateTime: 2015/06/26 11:03:37 $
 * $Revision: #4 $
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
#include "osp-sensors.h"
#if 0
#include "osp-types.h"
#endif

/*--------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*--------------------------------------------------------------------------*/
/********************************************************/
/*              COMMON PACKET DEFINITION                */
/********************************************************/
/* Packet Identifier for version 0 packets */
#define PKID_SENSOR_DATA                0x00
#define PKID_CONTROL_REQ_RD             0x10
#define PKID_CONTROL_REQ_WR             0x20
#define PKID_CONTROL_RESP               0x30
#define PKID_TEST_DATA                  0x40
#define PKID_MASK_VER0                  0xF0

/* CONTROL ID BYTE */
#define M_GetSensorType(i)	((u8)((i >> 16) & 0x01))
#define SENSOR_TYPE_ANDROID	0x0
#define SENSOR_TYPE_PRIVATE	0x1
/* SENSOR ID BYTE */
#define M_SensorMetaData(i)		((u8)((i << 6) & 0xC0))
#define M_ParseSensorMetaData(i)        ((u8)((i >> 6) & 0x03))
#define M_SensorType(s)                 ((u8)(s & 0x3F))

/* ATTRIBUTE BYTE */
#define M_SensorSubType(st)             ((u8)((st << 4) & 0xF0))
#define M_ParseSensorSubType(st)        ((u8)((st >> 4) & 0x0F))
#define M_FLUSH_COMPLETED(st)			((u8)((st >> 3) & 0x01))

/* ATTRIBUTE BYTE2*/
#define M_ParseConfigParam(st)			((u8)(st & 0xFF))

/* Checksum option (check sum is always 16-bit CRC if enabled) */
#define CHECK_SUM_PRESENT       0x01

/********************************************************/
/*              SENSOR DATA PACKET                      */
/********************************************************/
/** =============== CONTROL BYTE =============== */
/* CRC option */
#define CRC_ENABLE              0x08

/*Enumeration type of sensor*/
#define SENSOR_ANDROID_TYPE_MASK        0x01

/* Format for data values */
#define DATA_FORMAT_RAW         0x00  /* Raw counts (no units) from sensor */
#define DATA_FORMAT_FIXPOINT    0x04  /* FixPoint format in app defined units */
#define DATA_FORMAT_MASK        0x04

/* Time Format option */
#define TIME_FORMAT_RAW         0x00  /* RAW count values for timestamp base */
#define TIME_FORMAT_FIXPOINT    0x02  /* FixPoint format in seconds */
#define TIME_FORMAT_MASK        0x02

/** ============ SENSOR IDENTIFIER BYTE ========== */
#define META_DATA_UNUSED        0x00    /* Meta Data Identifier  no used*/
#define META_DATA_OFFSET_CHANGE 0x01    /* Meta Data Identifier */

/** =============== ATTRIBUTE BYTE =============== */
/* Data size option */
#define DATA_SIZE_8_BIT         0x00
#define DATA_SIZE_16_BIT        0x02
#define DATA_SIZE_32_BIT        0x04
#define DATA_SIZE_64_BIT        0x06
#define DATA_SIZE_MASK          0x06

/* Time Stamp Size */
#define TIME_STAMP_32_BIT       0x00    /* Uncompressed 32-bit time stamp */
#define TIME_STAMP_64_BIT       0x01    /* Uncompressed 64-bit time stamp */
#define TIME_STAMP_SIZE_MASK    0x01

/********************************************************/
/*          SENSOR CONTROL REQ/RESP PACKET              */
/********************************************************/
/** =============== CONTROL BYTE =============== */
/* First 5 MS bits are same as Sensor Data Packet */
/* Data Format */
#define CTRL_PKT_DF_INTEGER     0x00
#define CTRL_PKT_DF_FIXPOINT    0x02
#define CTRL_PKT_DF_FLOAT       0x04
#define CTRL_PKT_DF_DOUBLE      0x06

/* A/P Definition same as Sensor Data Packet */

/** =============== ATTRIBUTE BYTE 1 =============== */
/* Sequence Number for request/response */
#define M_SequenceNum(sNum)     ((sNum) & 0x0F)

/** =============== ATTRIBUTE BYTE 2 =============== */
/* Parameter ID */
#define M_GetParamId(AttrByte2)         (AttrByte2)
#define M_SetParamId(id)                (id)

/* Parameter Identifier*/
#define PARAM_ID_ERROR_CODE_DATA		0x00
#define PARAM_ID_ENABLE                 0x01
#define PARAM_ID_BATCH                  0x02
#define PARAM_ID_FLUSH                  0x03
#define PARAM_ID_RANGE_RESOLUTION       0x04
#define PARAM_ID_POWER                  0x05
#define PARAM_ID_MINMAX_DELAY           0x06
#define PARAM_ID_FIFO_EVT_CNT           0x07
#define PARAM_ID_AXIS_MAPPING           0x08
#define PARAM_ID_CONVERSION_OFFSET      0x09
#define PARAM_ID_CONVERSION_SCALE       0x0A
#define PARAM_ID_SENSOR_NOISE           0x0B
#define PARAM_ID_TIMESTAMP_OFFSET       0x0C
#define PARAM_ID_ONTIME_WAKETIME        0x0D
#define PARAM_ID_HPF_LPF_CUTOFF         0x0E
#define PARAM_ID_SENSOR_NAME            0x0F
#define PARAM_ID_XYZ_OFFSET             0x10
#define PARAM_ID_F_SKOR_MATRIX          0x11
#define PARAM_ID_F_CAL_OFFSET           0x12
#define PARAM_ID_F_NONLINEAR_EFFECTS    0x13
#define PARAM_ID_BIAS_STABILITY         0x14
#define PARAM_ID_REPEATABILITY          0x15
#define PARAM_ID_TEMP_COEFF             0x16
#define PARAM_ID_SHAKE_SUSCEPTIBILIY    0x17
#define PARAM_ID_EXPECTED_NORM          0x18
#define PARAM_ID_VERISON                0x19
#define PARAM_ID_DYNAMIC_CAL_SCALE      0x1A
#define PARAM_ID_DYNAMIC_CAL_SKEW       0x1B
#define PARAM_ID_DYNAMIC_CAL_OFFSET     0x1C
#define PARAM_ID_DYNAMIC_CAL_ROTATION   0x1D
#define PARAM_ID_DYNAMIC_CAL_QUALITY    0x1E
#define PARAM_ID_DYNAMIC_CAL_SOURCE     0x1F
#define PARAM_ID_CONFIG_DONE            0x20


/* Parameter Size */
#define M_GetParamSize(AttrByte2)       ((AttrByte2) & 0x07)

/* Param data size */
#define PARAM_DATA_SZ_8_BIT             0x00
#define PARAM_DATA_SZ_16_BIT            0x01
#define PARAM_DATA_SZ_32_BIT            0x02
#define PARAM_DATA_SZ_64_BIT            0x03
#define PARAM_DATA_SZ_BOOL_FALSE        0x04
#define PARAM_DATA_SZ_BOOL_TRUE         0x05
#define PARAM_DATA_SZ_UNKNOWN           0x07

/* Byte extraction macros */
#define BYTE7(x)                        ((u8)((x >> 56) & 0xFF))
#define BYTE6(x)                        ((u8)((x >> 48) & 0xFF))
#define BYTE5(x)                        ((u8)((x >> 40) & 0xFF))
#define BYTE4(x)                        ((u8)((x >> 32) & 0xFF))
#define BYTE3(x)                        ((u8)((x >> 24) & 0xFF))
#define BYTE2(x)                        ((u8)((x >> 16) & 0xFF))
#define BYTE1(x)                        ((u8)((x >> 8) & 0xFF))
#define BYTE0(x)                        ((u8)(x & 0xFF))

/* 16-bit & 32-bit data combine macros */
#define BYTES_TO_SHORT(b1, b0)	((int16_t)(((int16_t)b1 << 8) | b0))

#define BYTES_TO_LONG_ARR(arr, ind)	BYTES_TO_LONG(arr[ind],\
	arr[ind+1], arr[ind+2], arr[ind+3])

#define BYTES_TO_LONG(b3, b2, b1, b0)      \
	((int32_t)(((int32_t)b3 << 24) | ((u32)b2 << 16) | ((u32)b1 << 8) | b0))

#define BYTES_TO_LONGLONG(b7, b6, b5, b4, b3, b2, b1, b0)      \
	((u64)(((u64)b7 << 56) | ((u64)b6 << 48) | ((u64)b5 << 40) | \
	((u64)b4 << 32) | ((u64)b3 << 24) | ((u64)b2 << 16) | \
	((u64)b1 << 8) | b0))
/*--------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*--------------------------------------------------------------------------*/

/* To be removed */
typedef union _TimeStamp {
	uint64_t	TS64;
	uint32_t	TS32[2];
	uint8_t		TS8[8];
} TimeStamp_t;
/* End to be removed */

/* Definition for quaternion data packets for internal usage */
typedef struct _QuaternionFixP {
	TimeStamp_t TimeStamp;
	int32_t Quat[4]; //W,X,Y,Z order
#if 0
	// Add these two fields to comply with Android spec.
	int32_t HeadingError;
	int32_t TiltError;
#endif
} QuaternionFixP_t;

typedef struct _OrientationFixP {
	TimeStamp_t TimeStamp;
	int32_t Pitch;
	int32_t Roll;
	int32_t Yaw;
} OrientationFixP_t;

typedef struct _ThreeAxisFixP {
	TimeStamp_t TimeStamp;
	int32_t Axis[3];
	uint8_t Accuracy;
} ThreeAxisFixP_t;

typedef struct _SignificantMotion {
	TimeStamp_t TimeStamp;
	uint8_t MotionDetected;
} SignificantMotion_t;

typedef struct _StepCounter {
	TimeStamp_t TimeStamp;
	uint64_t NumStepsTotal;
} StepCounter_t;

typedef struct _StepDetector {
	TimeStamp_t TimeStamp;
	uint8_t StepDetected;
} StepDetector_t;

typedef struct _UncalibratedFixP {
	TimeStamp_t TimeStamp;
	int32_t Axis[3];   //X,Y,Z order
	int32_t Offset[3]; //XOFF,YOFF,ZOFF order
} UncalibratedFixP_t;

typedef struct _CalibratedFixP {
	TimeStamp_t TimeStamp;
	int32_t Axis[3]; //X,Y,Z,
} CalibratedFixP_t;

// Possible combine this and the CalibratedFixP_t into a common data type.
typedef struct _TriAxisRawData {
	TimeStamp_t TStamp;
	int32_t Axis[3]; //X,Y,Z,
} TriAxisRawData_t;

/* Union of the structures that can be parsed out of Sensor Data packet */
typedef struct _SensorPacketTypes {
	union {
		TriAxisRawData_t RawSensor;
		UncalibratedFixP_t UncalFixP;
		CalibratedFixP_t CalFixP;
		QuaternionFixP_t QuatFixP;
		OrientationFixP_t OrientFixP;
		ThreeAxisFixP_t ThreeAxisFixP;
		SignificantMotion_t SigMotion;
		StepCounter_t StepCount;
		StepDetector_t StepDetector;
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
	uint8_t ControlByte;
	uint8_t SensorIdByte;
	uint8_t AttributeByte;
} HifSnsrPktQualifier_t;
/* Sensor control packet for Boolean parameters */
struct _HifSensorReadWriteReqPacket {
	struct _HifSensorPktQualifier Q;
	u8 AttrByte2;
};

#define SENSOR_READ_WRITE_REQ_PKT_SZ sizeof(struct _HifSensorReadWriteReqPacket)
/* Basic packet: Raw 16-bit data, 32-bit integer time stamp; No checksum */
typedef struct _HifSensorDataRaw {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[4];	//32-bit Time Stamp
	uint8_t DataRaw[6];	//3-Axis Raw 16-bit sensor data
} HifSensorDataRaw_t;

#define SENSOR_RAW_DATA_PKT_SZ  sizeof(HifSensorDataRaw_t)

/* Uncalibrated data packet:
 *   32-bit Fixedpoint uncalibrated;
 *   64-bit Fixedpoint time stamp;
 *   No Checksum */
typedef struct _HifUncalibratedFixPoint {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8];   //64-bit Time Stamp in fixed point format
	uint8_t Data[12];       //3-Axis Fixed point 32-bit uncalibrated data
	uint8_t Offset[12];     //3-Axis Fixed point 32-bit Offset
} HifUncalibratedFixPoint_t;

#define UNCALIB_FIXP_DATA_PKT_SZ	\
	(offsetof(HifUncalibratedFixPoint_t, Offset))
#define UNCALIB_FIXP_DATA_OFFSET_PKT_SZ sizeof(HifUncalibratedFixPoint_t)

/* Calibrated data packet: 32-bit Fixedpoint Calibrated;
64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifCalibratedFixPoint {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8];   //64-bit Time Stamp in fixed point format
	uint8_t Data[12];       //3-Axis Fixed point 32-bit calibrated data
} HifCalibratedFixPoint_t;

#define CALIBRATED_FIXP_DATA_PKT_SZ     sizeof(HifCalibratedFixPoint_t)

/* Three Axis Fixed Point data packet: 32-bit Fixedpoint Calibrated;
   64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifThreeAxisPktFixPoint {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8]; //64-bit Time Stamp in fixed point format
	uint8_t Data[12];   //3-Axis Fixed point 32-bit calibrated data
} HifThreeAxisPktFixPoint_t;

#define THREEAXIS_FIXP_DATA_PKT_SZ     sizeof(HifThreeAxisPktFixPoint_t)

/* SignificantMotion Fixed Point data packet: 32-bit Fixedpoint Calibrated;
   64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifSignificantMotionPktFixPoint {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8]; //64-bit Time Stamp in fixed point format
	uint8_t significantMotionDetected;
} HifSignificantMotionPktFixPoint_t;

#define SIGNIFICANTMOTION_FIXP_DATA_PKT_SZ	\
	sizeof(HifSignificantMotionPktFixPoint_t)

/* StepCounter data packet: 64-bit Fixedpoint Calibrated;
   64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifStepCounter {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8]; //64-bit Time Stamp in fixed point format
	uint8_t NumStepsTotal[8];
} HifStepCounter_t;

#define STEPCOUNTER_DATA_PKT_SZ		sizeof(HifStepCounter_t)

/* StepDetector data packet: 8-bit Fixedpoint Calibrated;
   64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifStepDetector {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8]; //64-bit Time Stamp in fixed point format
	uint8_t stepDetected;
} HifStepDetector_t;

#define STEPDETECTOR_DATA_PKT_SZ	sizeof(HifStepDetector_t)

/* Orientation Fixed Point data packet: 32-bit Fixedpoint Calibrated;
   64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifOrientationPktFixPoint {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8];		//!< Time in seconds
	uint8_t Pitch[4];		//!< pitch in degrees
	uint8_t Roll[4];		//!< roll in degrees
	uint8_t Yaw[4];			//!< yaw in degrees
} HifOrientationPktFixPoint_t;

#define ORIENTATION_FIXP_DATA_PKT_SZ	sizeof(HifOrientationPktFixPoint_t)

/* Quaternion data packet: 32-bit Fixedpoint quaternion;
   64-bit Fixedpoint time stamp; No Checksum */
typedef struct _HifQuaternionFixPoint {
	HifSnsrPktQualifier_t Q;
	uint8_t TimeStamp[8];   //64-bit Time Stamp in fixed point format
	uint8_t Data[16];       //4-Axis Fixed point 32-bit quaternion data
} HifQuaternionFixPoint_t;

#define QUATERNION_FIXP_DATA_PKT_SZ	sizeof(HifQuaternionFixPoint_t)

/* Sensor control packet for Boolean parameters */
typedef struct _HifSensorParamBool {
	HifSnsrPktQualifier_t Q;
	uint8_t AttrByte2;
	uint8_t Data;
} HifSensorParamBool_t;

#define SENSOR_ENABLE_REQ_PKT_SZ	sizeof(HifSensorParamBool_t)

/* Define union for all the host interface packet types */
typedef union _HostIFPackets {
	HifSensorDataRaw_t SensPktRaw;
	HifUncalibratedFixPoint_t UncalPktFixP;
	HifCalibratedFixPoint_t CalPktFixP;
	HifQuaternionFixPoint_t QuatPktFixP;
	HifSensorParamBool_t Enable;
	HifOrientationPktFixPoint_t OrientationFixP;
	HifThreeAxisPktFixPoint_t ThreeAxisFixp;
	HifSignificantMotionPktFixPoint_t SignificantMotion;
	HifStepCounter_t StepCounter;
	HifStepDetector_t StepDetector;
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
int16_t FormatSensorDataPktRaw( uint8_t *pDest,
	const TriAxisRawData_t *pSensData, uint8_t metaData,
	ASensorType_t sType, uint8_t subType );
int16_t FormatQuaternionPktFixP( uint8_t *pDest,
	const QuaternionFixP_t *pQuatData, ASensorType_t sType );
int16_t FormatUncalibratedPktFixP( uint8_t *pDest,
	const UncalibratedFixP_t *pUncalData, uint8_t metaData,
	ASensorType_t sType );
int16_t FormatCalibratedPktFixP( uint8_t *pDest,
	const CalibratedFixP_t *pCalData, ASensorType_t sType );
int16_t ParseHostInterfacePkt( SensorPacketTypes_t *pOut, uint8_t *pPacket,
	uint16_t pktSize );
int16_t FormatSensorEnableReq( uint8_t *pDest, uint8_t enable,
	ASensorType_t sType, uint8_t subType, uint8_t seqNum );
int16_t FormatStepDetectorPkt( uint8_t *pDest,
	const StepDetector_t *pStepDetectorData, ASensorType_t sType );
int16_t FormatStepCounterPkt( uint8_t *pDest,
	const StepCounter_t *pStepCounterData, ASensorType_t sType );
int16_t FormatThreeAxisPktFixP( uint8_t *pDest,
	const ThreeAxisFixP_t *p3AxisData, ASensorType_t sType );
int16_t FormatSignificantMotionPktFixP( uint8_t *pDest,
	const SignificantMotion_t *pSignificantMotionData,
	ASensorType_t sType );
int16_t FormatOrientationFixP( uint8_t *pDest,
	const OrientationFixP_t *pOrientationData, ASensorType_t sType );

#endif /* SENSOR_PACKETS_H */
/*--------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*--------------------------------------------------------------------------*/
