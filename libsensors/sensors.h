/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_SENSORS_H
#define ANDROID_SENSORS_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <linux/input.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

__BEGIN_DECLS

#define USE_MAG_RAW

/*****************************************************************************/

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define ID_A        (0)
#define ID_M        (1)
#define ID_O        (2)
#define ID_L        (3)
#define ID_P        (4)
#define ID_GY       (5)
#define ID_PRESS    (6)
#define ID_TEMP     (7)
#define ID_A2       (8)
#define ID_VA       (9)
#define ID_RV       (10)
#define ID_LINACC   (11)
#define ID_GRAV     (12)
#define ID_UNCALIBRATED_MAG    (13)
#define ID_GAME_ROT_VEC        (14)
#define ID_UNCALIBRATED_GYRO   (15)
#define ID_SIG_MOTION          (16)
#define ID_STEP_DETECTOR       (17)
#define ID_STEP_COUNTER        (18)
#define ID_GEOMAGNETIC_ROT_VEC (19)

/**** SENSOR NAMES corresponding to DRIVERS *****/

/* LibFM Sensors */
#define ACCEL_UINPUT_NAME       "fm-accelerometer"
#define GYRO_UINPUT_NAME        "fm-gyroscope"
#define MAG_UINPUT_NAME         "fm-magnetometer"
#define SENSOR_LFM_ROT_VECT     "fm-rotation-vector"

/*****************************************************************************/

/*
 * The SENSORS Module
 */

/* the GP2A is a binary proximity sensor that triggers around 5 cm on
 * this hardware */
#define PROXIMITY_THRESHOLD_GP2A    5.0f

/*****************************************************************************/

#define  FREEMOTIOND_EVENT_X        ABS_X
#define  FREEMOTIOND_EVENT_Y        ABS_Y
#define  FREEMOTIOND_EVENT_Z        ABS_Z

#define EVENT_TYPE_YAW              REL_RX
#define EVENT_TYPE_PITCH            REL_RY
#define EVENT_TYPE_ROLL             REL_RZ
#define EVENT_TYPE_ORIENT_STATUS    REL_WHEEL

#define EVENT_TYPE_PRESSURE         REL_HWHEEL
#define EVENT_TYPE_TEMPERATURE      ABS_MISC


#define EVENT_TYPE_ROT_X            ABS_X
#define EVENT_TYPE_ROT_Y            ABS_Y
#define EVENT_TYPE_ROT_Z            ABS_Z

#define EVENT_TYPE_PROXIMITY        ABS_DISTANCE
#define EVENT_TYPE_LIGHT            REL_MISC


#define MAX_RANGE_ACCEL		(39.2f)
#define RESOLUTION_ACCEL	(0.01915f)
#define MIN_DELAY_ACCEL		(16000)
#define MAX_DELAY_ACCEL		(1000000)
#define FIFO_RESV_EC_ACCEL	(0)
#define FIFO_MAX_EC_ACCEL	(200)
#define FLAGS_ACCEL		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_GYRO		(34.906f)
#define RESOLUTION_GYRO		(0.001067f)
#define MIN_DELAY_GYRO		(10000)
#define MAX_DELAY_GYRO		(1000000)
#define FIFO_RESV_EC_GYRO	(0)
#define FIFO_MAX_EC_GYRO	(200)
#define FLAGS_GYRO		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_MAG		(1300.0f)
#define RESOLUTION_MAG		(0.305f)
#define MIN_DELAY_MAG		(40000)
#define MAX_DELAY_MAG		(1000000)
#define FIFO_RESV_EC_MAG	(0)
#define FIFO_MAX_EC_MAG		(200)
#define FLAGS_MAG		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_ORIENT	(360.0f) // Default
#define RESOLUTION_ORIENT	(0.00001f) // Default
#define MIN_DELAY_ORIENT	(10000)
#define MAX_DELAY_ORIENT	(1000000)
#define FIFO_RESV_EC_ORIENT	(0)
#define FIFO_MAX_EC_ORIENT	(200)
#define FLAGS_ORIENT		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_ROT		(1.0f) // Default
#define RESOLUTION_ROT		(0.000000001f) // Default
#define MIN_DELAY_ROT		(10000)
#define MAX_DELAY_ROT		(1000000)
#define FIFO_RESV_EC_ROT	(0)
#define FIFO_MAX_EC_ROT		(200)
#define FLAGS_ROT		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_LIN_ACCEL	(39.2f)
#define RESOLUTION_LIN_ACCEL	(0.01915f)
#define MIN_DELAY_LIN_ACCEL	(16000)
#define MAX_DELAY_LIN_ACCEL	(1000000)
#define FIFO_RESV_EC_LIN_ACCEL	(0)
#define FIFO_MAX_EC_LIN_ACCEL	(200)
#define FLAGS_LIN_ACCEL		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_GRAV		(39.2f)
#define RESOLUTION_GRAV		(0.01915f)
#define MIN_DELAY_GRAV		(16000)
#define MAX_DELAY_GRAV		(1000000)
#define FIFO_RESV_EC_GRAV	(0)
#define FIFO_MAX_EC_GRAV	(200)
#define FLAGS_GRAV		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_MAG_UNCAL	(1300.0f)
#define RESOLUTION_MAG_UNCAL	(0.305f)
#define MIN_DELAY_MAG_UNCAL	(40000)
#define MAX_DELAY_MAG_UNCAL	(1000000)
#define FIFO_RESV_EC_MAG_UNCAL	(0)
#define FIFO_MAX_EC_MAG_UNCAL	(200)
#define FLAGS_MAG_UNCAL		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_GAME_RV	(1.0f) // Default
#define RESOLUTION_GAME_RV	(0.000000001f) // Default
#define MIN_DELAY_GAME_RV	(20000)
#define MAX_DELAY_GAME_RV	(1000000)
#define FIFO_RESV_EC_GAME_RV	(0)
#define FIFO_MAX_EC_GAME_RV	(200)
#define FLAGS_GAME_RV		(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_GYRO_UNCAL	(34.906f)
#define RESOLUTION_GYRO_UNCAL	(0.001067f)
#define MIN_DELAY_GYRO_UNCAL	(10000)
#define MAX_DELAY_GYRO_UNCAL	(1000000)
#define FIFO_RESV_EC_GYRO_UNCAL	(0)
#define FIFO_MAX_EC_GYRO_UNCAL	(200)
#define FLAGS_GYRO_UNCAL	(SENSOR_FLAG_CONTINUOUS_MODE)

#define MAX_RANGE_SIG_MOT	(1.0f) // Default
#define RESOLUTION_SIG_MOT	(1.0f) // Default
#define MIN_DELAY_SIG_MOT	(0) // NA
#define MAX_DELAY_SIG_MOT	(1000000)
#define FIFO_RESV_EC_SIG_MOT	(0)
#define FIFO_MAX_EC_SIG_MOT	(200)
#define FLAGS_SIG_MOT		(SENSOR_FLAG_ONE_SHOT_MODE)

#define MAX_RANGE_STEP_DET	(1.0f) // Default
#define RESOLUTION_STEP_DET	(1.0f) // Default
#define MIN_DELAY_STEP_DET	(0) // NA
#define MAX_DELAY_STEP_DET	(1000000)
#define FIFO_RESV_EC_STEP_DET	(0)
#define FIFO_MAX_EC_STEP_DET	(200)
#define FLAGS_STEP_DET		(SENSOR_FLAG_SPECIAL_REPORTING_MODE)

#define MAX_RANGE_STEP_COUNT	(1000.0f) // Default
#define RESOLUTION_STEP_COUNT	(1.0f) // Default
#define MIN_DELAY_STEP_COUNT	(0) // NA
#define MAX_DELAY_STEP_COUNT	(1000000)
#define FIFO_RESV_EC_STEP_COUNT	(0)
#define FIFO_MAX_EC_STEP_COUNT	(200)
#define FLAGS_STEP_COUNT	(SENSOR_FLAG_ON_CHANGE_MODE)

#define MAX_RANGE_GEOM_RV	(1.0f) // Default
#define RESOLUTION_GEOM_RV	(0.000000001f) // Default
#define MIN_DELAY_GEOM_RV	(20000)
#define MAX_DELAY_GEOM_RV	(1000000)
#define FIFO_RESV_EC_GEOM_RV	(0)
#define FIFO_MAX_EC_GEOM_RV	(200)
#define FLAGS_GEOM_RV		(SENSOR_FLAG_CONTINUOUS_MODE)


/*****************************************************************************/

__END_DECLS

#endif  // ANDROID_SENSORS_H
