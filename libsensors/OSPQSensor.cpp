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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <strings.h>
#include <sys/select.h>
#include <cutils/log.h>
#include <utils/SystemClock.h>

#include "local_log_def.h"

#include "OSPQSensor.h"

#include "OSPDaemon.h"

OSPQSensor::OSPQSensor(const char* uinputName,
        int32_t sensorId,
        int32_t sensorType,
        bool evtFloat) :
    SensorBase(NULL, uinputName, 1),
    mEnabled(false),
    mEventsAreFloat(evtFloat),
    mHasPendingEvent(false),
    uinputName(uinputName),
    mSensorType(sensorType),
    mSensorId(sensorId),
    mHandle(-1),
    mHostFirstReportedTime(0),
    mSHFirstReportedTime(0.0),
    mNumPacketsRecv(0)
{
	if (sensorType == SENSOR_TYPE_MAGNETIC_FIELD ||
		sensorType == SENSOR_TYPE_ORIENTATION ||
		sensorType == SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
			LOGI("sensortype : %d\n", sensorType);
			Qscale = (1<<12);
	} else if (sensorType == SENSOR_TYPE_STEP_DETECTOR ||
				sensorType == SENSOR_TYPE_SIGNIFICANT_MOTION) {
				Qscale = 1;
	} else {
			Qscale = (1<<24);
	}
}

OSPQSensor::~OSPQSensor()
{
    if (mEnabled) {
        enable(0, 0);
    }
}

int OSPQSensor::enable(int32_t handle, int enabled)
{
    bool flags = enabled ? true : false;
    char enablePath[512];
    int ret = 0;
    LOGI("@@@@ sensor: [%s] enabled - %d sensortype : %d ", uinputName, enabled, mSensorType);
    if (flags && flags != mEnabled) {
        mEnabled = flags;
	ret = OSPDaemon_sensor_enable(enabled, mSensorType);
	/* Reset the First reported time stamp here to get new elapsed time */
	mHostFirstReportedTime = android::elapsedRealtimeNano();
	mSHFirstReportedTime = 0.0;
	mNumPacketsRecv = 0;
    } else if (!flags) {
	mEnabled = flags;
	ret = OSPDaemon_sensor_enable(enabled, mSensorType);
	mSHFirstReportedTime = 0.0;
	mHostFirstReportedTime = 0;
	LOGE("Disable - Num packets received is %d", mNumPacketsRecv);
    }
    return ret;
}

int OSPQSensor::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
	int ret;
	switch (mSensorType){
	case SENSOR_ACCELEROMETER:
		if (period_ns <= (MIN_DELAY_ACCEL * 1000))/*DELAY is in usec period_ns in nsec*/
			period_ns = MIN_DELAY_ACCEL * 1000;
		if (period_ns >= (MAX_DELAY_ACCEL * 1000))
			period_ns = MAX_DELAY_ACCEL * 1000;
		break;
	case SENSOR_MAGNETIC_FIELD:
		if (period_ns <= (MIN_DELAY_MAG * 1000))
			period_ns = MIN_DELAY_MAG * 1000;
		if (period_ns >= (MAX_DELAY_MAG * 1000))
			period_ns = MAX_DELAY_MAG * 1000;
		break;
	case SENSOR_ORIENTATION:
		if (period_ns <= (MIN_DELAY_ORIENT * 1000))
			period_ns = MIN_DELAY_ORIENT * 1000;
		if (period_ns >= (MAX_DELAY_ORIENT * 1000))
			period_ns = MAX_DELAY_ORIENT * 1000;
		break;
	case SENSOR_GYROSCOPE:
		if (period_ns <= (MIN_DELAY_GYRO * 1000))
			period_ns = MIN_DELAY_GYRO * 1000;
		if (period_ns >= (MAX_DELAY_GYRO * 1000))
			period_ns = MAX_DELAY_GYRO * 1000;
		break;
	case SENSOR_GRAVITY:
		if (period_ns <= (MIN_DELAY_GRAV * 1000))
			period_ns = MIN_DELAY_GRAV * 1000;
		if (period_ns >= (MAX_DELAY_GRAV * 1000))
			period_ns = MAX_DELAY_GRAV * 1000;
		break;
	case SENSOR_LINEAR_ACCELERATION:
		if (period_ns <= (MIN_DELAY_LIN_ACCEL * 1000))
			period_ns = MIN_DELAY_LIN_ACCEL * 1000;
		if (period_ns >= (MAX_DELAY_LIN_ACCEL * 1000))
			period_ns >= MAX_DELAY_LIN_ACCEL * 1000;
		break;
	case SENSOR_ROTATION_VECTOR:
		if (period_ns <= (MIN_DELAY_ROT * 1000))
			period_ns = MIN_DELAY_ROT * 1000;
		if (period_ns >= (MAX_DELAY_ROT * 1000))
			period_ns >= MAX_DELAY_ROT * 1000;
		break;
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		if (period_ns <= (MIN_DELAY_MAG_UNCAL * 1000))
			period_ns = MIN_DELAY_MAG_UNCAL * 1000;
		if (period_ns >= (MAX_DELAY_MAG_UNCAL * 1000))
			period_ns >= MAX_DELAY_MAG_UNCAL * 1000;
		break;
	case SENSOR_GAME_ROTATION_VECTOR:
		if (period_ns <= (MIN_DELAY_GAME_RV * 1000))
			period_ns = MIN_DELAY_GAME_RV * 1000;
		if (period_ns >= (MAX_DELAY_GAME_RV * 1000))
			period_ns >= MAX_DELAY_GAME_RV * 1000;
		break;
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		if (period_ns <= (MIN_DELAY_GYRO_UNCAL * 1000))
			period_ns = MIN_DELAY_GYRO_UNCAL * 1000;
		if (period_ns >= (MAX_DELAY_GYRO_UNCAL * 1000))
			period_ns >= MAX_DELAY_GYRO_UNCAL * 1000;
		break;
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		if (period_ns <= (MIN_DELAY_GEOM_RV * 1000))
			period_ns = MIN_DELAY_GEOM_RV * 1000;
		if (period_ns >= (MAX_DELAY_GEOM_RV * 1000))
			period_ns >= MAX_DELAY_GEOM_RV * 1000;
		break;
	default:
			break;
	}
	LOGI("@@@@ batch: [%s]  sensortype : %d sampling period %lld MRL : %lld",
		uinputName, mSensorType, period_ns, timeout);
	ret = OSPDaemon_batch(mSensorType, period_ns, timeout);
	mMRL = timeout;
	return ret;
}

int OSPQSensor::flush(int32_t handle)
{
	int ret;
	LOGI("@@@@ flush: [%s]  sensortype : %d", uinputName, mSensorType);
	if (mSensorType == SENSOR_TYPE_SIGNIFICANT_MOTION){
		LOGE("Can't send flush command for one shot sensors");
		return -EINVAL;
	}
	ret = OSPDaemon_flush(mSensorType);
	mHandle = handle;
	return ret;
}

bool OSPQSensor::handleEvent(input_event const * event,
        sensors_event_t* androidData)
{
    return false;
}

bool OSPQSensor::hasPendingEvents() const
{
    return mHasPendingEvent;
}

int OSPQSensor::setDelay(int32_t handle, int64_t delay_ns)
{
    LOGI("@@@@ setDelay: [%d] - %lldms", handle, (delay_ns/1000000));
    return 0;
}

int OSPQSensor::readEvents(sensors_event_t* data, int count)
{
	int fc = 0, ret, i = 0;
	struct psen_data ld;
	double tsq24, delta;
	if (count < 1)
	return -EINVAL;
	if (0 == mEnabled) {
		/* LOGE("Sensor %s is not enabled", uinputName);*/
		return 0;
	}
	ret = OSPDaemon_get_sensor_data(mSensorType, &ld);
	if (0 == ret || -1 == ret) {
		/* LOGE("Failed to get the sensor data ret is %d", ret);*/
		return fc;
	}
	if (ld.flush_completed == 1) {
		LOGI("sensor HAL received flush complete event for sensor %d \n", mHandle);
		data[fc].version = META_DATA_VERSION;
		data[fc].type = SENSOR_TYPE_META_DATA;
		data[fc].meta_data.what = META_DATA_FLUSH_COMPLETE;
		data[fc].meta_data.sensor = mHandle;
		fc++;
		return fc;
	}

	if (((double)ld.ts / HIF_TSCALE) < mSHFirstReportedTime) {
		/* Something went wrong, let's reset*/
		LOGE("Current timestamp is lesser than first reported one");
		LOGE("CurTS %lld FRTS %lf", ld.ts, mSHFirstReportedTime);
		mSHFirstReportedTime = 0.0;
	}

	if (0.0 == mSHFirstReportedTime) {
		mSHFirstReportedTime = (double)ld.ts;
		mSHFirstReportedTime = mSHFirstReportedTime/ (double)HIF_TSCALE;
	}
	tsq24 = ld.ts / (double)HIF_TSCALE;
	delta = tsq24 - mSHFirstReportedTime; /* in seconds.*/
	delta = delta * 1000000000;
	//LOGE("After delta %lf", delta);
	data[fc].version   = sizeof(sensors_event_t);
	data[fc].sensor    = mSensorId;
	data[fc].type      = mSensorType;
	data[fc].timestamp = mHostFirstReportedTime + (int64_t)delta;
	mNumPacketsRecv++;
	//LOGE("SensorHAL timestamp is %lld , sensor : %s\n", data[fc].timestamp, uinputName);
	for (i = 0; i < ret; i++)
		data[fc].data[i] = (float)ld.val[i] / (float)Qscale;
	fc++;

	if (data[fc].timestamp > (mHostFirstReportedTime + mMRL) && mMRL != 0) {
		LOGE("TS Exceeds MRL duration");
		LOGE("mHostFirstReportedTime %lld", mHostFirstReportedTime);
		LOGE("mSHFirstReportedTime %lf", mSHFirstReportedTime);
		LOGE("delta %lf", delta);
		LOGE("ld.ts %lld", ld.ts);
		LOGE("tsq24 %lf", tsq24);
		LOGE("mMRL %lld", mMRL);
	}

    return fc;
}

//! use the handling to generate the necessary sideeffect of making SIG_MOTION a oneshot
bool OSPQSignificantMotion::handleEvent(input_event const * event,
	sensors_event_t* androidData)
{
    LOGE("@@@@ custom handling of SIG_MOTION");

    if (event->code == REL_X) {
        LOGE("@@@@ auto-disable SIG_MOTION: [%d]", androidData->sensor);
    }
    androidData->data[0] = 1.0;

    return false;
}

bool OSPQStepCounter::handleEvent(input_event const * event, sensors_event_t* androidData) {
    if (event->code == FREEMOTIOND_EVENT_X) {
        androidData->u64.step_counter= event->value;
    }

    return true;
}
