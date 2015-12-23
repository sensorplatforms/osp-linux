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
    mIsFlushCalled(false),
    mEventsAreFloat(evtFloat),
    mHasPendingEvent(false),
    uinputName(uinputName),
    mSensorType(sensorType),
    mSensorId(sensorId),
    mHandle(-1),
    mHostFirstReportedTime(0),
    mSHFirstReportedTime(0.0),
    mNumPacketsRecv(0),
	mIsFlushEventSent(false)
{
	Qscale = (1<<24);
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
	int ret;
    LOGE("@@@@ enable: [%d] - %d, %d sensortype : %d ", handle, enabled, mEnabled, mSensorType);
    if (flags && flags != mEnabled) {
        mEnabled = flags;
	ret = OSPDaemon_sensor_enable(enabled, mSensorType);
	/* Reset the First reported time stamp here to get new elapsed time */
	mHostFirstReportedTime = android::elapsedRealtimeNano();
	mSHFirstReportedTime = 0.0;
	mNumPacketsRecv = 0;
	mIsFlushEventSent = false;
    } else if (!flags) {
	mEnabled = flags;
	ret = OSPDaemon_sensor_enable(enabled, mSensorType);
	mSHFirstReportedTime = 0.0;
	mHostFirstReportedTime = 0;
    }
    return ret;
}

int OSPQSensor::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
	int ret;
	LOGE("@@@@ batch: [%d]  sensortype : %d sampling period 0x%llx MRL : 0x%llx ",
		handle, mSensorType, period_ns, timeout);
	ret = OSPDaemon_batch(mSensorType, period_ns, timeout);
	LOGE("@@@@ batch after: sensortype[%d] ret : %d\n", mSensorType, ret);
	return ret;
}

int OSPQSensor::flush(int32_t handle)
{
	int ret;
	LOGE("@@@@ flush: [%d]  sensortype : %d", handle, mSensorType);
	ret = OSPDaemon_flush(mSensorType);
	LOGE("@@@@ flush after: sensortype[%d] ret : %d\n", mSensorType, ret);
	mIsFlushCalled = true;
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
	if (0 == mEnabled || true == mIsFlushEventSent) {
		/* LOGE("Sensor %s is not enabled", uinputName);*/
		return 0;
	}
	if (mIsFlushCalled == true){
		data[fc].version = META_DATA_VERSION;
		data[fc].type = SENSOR_TYPE_META_DATA;
		data[fc].meta_data.what = META_DATA_FLUSH_COMPLETE;
		LOGI("Flush Sending flush for sensor %d", mHandle);
		data[fc].meta_data.sensor = mHandle;
		fc++;
		mIsFlushCalled = false;
		mIsFlushEventSent = true;
		return fc;
	}
	ret = OSPDaemon_get_sensor_data(mSensorType, &ld);
	if (0 == ret || -1 == ret) {
		LOGE("Failed to get the sensor data ret is %d", ret);
		return fc;
	}
	if (ld.ts < mSHFirstReportedTime) {
	/* Something went wrong, let's reset*/
		mHostFirstReportedTime = 0;
	}

	if (0.0 == mSHFirstReportedTime) {
		mSHFirstReportedTime = (double)ld.ts;
		mSHFirstReportedTime = mSHFirstReportedTime/ (double)Qscale;
	}
	tsq24 = ld.ts / (double)Qscale;
	delta = tsq24 - mSHFirstReportedTime; /* in seconds.*/
	delta = delta * 1000000000;
	LOGE("After delta %lf", delta);
	data[fc].version   = sizeof(sensors_event_t);
	data[fc].sensor    = mSensorId;
	data[fc].type      = mSensorType;
	data[fc].timestamp = mHostFirstReportedTime + (int64_t)delta;
	mNumPacketsRecv++;
	LOGE("SensorHAL timestamp is %lld , sensor : %s\n", data[fc].timestamp, uinputName);
	for (i = 0; i < ret; i++)
		data[fc].data[i] = (float)ld.val[i] / (float)Qscale;
	fc++;
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
