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
    mSensorId(sensorId)
{
    LOGE("Inside OSPQSensor");

    if (sensorType == SENSOR_TYPE_MAGNETIC_FIELD ||
            sensorType == SENSOR_TYPE_ORIENTATION ||
            sensorType == SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
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
    LOGE("@@@@ enable: [%d] - %d, %d sensortype : %d ", handle, enabled, mEnabled, mSensorType);
    if (flags && flags != mEnabled) {
        mEnabled = flags;
	OSPDaemon_sensor_enable(enabled, mSensorType);
    } else if (!flags) {
        mEnabled = flags;
	OSPDaemon_sensor_enable(enabled, mSensorType);
    }
    LOGE("@@@@ enable after: [%d] - %d %d", handle, enabled, mEnabled);
    return 0;
}

int OSPQSensor::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
	LOGE("@@@@ batch: [%d]  sensortype : %d sampling period 0x%llx MRL : 0x%llx ",
		handle, mSensorType, period_ns, timeout);
	OSPDaemon_batch(mSensorType, period_ns, timeout);
	return 0;
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

// TBD: Fix the handle events call to OSPQStepCounter OSPQSignificantMotion
int OSPQSensor::readEvents(sensors_event_t* data, int count)
{
    //LOGE("OSPQSensor::readEvents");
    int fc = 0, ret, i = 0;
    struct psen_data ld;
    if (count < 1)
	return -EINVAL;
    if (0 == mEnabled) {
        //LOGE("Sensor %s is not enabled", uinputName);
        return 0;
    }

    ret = OSPDaemon_get_sensor_data(mSensorType, &ld);
    if (0 == ret || -1 == ret) {
        //LOGE("Failed to get the sensor data ret is %d", ret);
        return fc;
    }

    //LOGE("Returning %d data values for sensor type %d", ret, mSensorType);
    data[fc].version   = sizeof(sensors_event_t);
    data[fc].sensor    = mSensorId;
    data[fc].type      = mSensorType;
    data[fc].timestamp = ld.ts;
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
