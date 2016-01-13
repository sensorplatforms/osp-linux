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

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <linux/input.h>
#include <utils/Atomic.h>
#include <cutils/log.h>
#include "local_log_def.h"

#include "sensors.h"

#include "OSPQSensor.h"
#include "OSPDaemon.h"
#include <semaphore.h>

/*****************************************************************************/


#define DELAY_OUT_TIME                  0x7FFFFFFF

#define LIGHT_SENSOR_POLLTIME           2000000000

#define SENSORS_ACCELERATION            (1<<ID_A)
#define SENSORS_ACCELERATION2           (1<<ID_A2)
#define SENSORS_MAGNETIC_FIELD          (1<<ID_M)
#define SENSORS_ORIENTATION             (1<<ID_O)
#define SENSORS_LIGHT                   (1<<ID_L)
#define SENSORS_PROXIMITY               (1<<ID_P)
#define SENSORS_GYROSCOPE               (1<<ID_GY)
#define SENSORS_PRESSURE                (1<<ID_PRESS)
#define SENSORS_TEMPERATURE             (1<<ID_TEMP)
#define SENSORS_ROTATION                (1<<ID_RV)
#define SENSORS_LINEAR_ACCELERATION     (1<<ID_LINACC)
#define SENSORS_GRAVITY                 (1<<ID_GRAV)
#define SENSORS_UNCALIBRATED_MAG        (1<<ID_UNCALIBRATED_MAG)
#define SENSORS_GAME_ROT_VEC            (1<<ID_GAME_ROT_VEC)
#define SENSORS_UNCALIBRATED_GYRO       (1<<ID_UNCALIBRATED_GYRO)
#define SENSORS_SIG_MOTION              (1<<ID_SIG_MOTION)
#define SENSORS_STEP_DETECTOR           (1<<ID_STEP_DETECTOR)
#define SENSORS_STEP_COUNTER            (1<<ID_STEP_COUNTER)
#define SENSORS_GEOMAGNETIC_ROT_VEC     (1<<ID_GEOMAGNETIC_ROT_VEC)

#define SENSORS_ACCELERATION_HANDLE     0
#define SENSORS_MAGNETIC_FIELD_HANDLE   1
#define SENSORS_ORIENTATION_HANDLE      2
#define SENSORS_LIGHT_HANDLE            3
#define SENSORS_PROXIMITY_HANDLE        4
#define SENSORS_GYROSCOPE_HANDLE        5
#define SENSORS_PRESSURE_HANDLE         6
#define SENSORS_TEMPERATURE_HANDLE      7
#define SENSORS_ACCELERATION_2_HANDLE   8
#define SENSORS_DEBUG_TESTING_HANDLE    9
#define SENSORS_ROTATION_VECTOR_HANDLE  10
#define SENSORS_LIN_ACCELERATION_HANDLE 11
#define SENSORS_GRAVITY_HANDLE          12

#define SENSORS_UNCALIBRATED_MAG_HANDLE       13
#define SENSORS_GAME_ROT_VEC_HANDLE           14
#define SENSORS_UNCALIBRATED_GYRO_HANDLE      15
#define SENSORS_SIG_MOTION_HANDLE             16
#define SENSORS_STEP_DETECTOR_HANDLE          17
#define SENSORS_STEP_COUNTER_HANDLE           18
#define SENSORS_GEOMAGNETIC_ROT_VEC_HANDLE    19


//For android:
#ifndef UNIX_PATH_MAX
# define UNIX_PATH_MAX 108
#endif

sem_t osp_sync;

/*****************************************************************************/
struct uinput_user_dev;

/* The SENSORS Module */
#if (PLATFORM_VERSION_MAJOR >= 5)
static struct sensor_t sSensorList[] = {
    { "OSP Accelerometer",
      "Sensor Platforms",
      1, SENSORS_ACCELERATION_HANDLE,
      SENSOR_TYPE_ACCELEROMETER, MAX_RANGE_ACCEL, RESOLUTION_ACCEL, 0.25f,
      MIN_DELAY_ACCEL, FIFO_RESV_EC_ACCEL, FIFO_MAX_EC_ACCEL, 0, 0,
      MAX_DELAY_ACCEL, FLAGS_ACCEL, { } },
    { "OSP Gyroscope",
      "Sensor Platforms",
      1, SENSORS_GYROSCOPE_HANDLE,
      SENSOR_TYPE_GYROSCOPE, MAX_RANGE_GYRO, RESOLUTION_GYRO, 6.5f,
      MIN_DELAY_GYRO, FIFO_RESV_EC_GYRO, FIFO_MAX_EC_GYRO, 0, 0,
      MAX_DELAY_GYRO, FLAGS_GYRO, { } },
    { "OSP Magnetometer",
      "Sensor Platforms",
      1, SENSORS_MAGNETIC_FIELD_HANDLE,
      SENSOR_TYPE_MAGNETIC_FIELD, MAX_RANGE_MAG, RESOLUTION_MAG, .6f,
      MIN_DELAY_MAG, FIFO_RESV_EC_MAG, FIFO_MAX_EC_MAG, 0, 0,
      MAX_DELAY_MAG, FLAGS_MAG, { } },
    { "OSP Orientation",
      "Sensor Platform",
      1, SENSORS_ORIENTATION_HANDLE,
      SENSOR_TYPE_ORIENTATION, MAX_RANGE_ORIENT, RESOLUTION_ORIENT, 7.35f,
      MIN_DELAY_ORIENT, FIFO_RESV_EC_ORIENT, FIFO_MAX_EC_ORIENT, 0, 0,
      MAX_DELAY_ORIENT, FLAGS_ORIENT, { } },
    { "OSP Rotation sensor",
      "Sensor Platforms",
      1, SENSORS_ROTATION_VECTOR_HANDLE,
      SENSOR_TYPE_ROTATION_VECTOR, MAX_RANGE_ROT, RESOLUTION_ROT, 7.35f,
      MIN_DELAY_ROT, FIFO_RESV_EC_ROT, FIFO_MAX_EC_ROT, 0, 0,
      MAX_DELAY_ROT, FLAGS_ROT, { } },
    { "OSP Linear Acceleration sensor",
      "Sensor Platforms",
      1, SENSORS_LIN_ACCELERATION_HANDLE,
      SENSOR_TYPE_LINEAR_ACCELERATION, MAX_RANGE_LIN_ACCEL, RESOLUTION_LIN_ACCEL,
      7.35f, MIN_DELAY_LIN_ACCEL, FIFO_RESV_EC_LIN_ACCEL, FIFO_MAX_EC_LIN_ACCEL,
      0, 0, MAX_DELAY_LIN_ACCEL, FLAGS_LIN_ACCEL, { } },
    { "OSP Gravity sensor",
      "Sensor Platforms",
      1, SENSORS_GRAVITY_HANDLE,
      SENSOR_TYPE_GRAVITY, MAX_RANGE_GRAV, RESOLUTION_GRAV, 7.35f,
      MIN_DELAY_GRAV, FIFO_RESV_EC_GRAV, FIFO_MAX_EC_GRAV, 0, 0,
      MAX_DELAY_GRAV, FLAGS_GRAV, { } },
    { "OSP Uncalibrated Magnetometer",
      "Sensor Platforms",
      1, SENSORS_UNCALIBRATED_MAG_HANDLE,
      SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, MAX_RANGE_MAG_UNCAL,
      RESOLUTION_MAG_UNCAL, .6f, MIN_DELAY_MAG_UNCAL, FIFO_RESV_EC_MAG_UNCAL,
      FIFO_MAX_EC_MAG_UNCAL, 0, 0, MAX_DELAY_MAG_UNCAL, FLAGS_MAG_UNCAL, { } },
    { "OSP Game Rot Vec",
      "Sensor Platforms",
      1, SENSORS_GAME_ROT_VEC_HANDLE,
      SENSOR_TYPE_GAME_ROTATION_VECTOR, MAX_RANGE_GAME_RV, RESOLUTION_GAME_RV,
      7.35f, MIN_DELAY_GAME_RV, FIFO_RESV_EC_GAME_RV, FIFO_MAX_EC_GAME_RV,
      0, 0, MAX_DELAY_GAME_RV, FLAGS_GAME_RV, { } },
    { "OSP Uncalibrated Gyroscope",
      "Sensor Platforms",
      1, SENSORS_UNCALIBRATED_GYRO_HANDLE,
      SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, MAX_RANGE_GYRO_UNCAL,
      RESOLUTION_GYRO_UNCAL, 6.5f, MIN_DELAY_GYRO_UNCAL, FIFO_RESV_EC_GYRO_UNCAL,
      FIFO_MAX_EC_GYRO_UNCAL, 0, 0, MAX_DELAY_GYRO_UNCAL, FLAGS_GYRO_UNCAL, { } },
    { "OSP Significant Motion",
      "Sensor Platforms",
       1, SENSORS_SIG_MOTION_HANDLE,
       SENSOR_TYPE_SIGNIFICANT_MOTION, MAX_RANGE_SIG_MOT, RESOLUTION_SIG_MOT,
       1.1f, MIN_DELAY_SIG_MOT, FIFO_RESV_EC_SIG_MOT, FIFO_MAX_EC_SIG_MOT,
       0, 0, MAX_DELAY_SIG_MOT, FLAGS_SIG_MOT, { } },
    { "OSP Step Detector",
      "Sensor Platforms",
      1, SENSORS_STEP_DETECTOR_HANDLE,
      SENSOR_TYPE_STEP_DETECTOR, MAX_RANGE_STEP_DET, RESOLUTION_STEP_DET,
      1.1f, MIN_DELAY_STEP_DET, FIFO_RESV_EC_STEP_DET, FIFO_MAX_EC_STEP_DET,
      0, 0, MAX_DELAY_STEP_DET, FLAGS_STEP_DET, { } },
    { "OSP Step Counter",
      "Sensor Platforms",
      1, SENSORS_STEP_COUNTER_HANDLE,
      SENSOR_TYPE_STEP_COUNTER, MAX_RANGE_STEP_COUNT, RESOLUTION_STEP_COUNT,
      1.1f, MIN_DELAY_STEP_COUNT, FIFO_RESV_EC_STEP_COUNT, FIFO_MAX_EC_STEP_COUNT,
      0, 0, MAX_DELAY_STEP_COUNT, FLAGS_STEP_COUNT, { } },
    { "OSP Geomagnetic Rot Vec",
      "Sensor Platforms",
      1, SENSORS_GEOMAGNETIC_ROT_VEC_HANDLE,
      SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR, MAX_RANGE_GEOM_RV,
      RESOLUTION_GEOM_RV, 7.35f, MIN_DELAY_GEOM_RV, FIFO_RESV_EC_GEOM_RV,
      FIFO_MAX_EC_GEOM_RV, 0, 0, MAX_DELAY_GEOM_RV, FLAGS_GEOM_RV, { } },
};
#else
static struct sensor_t sSensorList[] = {
    { "OSP Accelerometer",
      "Sensor Platforms",
      1, SENSORS_ACCELERATION_HANDLE,
      SENSOR_TYPE_ACCELEROMETER, RANGE_A, RESOLUTION_A, 0.25f, 20000, 0, 0, { } },
    { "OSP Gyroscope",
      "Sensor Platforms",
      1, SENSORS_GYROSCOPE_HANDLE,
      SENSOR_TYPE_GYROSCOPE, RANGE_GYRO, RESOLUTION_GYRO, 6.5f, 20000, 0, 0, { } },
    { "OSP magnetometer",
      "Sensor Platforms",
      1, SENSORS_MAGNETIC_FIELD_HANDLE,
      SENSOR_TYPE_MAGNETIC_FIELD, RANGE_M, RESOLUTION_M, .6f, 20000, 0, 0, { } },
    { "OSP Orientation",
      "Sensor Platform",
      1, SENSORS_ORIENTATION_HANDLE,
      SENSOR_TYPE_ORIENTATION, 360.0f, 0.00001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Rotation sensor",
      "Sensor Platforms",
      1, SENSORS_ROTATION_VECTOR_HANDLE,
      SENSOR_TYPE_ROTATION_VECTOR, 1.0f, 0.000000001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Linear Acceleration sensor",
      "Sensor Platforms",
      1, SENSORS_LIN_ACCELERATION_HANDLE,
      SENSOR_TYPE_LINEAR_ACCELERATION, RANGE_A, 0.00001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Gravity sensor",
      "Sensor Platforms",
      1, SENSORS_GRAVITY_HANDLE,
      SENSOR_TYPE_GRAVITY, RANGE_A, 0.00001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Uncalibrated Magnetometer",
      "Sensor Platforms",
      1, SENSORS_UNCALIBRATED_MAG_HANDLE,
      SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, RANGE_M, RESOLUTION_M, .6f, 20000, 0, 0, { } },
    { "OSP Game Rot Vec",
      "Sensor Platforms",
      1, SENSORS_GAME_ROT_VEC_HANDLE,
      SENSOR_TYPE_GAME_ROTATION_VECTOR,  1.0f, 0.000000001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Uncalibrated Gyroscope",
      "Sensor Platforms",
      1, SENSORS_UNCALIBRATED_GYRO_HANDLE,
      SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, RANGE_GYRO, RESOLUTION_GYRO, 6.5f, 20000, 0, 0, { } },
    { "OSP Significant Motion",
      "Sensor Platforms",
       1, SENSORS_SIG_MOTION_HANDLE,
       SENSOR_TYPE_SIGNIFICANT_MOTION, 1.0f, 1.0f, 1.1f, 0, 0, 0, {}},
    { "OSP Step Detector",
      "Sensor Platforms",
      1, SENSORS_STEP_DETECTOR_HANDLE,
      SENSOR_TYPE_STEP_DETECTOR, 1.0f, 1.0f, 1.1f, 0, 0, 0, {}},
    { "OSP Step Counter",
      "Sensor Platforms",
      1, SENSORS_STEP_COUNTER_HANDLE,
      SENSOR_TYPE_STEP_COUNTER, 1000.0f, 1.0f, 1.1f, 0, 0, 0, {}},
    { "OSP Geomagnetic Rot Vec",
      "Sensor Platforms",
      1, SENSORS_GEOMAGNETIC_ROT_VEC_HANDLE,
      SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR,  1.0f, 0.000000001f, 7.35f, 20000, 0, 0, { } },
};
#endif


static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device);



static struct sensor_t* get_sensor_list_item(int handle)
{
	struct sensor_t *sensor = NULL;
	int i = 0;
	for(; i< ARRAY_SIZE(sSensorList); i++) {
		if(handle == sSensorList[i].handle) {
			sensor = &sSensorList[i];
			break;
		}
	}
	return sensor;
}

static int sensors__get_sensors_list(struct sensors_module_t* module,
                                     struct sensor_t const** list)
{
    *list = sSensorList;
    return ARRAY_SIZE(sSensorList);
}

#if (PLATFORM_VERSION_MAJOR >= 6)
static int sensors__set_operation_mode(unsigned int mode)
{
	return -EPERM;
}
#endif


static struct hw_module_methods_t sensors_module_methods = {
        open: open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    common: {
        tag: HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id: SENSORS_HARDWARE_MODULE_ID,
        name: "Sensor module",
        author: "Sensor Platforms Inc.",
        methods: &sensors_module_methods,
        dso: 0,
        reserved: {0}
    },
    get_sensors_list: sensors__get_sensors_list,
#if (PLATFORM_VERSION_MAJOR >= 6)
	set_operation_mode: sensors__set_operation_mode,
#endif
};

struct sensors_poll_context_t {
    struct sensors_poll_device_1 device; // must be first

        sensors_poll_context_t();
        ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t delay_ns);
    int pollEvents(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    int flush(int handle);

private:
    enum {
        accel,
        gyro,
        mag,
        orientation,
        rot_vec,
        linear_accel,
        gravity,
        uncal_mag,
        game_rot_vec,
        uncal_gyro,
        sig_motion,
        step_detector,
        step_counter,
        geo_rot_vec,
        numSensorDrivers,
        numFds,
    };

    enum {
        eCmdEnable,
        eCmdDisable,
        eCmdSetDelay
    };

    static const size_t wake = numFds - 1;
    static const char WAKE_MESSAGE = 'W';
    struct pollfd mPollFds[numFds];
    int mWritePipeFd;
    SensorBase* mSensors[numSensorDrivers];

    int handleToDriver(int handle) const {
        switch (handle) {
        case ID_A:
            return accel;
        case ID_M:
            return mag;
        case ID_O:
            return orientation;

        case ID_GY:
            return gyro;

        case ID_RV:
            return rot_vec;

        case ID_LINACC:
            return linear_accel;

        case ID_GRAV:
            return gravity;

        case ID_UNCALIBRATED_MAG:
            return uncal_mag;

        case ID_GAME_ROT_VEC:
            return game_rot_vec;

        case ID_SIG_MOTION:
            return sig_motion;

        case ID_UNCALIBRATED_GYRO:
            return uncal_gyro;

        case ID_STEP_DETECTOR:
            return step_detector;

        case ID_STEP_COUNTER:
            return step_counter;

        case ID_GEOMAGNETIC_ROT_VEC:
            return geo_rot_vec;
        }
        return -EINVAL;
    }
};

void* OSPThreadLauncher(void*)
{
    int count = 6;
    char *args[] = {"", "-c", "/etc/N7.config", "-d", "0", "-p"};
    OSPDaemon_looper(count, args);

    return 0;
}

int launchOSPDaemon()
{
    pthread_t   mThrd;

    int err = pthread_create(&mThrd, NULL, &OSPThreadLauncher, NULL);
    if (0 != err) {
        LOGE("Failed to create the thread with error %s", strerror(errno));
        return 1;
    }

    err = sem_wait(&osp_sync);
    if (0 != err) {
        LOGE("Sem wait failed error - %s", strerror(errno));
        return 1;
    }

    LOGE("OSP init'ed");

    return 0;
}

sensors_poll_context_t::sensors_poll_context_t()
{
    char name[256];
    int v;
    int result;

    int err = sem_init(&osp_sync, 0, 0);
    if (0 != err) {
        LOGE("Sem init failed error -d %s", strerror(errno));
    }

    launchOSPDaemon();

    mSensors[accel]         = new OSPQSensor(ACCEL_UINPUT_NAME,
                                             ID_A,
                                             SENSOR_TYPE_ACCELEROMETER,
                                             true);
    mSensors[gyro]          = new OSPQSensor(GYRO_UINPUT_NAME,
                                             ID_GY,
                                             SENSOR_TYPE_GYROSCOPE,
                                             true);
    mSensors[mag]           = new OSPQSensor(MAG_UINPUT_NAME,
                                             ID_M,
                                             SENSOR_TYPE_MAGNETIC_FIELD,
                                             true);
    mSensors[rot_vec]       = new OSPQSensor("fm-rotation-vector",
                                             ID_RV,
                                             SENSOR_TYPE_ROTATION_VECTOR,
                                             true);
    mSensors[orientation]   = new OSPQSensor("fm-compass-orientation",
                                             ID_O,
                                             SENSOR_TYPE_ORIENTATION,
                                             true);
    mSensors[linear_accel]  = new OSPQSensor("fm-linear-acceleration",
                                             ID_LINACC,
                                             SENSOR_TYPE_LINEAR_ACCELERATION,
                                             true);
    mSensors[gravity]       = new OSPQSensor("fm-gravity",
                                             ID_GRAV,
                                             SENSOR_TYPE_GRAVITY,
                                             true);
    mSensors[uncal_mag] = new OSPQSensor("fm-uncalibrated-magnetometer",
                            ID_UNCALIBRATED_MAG,
                            SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
                            true);
     mSensors[game_rot_vec] = new OSPQSensor("fm-game-rotation-vector",
				 			ID_GAME_ROT_VEC,
				 			SENSOR_TYPE_GAME_ROTATION_VECTOR,
				 			true);
    mSensors[uncal_gyro] = new OSPQSensor("fm-uncalibrated-gyroscope",
							ID_UNCALIBRATED_GYRO,
							SENSOR_TYPE_GYROSCOPE_UNCALIBRATED,
							true);
    mSensors[sig_motion] = new OSPQSensor("fm-significant-motion",
                                               ID_SIG_MOTION,
                                               SENSOR_TYPE_SIGNIFICANT_MOTION,
					       FM_DECODE_VALUE_AS_INTEGER);
    mSensors[step_detector] = new OSPQSensor("fm-step-detector",
							 ID_STEP_DETECTOR,
							 SENSOR_TYPE_STEP_DETECTOR,
							 FM_DECODE_VALUE_AS_INTEGER);
    mSensors[step_counter] = new OSPQStepCounter("fm-step-counter",
						       ID_STEP_COUNTER,
						       SENSOR_TYPE_STEP_COUNTER,
						       FM_DECODE_VALUE_AS_INTEGER);
    mSensors[geo_rot_vec] = new OSPQSensor("fm-geomagnetic-rotation-vector",
							ID_GEOMAGNETIC_ROT_VEC,
							SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR,
							true);

    int wakeFds[2];
    result = pipe(wakeFds);
    LOGE_IF(result<0, "error creating wake pipe (%s)", strerror(errno));
    fcntl(wakeFds[0], F_SETFL, O_NONBLOCK); //@@ read should probably be blocking but since
    fcntl(wakeFds[1], F_SETFL, O_NONBLOCK); //we are using 'poll' it doesn't matter
    mWritePipeFd = wakeFds[1];

    mPollFds[wake].fd = wakeFds[0];
    mPollFds[wake].events = POLLIN;
    mPollFds[wake].revents = 0;
}

sensors_poll_context_t::~sensors_poll_context_t() {
    for (int i=0 ; i<numSensorDrivers ; i++) {
        delete mSensors[i];
    }
    close(mPollFds[wake].fd);
    close(mWritePipeFd);

    int err = sem_destroy(&osp_sync);
    if (0 != err) {
        LOGE("Sem destroy failed error - %s", strerror(errno));
        //return 1;
    }

    // TODO Delete the thread instance
}

int sensors_poll_context_t::activate(int handle, int enabled) {

    int index = handleToDriver(handle);
    if (index < 0) return index;

    LOGI("Sensor-activate - enum sensor %d enabled %d ", index, enabled);
    int err =  mSensors[index]->enable(handle, enabled);
    LOGE_IF(err != 0, "Sensor-activate failed (%d)", err);
    if (enabled && !err) {
        const char wakeMessage(WAKE_MESSAGE);
        int result = write(mWritePipeFd, &wakeMessage, 1);
        LOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
    }
    return err;
}

int sensors_poll_context_t::setDelay(int handle, int64_t delay_ns) {

    int index = handleToDriver(handle);
    if (index < 0) return index;

    int err= mSensors[index]->setDelay(handle, delay_ns);
    LOGE_IF(err < 0, "set delay failed (%d)", err);

    return err;
}

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
    int total = 0, nb;
    bool hadData = false;

    //while (count) {
        int err = sem_wait(&osp_sync);
        if (0 != err) {
            LOGE("Sem wait failed error - %s", strerror(errno));
            return 0;
        }

    do {
        hadData = false;
        for (int i = 0; count && i < numSensorDrivers; i++) {
            SensorBase *const sensor(mSensors[i]);
            nb = sensor->readEvents(data, count);
            //LOGE("For sensor Driver %d, events received %d", i, nb);
            if (nb > 0) {
                hadData = true;
                count -= nb;
                total += nb;
                data  += nb;
            }

            if (count <= 0)
                break;
        }
    } while (true == hadData && count > 0);

    //}

    return total;
}

int sensors_poll_context_t::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{

    int index = handleToDriver(handle);
    if (index < 0) return index;

    int err= mSensors[index]->batch(handle, flags, period_ns, timeout);
    LOGE_IF(err < 0, "batch failed (%d)", err);

    return err;
}

int sensors_poll_context_t::flush(int handle) {

	int index = handleToDriver(handle);
	struct sensor_t *sensor = get_sensor_list_item(handle);

	if (index < 0) return index;
	if(sensor == NULL) {
		LOGE("Can't find sensor in Sensor list");
		return -EINVAL;
	}
	if (sensor->flags & SENSOR_FLAG_ONE_SHOT_MODE) {
		LOGE("Can't send flush command for one shot sensors");
		return -EINVAL;
	}

	int err= mSensors[index]->flush(handle);
	LOGE_IF(err < 0, "flush failed (%d)", err);
	return err;
}


/*****************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx) {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
        int handle, int enabled) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
        int handle, int64_t ns)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->setDelay(handle, ns);
}

static int poll__poll(struct sensors_poll_device_t *dev,
        sensors_event_t* data, int count)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

static int poll__batch(struct sensors_poll_device_1 *dev,
                      int handle, int flags, int64_t period_ns, int64_t timeout)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->batch(handle, flags, period_ns, timeout);
}

static int poll__flush(struct sensors_poll_device_1* dev, int handle)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->flush(handle);
}

#if (PLATFORM_VERSION_MAJOR >= 6)
int poll__inject_sensor_data(struct sensors_poll_device_1 *dev, const sensors_event_t *data)
{
	return -EPERM;
}
#endif

/*****************************************************************************/

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device)
{
    int status = -EINVAL;
    sensors_poll_context_t *dev = new sensors_poll_context_t();

    memset(&dev->device, 0, sizeof(sensors_poll_device_t));

    dev->device.common.tag = HARDWARE_DEVICE_TAG;
    dev->device.common.version  = SENSORS_DEVICE_API_VERSION_1_3;
    dev->device.common.module   = const_cast<hw_module_t*>(module);
    dev->device.common.close    = poll__close;
    dev->device.activate        = poll__activate;
    dev->device.setDelay        = poll__setDelay;
    dev->device.poll            = poll__poll;
    dev->device.batch           = poll__batch;
    dev->device.flush           = poll__flush;
#if (PLATFORM_VERSION_MAJOR >= 6)
	dev->device.inject_sensor_data = poll__inject_sensor_data;
#endif

    *device = &dev->device.common;
    status = 0;

    return status;
}
