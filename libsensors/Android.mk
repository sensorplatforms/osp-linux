# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


LOCAL_PATH := $(call my-dir)

ifneq ($(TARGET_SIMULATOR),true)

# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
# Figure out the current Android version
PLATFORM_VERSION_MAJOR := $(word 1, $(subst ., ,$(PLATFORM_VERSION)))
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog libcutils libdl libutils
LOCAL_STATIC_LIBRARIES := libOSP
LOCAL_MODULE_OWNER := audience 

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../ospd/ \
		    $(LOCAL_PATH)/../include

OSPD_SRC_FILES := ../ospd/OSPDaemon_queue.c \
		  ../ospd/OSPDaemon.c \
		  ../ospd/OSPDaemon_iio.c \
		  ../ospd/OSPDaemon_config.c \
		  ../ospd/OSPDaemon_input.c \
		  ../ospd/OSPDaemon_inputreader.c \
		  ../ospd/OSPDaemon_filecsv.c \
		  ../ospd/OSPDaemon_pm.c \
		  ../ospd/OSPDaemon_driver.c \
		  ../ospd/OSPDaemon_imu_config.c \
		  ../ospd/OSPDaemon_queue_driver.c

LOCAL_SRC_FILES := \
                sensors.cpp 			\
                SensorBase.cpp			\
		OSPQSensor.cpp			\
		$(OSPD_SRC_FILES)

LOCAL_MODULE := sensors.grouper
LOCAL_MODULE_TAGS := debug
LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\" \
                -DPLATFORM_VERSION_MAJOR=$(PLATFORM_VERSION_MAJOR) \
		-Wall -g -DANDROID_DEBUG

include $(BUILD_SHARED_LIBRARY)

endif # !TARGET_SIMULATOR
