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

#ifneq ($(TARGET_SIMULATOR),true)

# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
#include $(NVIDIA_DEFAULTS)
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
# Figure out the current Android version
PLATFORM_VERSION_MAJOR := $(word 1, $(subst ., ,$(PLATFORM_VERSION)))
LOCAL_SHARED_LIBRARIES := liblog libcutils libdl
LOCAL_MODULE_OWNER := audience 
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SRC_FILES := \
                sensors.cpp 			\
                SensorBase.cpp			\
                SensorHubInputSensor.cpp	\
                InputEventReader.cpp

LOCAL_MODULE := sensors.grouper
LOCAL_MODULE_TAGS := debug
LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\" \
                -DPLATFORM_VERSION_MAJOR=$(PLATFORM_VERSION_MAJOR)

include $(BUILD_SHARED_LIBRARY)
#include $(NVIDIA_SHARED_LIBRARY)

#endif # !TARGET_SIMULATOR
