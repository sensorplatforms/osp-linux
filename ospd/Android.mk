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

include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE := OSPDaemon
LOCAL_SRC_FILES := OSPDaemon_queue.c \
		   OSPDaemon.c \
		   OSPDaemon_iio.c \
		   OSPDaemon_config.c \
		   OSPDaemon_input.c \
		   OSPDaemon_inputreader.c \
		   OSPDaemon_filecsv.c \
		   OSPDaemon_pm.c \
		   OSPDaemon_driver.c
LOCAL_CFLAGS := -Wall -g -DANDROID_DEBUG
LOCAL_LDLIBS := -Wall -g
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../include
LOCAL_STATIC_LIBRARIES := libOSP
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_MODULE_OWNER := audience
include $(BUILD_EXECUTABLE)

endif # !TARGET_SIMULATOR

