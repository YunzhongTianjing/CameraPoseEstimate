LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE:=rpp

LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/librpp/

LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/*.cpp)
LOCAL_SRC_FILES += $(wildcard $(LOCAL_PATH)/*.c)
LOCAL_SRC_FILES += $(wildcard $(LOCAL_PATH)/librpp/*.cpp)
LOCAL_SRC_FILES := $(LOCAL_SRC_FILES:$(LOCAL_PATH)/%=%)
LOCAL_LDLIBS +=  -llog -ldl -lz

include $(BUILD_SHARED_LIBRARY)