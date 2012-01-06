##
# Project Name : Samsung Android Build System
#
# Copyright 2009 by Samsung Electronics, Inc.
# All rights reserved.
#
# Project Description :
##

##
# File Name : Android.mk
#
# File Description :
#
# Author : Seo, Won Hee (#4579, wonhee48.seo@samsung.com)
# Dept : Symbian Lab. (Open OS S/W Group)
# Created Date : 04/Dec./2009
# Version : Baby-Raccoon
##

ifeq ($(strip $(SEC_BOARD_USE_DPRAM)),true)
#ifneq ($(strip $(SEC_BOARD_USE_DPRAM_FLASHLESS)),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE        := sec_dpram_kobject
LOCAL_KERNEL_OBJECT := dpram
LOCAL_MODULE_CLASS  := SEC_KERNEL_OBJECTS
LOCAL_SRC_FILES := \
    dpram.c
LOCAL_ADDITIONAL_DEPENDENCIES := \
    dpram.h

include vendor/samsung/build/sec_kobject.mk

#endif   ## $(SEC_BOARD_USE_DPRAM_FLASHLESS)
endif   ## $(SEC_BOARD_USE_DPRAM)

