#
# This is a project Makefile. It is assumed the directory this Makefile resides
# in is a project subdirectory.
#

PROJECT_NAME := hikebuddy-test

ifndef PROJECT_PATH
PROJECT_PATH := $(abspath $(dir $(firstword $(MAKEFILE_LIST))))
export PROJECT_PATH
endif

ifndef IDF_PATH
IDF_PATH := $(PROJECT_PATH)/esp-idf
export IDF_PATH
endif

include $(IDF_PATH)/make/project.mk
