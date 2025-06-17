COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= aux_fifo_watermark_header_mode.c

API_LOCATION ?= ../..

COMMON_LOCATION ?= ..

BMM150_SOURCE ?= ../../../bmm150

C_SRCS += \
$(API_LOCATION)/bmi2.c \
$(API_LOCATION)/bmi270_dsd.c \
$(BMM150_SOURCE)/bmm150.c \
$(COMMON_LOCATION)/common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
$(BMM150_SOURCE) \
$(COMMON_LOCATION)/common 

ifndef TARGET
$(error TARGET is not defined; please specify a target)
endif
INVALID_TARGET = PC
$(if $(filter $(TARGET),PC), $(error TARGET has an invalid value '$(TARGET)'; 'PC' is not a valid target. Please use a MCU target (eg. MCU_APP30).), )

include $(COINES_INSTALL_PATH)/coines.mk