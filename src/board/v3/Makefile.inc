
ifeq ($(PIKSI_REV),)
  PIKSI_REV=prod
endif

USE_OPT += -mfpu=neon -mfloat-abi=hard

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the ARM System/User stack. This
# stack is the stack used by the main() thread.
ifeq ($(USE_SYSTEM_STACKSIZE),)
# TODO: The original value here was 0x10000 but had to be increased to fix
# master to unblock some project work. The original error was caused by some
# undefined increase in stack usage caused by some change in 
# https://github.com/swift-nav/starling/pull/5603/ . The impact of this change
# needs to be understood, fixed, and then this value can be reduced back to 
# something more sensible.
  USE_SYSTEM_STACKSIZE = 0x100000
endif

# Stack size to the allocated to the ARM IRQ stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_IRQ_STACKSIZE = 0x800
endif

# Stack size to the allocated to the ARM FIQ stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_FIQ_STACKSIZE),)
  USE_FIQ_STACKSIZE = 64
endif

# Stack size to the allocated to the ARM Supervisor stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_SUPERVISOR_STACKSIZE),)
  USE_SUPERVISOR_STACKSIZE = 8
endif

# Stack size to the allocated to the ARM Undefined stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_UND_STACKSIZE),)
  USE_UND_STACKSIZE = 8
endif

# Stack size to the allocated to the ARM Abort stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_ABT_STACKSIZE),)
  USE_ABT_STACKSIZE = 8
endif

# Enables the use of FPU.
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# NOTE: this assumes libraries are not PIKSI_REV sensitive
ifeq ($(LIB_BUILDFOLDER),)
  LIB_BUILDFOLDER = build_$(PIKSI_HW)
endif

ifeq ($(OPENAMP_BUILDDIR),)
  OPENAMP_BUILDDIR=$(SWIFTNAV_ROOT)/open-amp/$(LIB_BUILDFOLDER)
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

include $(SWIFTNAV_ROOT)/src/board/v3/$(PIKSI_REV)/Makefile.inc

include $(CHIBIOS)/os/common/ports/ARM/compilers/GCC/mk/startup_zynq7000.mk
include $(CHIBIOS)/os/hal/ports/ZYNQ7000/platform.mk
include $(CHIBIOS)/os/rt/ports/ARM/compilers/GCC/mk/port_generic.mk

BOARDDIR := $(SWIFTNAV_ROOT)/src/board/v3
BOARDINC := $(REVINC) \
            $(BOARDDIR) \
            $(SWIFTNAV_ROOT)/open-amp/lib/include

BOARDSRC := \
        $(REVSRC) \
        $(BOARDDIR)/board.o \
        $(BOARDDIR)/error.o \
        $(BOARDDIR)/frontend.o \
        $(BOARDDIR)/clk_dac.o \
        $(BOARDDIR)/init.o \
        $(BOARDDIR)/io_support.o \
        $(BOARDDIR)/xadc.o \
        $(BOARDDIR)/factory_data.o \
        $(BOARDDIR)/manage_pv.o \
        $(BOARDDIR)/imu.o \
        $(BOARDDIR)/nap/axi_dma.o \
        $(BOARDDIR)/nap/grabber.o \
        $(BOARDDIR)/nap/nap_common.o \
        $(BOARDDIR)/nap/nap_conf.o \
        $(BOARDDIR)/nap/track_channel.o \
        $(BOARDDIR)/platform_signal.o \
        $(BOARDDIR)/remoteproc/remoteproc_config.o \
        $(BOARDDIR)/remoteproc/remoteproc_env.o \
        $(BOARDDIR)/remoteproc/rpmsg.o \
        $(BOARDDIR)/peripherals/antenna.o \
        $(BOARDDIR)/peripherals/can_termination.o \
        $(BOARDDIR)/peripherals/rf_clk.o \
        $(BOARDDIR)/peripherals/bmi160.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_gps_l1ca.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_gps_l2c.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_gps_l5.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_glo_l1of.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_glo_l2of.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_sbas_l1.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_bds2_b11.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_bds2_b2.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_bds3_b5.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_qzss_l1ca.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_qzss_l2c.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_qzss_l5.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_gal_e1.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_gal_e7.o \
        $(SWIFTNAV_ROOT)/src/track/signals/track_gal_e5.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_gps_l1ca.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_gps_l2c.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_glo_l1of.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_glo_l2of.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_sbas_l1.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_qzss_l1ca.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_bds_b1.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_bds_b2.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_bds3_b5.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_gal_e1.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_gal_e7.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_gal_e5.o \
        $(SWIFTNAV_ROOT)/src/decode/decode_common.o \

BOARDASM := \
        $(REVASM) \
        $(BOARDDIR)/cpu_init.s \
        $(BOARDDIR)/mmu_table.s \
        $(BOARDDIR)/error_asm.s \

BOARDLIBS := -lopen_amp

BOARDLIBDIR := $(OPENAMP_BUILDDIR)/lib

# Define linker script file here
LDSCRIPT= $(BOARDDIR)/ZYNQ7000.ld

RULESPATH = $(CHIBIOS)/os/common/ports/ARM/compilers/GCC

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-a9

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

#
# Compiler settings
##############################################################################
