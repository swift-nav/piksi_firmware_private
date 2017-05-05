##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

ifeq ($(PIKSI_HW),)
  PIKSI_HW=v3
endif

ifeq ($(PIKSI_HW),v3)
  ifeq ($(PIKSI_REV),)
    PIKSI_REV=prod
  endif
endif

ifeq ($(PIKSI_REV),)
  PIKSI_TARGET=$(PIKSI_HW)
else
  PIKSI_TARGET=$(PIKSI_HW)_$(PIKSI_REV)
endif

ifeq ($(SWIFTNAV_ROOT),)
  SWIFTNAV_ROOT = ..
endif

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb3 -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT := --wrap=_vfprintf,--wrap=_vfprintf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=vfiprintf,--wrap=_vfiprintf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=_svfprintf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=_svfiprintf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=vfscanf,--wrap=_vfscanf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=__svfscanf,--wrap=__svfscanf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=vfiscanf,--wrap=_vfiscanf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=__svfiscanf,--wrap=__svfiscanf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=__ssvfscanf_r
  USE_LDOPT := $(USE_LDOPT),--wrap=__ssvfiscanf_r
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

ifeq ($(BUILDFOLDER),)
  BUILDFOLDER = build_$(PIKSI_TARGET)
endif

ifeq ($(BUILDDIR),)
  BUILDDIR = $(SWIFTNAV_ROOT)/$(BUILDFOLDER)
endif

# NOTE: this assumes libraries are not PIKSI_REV sensitive
ifeq ($(LIB_BUILDFOLDER),)
  LIB_BUILDFOLDER = build_$(PIKSI_HW)
endif

ifeq ($(LIBSBP_BUILDDIR),)
  LIBSBP_BUILDDIR=$(SWIFTNAV_ROOT)/libsbp/c/$(LIB_BUILDFOLDER)
endif

ifeq ($(LIBSWIFTNAV_BUILDDIR),)
  LIBSWIFTNAV_BUILDDIR=$(SWIFTNAV_ROOT)/libswiftnav/$(LIB_BUILDFOLDER)
endif

#
# Build global options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = piksi_firmware_$(PIKSI_TARGET)

# Imported source files and paths
CHIBIOS = ../ChibiOS

include $(SWIFTNAV_ROOT)/src/board/$(PIKSI_HW)/Makefile.inc

include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
include $(CHIBIOS)/os/rt/rt.mk

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC := $(STARTUPSRC) \
        $(PORTSRC) \
        $(KERNSRC) \
        $(HALSRC) \
        $(OSALSRC) \
        $(PLATFORMSRC) \
        $(BOARDSRC) \
        $(SWIFTNAV_ROOT)/src/lib/fifo.o \
        $(SWIFTNAV_ROOT)/src/dum.o \
        $(SWIFTNAV_ROOT)/src/sbp.o \
        $(SWIFTNAV_ROOT)/src/sbp_fileio.o \
        $(SWIFTNAV_ROOT)/src/sbp_utils.o \
        $(SWIFTNAV_ROOT)/src/track.o \
        $(SWIFTNAV_ROOT)/src/track_internal.o \
        $(SWIFTNAV_ROOT)/src/track_api.o \
        $(SWIFTNAV_ROOT)/src/track/track_cn0.o \
        $(SWIFTNAV_ROOT)/src/track/track_profiles.o \
        $(SWIFTNAV_ROOT)/src/track/track_profile_utils.o \
        $(SWIFTNAV_ROOT)/src/track/track_profile_loop.o \
        $(SWIFTNAV_ROOT)/src/track/track_profile_corrs.o \
        $(SWIFTNAV_ROOT)/src/track/track_common.o \
        $(SWIFTNAV_ROOT)/src/track/track_sid_db.o \
        $(SWIFTNAV_ROOT)/src/track/track_sbp.o \
        $(SWIFTNAV_ROOT)/src/manage.o \
        $(SWIFTNAV_ROOT)/src/settings.o \
        $(SWIFTNAV_ROOT)/src/timing.o \
        $(SWIFTNAV_ROOT)/src/ext_events.o \
        $(SWIFTNAV_ROOT)/src/position.o \
        $(SWIFTNAV_ROOT)/src/solution.o \
        $(SWIFTNAV_ROOT)/src/base_obs.o \
        $(SWIFTNAV_ROOT)/src/simulator.o \
        $(SWIFTNAV_ROOT)/src/simulator_data.o \
        $(SWIFTNAV_ROOT)/src/syscalls.o \
        $(SWIFTNAV_ROOT)/src/nmea.o \
        $(SWIFTNAV_ROOT)/src/system_monitor.o \
        $(SWIFTNAV_ROOT)/src/ephemeris.o \
        $(SWIFTNAV_ROOT)/src/pps.o \
        $(SWIFTNAV_ROOT)/src/decode.o \
        $(SWIFTNAV_ROOT)/src/signal.o \
        $(SWIFTNAV_ROOT)/src/ndb/ndb_internal.o \
        $(SWIFTNAV_ROOT)/src/ndb/ndb_ephemeris.o \
        $(SWIFTNAV_ROOT)/src/ndb/ndb_almanac.o \
        $(SWIFTNAV_ROOT)/src/ndb/ndb_iono.o \
        $(SWIFTNAV_ROOT)/src/ndb/ndb_lgf.o \
        $(SWIFTNAV_ROOT)/src/ndb/ndb_utc.o \
        $(SWIFTNAV_ROOT)/src/ndb/ndb.o \
        $(SWIFTNAV_ROOT)/src/shm.o \
        $(SWIFTNAV_ROOT)/src/reacq/reacq_manage.o \
        $(SWIFTNAV_ROOT)/src/reacq/reacq_sbp_utility.o \
        $(SWIFTNAV_ROOT)/src/reacq/search_manager.o \
        $(SWIFTNAV_ROOT)/src/reacq/search_manager_utils.o \
        $(SWIFTNAV_ROOT)/src/reacq/scheduler.o \
        $(SWIFTNAV_ROOT)/src/reacq/scheduler_utils.o \
        $(SWIFTNAV_ROOT)/src/reacq/task_generator.o \
        $(BOARDDIR)/ndb_fs_access.o \
        $(BOARDDIR)/cnav_msg_storage.o \
        main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM) $(BOARDASM)

INCDIR = $(STARTUPINC) $(PORTINC) $(KERNINC) \
         $(HALINC) $(OSALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various \
         $(SWIFTNAV_ROOT)/libsbp/c/include \
         $(SWIFTNAV_ROOT)/libswiftnav/include \
         $(SWIFTNAV_ROOT)/src \
         $(SWIFTNAV_ROOT)/src/board \
         $(SWIFTNAV_ROOT)/libswiftnav/libfec/include \

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# Define C warning options here
CWARN = -Wall -Wextra -Werror -std=gnu99 -Wimplicit \
	-Wshadow -Wswitch-default -Wswitch-enum -Wundef \
	-Wuninitialized -Wstrict-prototypes \
	-Wformat=2 \
	-Wimplicit-function-declaration -Wredundant-decls \
	-Wformat-security -Wpointer-arith
	# TODO Leith: enable these once I figure out how to skip them on ChibiOS stuff
	# -Wmissing-prototypes -Wcast-align

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Werror -Weffc++ -std=gnu++11 \
  -Waggregate-return -Wcast-align \
  -Wcast-qual  -Wchar-subscripts  -Wcomment -Wconversion \
  -Wdisabled-optimization -Wfloat-equal  -Wformat  -Wformat=2 \
  -Wformat-nonliteral -Wformat-security -Wformat-y2k \
  -Wimport  -Winit-self  -Winline -Winvalid-pch   \
  -Wunsafe-loop-optimizations -Wlong-long -Wmissing-braces \
  -Wmissing-field-initializers -Wmissing-format-attribute   \
  -Wmissing-include-dirs -Wmissing-noreturn \
  -Wpacked  -Wpadded -Wparentheses  -Wpointer-arith \
  -Wredundant-decls -Wreturn-type \
  -Wsequence-point  -Wshadow -Wsign-compare  -Wstack-protector \
  -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch  -Wswitch-default \
  -Wswitch-enum -Wtrigraphs  -Wuninitialized \
  -Wunknown-pragmas  -Wunreachable-code -Wunused \
  -Wunused-function  -Wunused-label  -Wunused-parameter \
  -Wunused-value  -Wunused-variable  -Wvariadic-macros \
  -Wvolatile-register-var  -Wwrite-strings

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

GIT_VERSION := $(shell git describe --tags --dirty --always --long)
version.h: FORCE
	@echo "Generating $@"
	@echo "#define GIT_VERSION \"$(GIT_VERSION)\"" > $@

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here

DLIBDIR = $(LIBSBP_BUILDDIR)/src \
          $(LIBSWIFTNAV_BUILDDIR)/src \
          $(LIBSWIFTNAV_BUILDDIR)/libfec/src \
          $(BOARDLIBDIR)

# List all default libraries here
DLIBS = $(BOARDLIBS) -lsbp-static -lswiftnav-static \
        -lm -Wl,--start-group -lstdc++ -lc -lnosys -Wl,--end-group -lfec

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

%.stripped.elf: %.elf
	@echo "Stripping $@"
	@$(TRGT)strip -o $@ $<

$(SWIFTNAV_ROOT)/src/ndb/ndb_internal.c $(SWIFTNAV_ROOT)/src/main.c: version.h
.DEFAULT_GOAL := $(BUILDDIR)/$(PROJECT).stripped.elf
include $(SWIFTNAV_ROOT)/ext/Makefile.include

include $(RULESPATH)/rules.mk
