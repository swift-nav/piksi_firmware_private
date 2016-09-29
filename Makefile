SWIFTNAV_ROOT := $(shell pwd)
MAKEFLAGS += SWIFTNAV_ROOT=$(SWIFTNAV_ROOT)

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

ifneq (,$(findstring W32,$(shell uname)))
	CMAKEFLAGS += -G "MSYS Makefiles"
endif

ifndef PRN
	MAKEFLAGS += $(warning PRN not defined, using default PRN (22) for tests, specify the PRN with 'make PRN=22')PRN=22
else
	MAKEFLAGS += PRN=$(PRN)
endif

ifeq ($(PIKSI_HW),)
  PIKSI_HW=v3
endif

ifeq ($(PIKSI_HW),v2)
	$(error PIKSI_HW=v2 is no longer supported.)
endif

ifeq ($(PIKSI_HW),v3)
	CMAKEFLAGS += -DCMAKE_SYSTEM_PROCESSOR=cortex-a9
	CMAKEFLAGS += -DMAX_CHANNELS=31

	ifeq ($(PIKSI_REV),)
		PIKSI_REV=microzed
	endif
endif

ifeq ($(PIKSI_REV),)
	PIKSI_TARGET=$(PIKSI_HW)
else
	PIKSI_TARGET=$(PIKSI_HW)_$(PIKSI_REV)
endif

MAKEFLAGS += PIKSI_HW=$(PIKSI_HW)
MAKEFLAGS += PIKSI_REV=$(PIKSI_REV)
MAKEFLAGS += PIKSI_TARGET=$(PIKSI_TARGET)

BUILDFOLDER = build_$(PIKSI_TARGET)
MAKEFLAGS += BUILDFOLDER=$(BUILDFOLDER)

# NOTE: assumes libsbp and libswiftnav are not PIKSI_TARGET sensitive
LIBSBP_BUILDDIR=$(SWIFTNAV_ROOT)/libsbp/c/build_$(PIKSI_HW)
LIBSWIFTNAV_BUILDDIR=$(SWIFTNAV_ROOT)/libswiftnav/build_$(PIKSI_HW)
MAKEFLAGS += LIBSBP_BUILDDIR=$(LIBSBP_BUILDDIR)
MAKEFLAGS += LIBSWIFTNAV_BUILDDIR=$(LIBSWIFTNAV_BUILDDIR)

.PHONY: all tests firmware docs hitl_setup hitl .FORCE

all: firmware # tests
	@printf "BUILDING For target $(PIKSI_TARGET)\n"

firmware: $(LIBSBP_BUILDDIR)/src/libsbp-static.a $(LIBSWIFTNAV_BUILDDIR)/src/libswiftnav-static.a
	@printf "BUILD   src for target $(PIKSI_TARGET)\n"; \
	$(MAKE) -r -C src $(MAKEFLAGS)

tests:
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "BUILD   $$i\n"; \
			$(MAKE) -r -C $$i $(MAKEFLAGS) || exit $?; \
		fi; \
	done

$(LIBSBP_BUILDDIR)/src/libsbp-static.a:
	@printf "BUILD   libsbp for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(LIBSBP_BUILDDIR); cd $(LIBSBP_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-gcc-arm-embedded.cmake $(CMAKEFLAGS) ../
	$(MAKE) -C $(LIBSBP_BUILDDIR) $(MAKEFLAGS)

$(LIBSWIFTNAV_BUILDDIR)/src/libswiftnav-static.a: .FORCE
	@printf "BUILD   libswiftnav for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(LIBSWIFTNAV_BUILDDIR); cd $(LIBSWIFTNAV_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-gcc-arm-embedded.cmake $(CMAKEFLAGS) ../
	$(MAKE) -C $(LIBSWIFTNAV_BUILDDIR) $(MAKEFLAGS)

clean:
	@printf "CLEAN   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS) clean
	@printf "CLEAN   libsbp\n"; \
	$(RM) -rf $(LIBSBP_BUILDDIR)
	@printf "CLEAN   libswiftnav\n"; \
	$(RM) -rf $(LIBSWIFTNAV_BUILDDIR)
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "CLEAN   $$i\n"; \
			$(MAKE) -C $$i $(MAKEFLAGS) clean || exit $?; \
		fi; \
	done

docs:
	$(MAKE) -C docs/diagrams
	doxygen docs/Doxyfile

hitl_setup: firmware
	# Usage:
	# `make hitl` will run the default "quick" test plan (1 capture job)
	# Optionally specify a desired test plan:
	# `make hitl TEST_PLAN=merge` will run the "merge" test plan (10 capture jobs)
	#
	# First, this script will pull or clone the hitl_tools repo.
	if cd $(BUILDFOLDER)/hitl_tools; then \
		git pull; \
	else \
		git clone git@github.com:swift-nav/hitl_tools.git $(BUILDFOLDER)/hitl_tools --depth 1; \
	fi

hitl: hitl_setup
	TEST_PLAN=$(TEST_PLAN) TEST_CONFIG=$(TEST_CONFIG) \
	BUILDFOLDER=$(BUILDFOLDER) bash $(BUILDFOLDER)/hitl_tools/make_hitl.sh

.FORCE:
