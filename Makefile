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

MAKEFLAGS += PIKSI_HW=$(PIKSI_HW)
MAKEFLAGS += PIKSI_REV=$(PIKSI_REV)
MAKEFLAGS += PIKSI_TARGET=$(PIKSI_TARGET)

BUILDFOLDER = build_$(PIKSI_TARGET)
MAKEFLAGS += BUILDFOLDER=$(BUILDFOLDER)

# NOTE: this assumes libraries are not PIKSI_REV sensitive
LIB_BUILDFOLDER = build_$(PIKSI_HW)
MAKEFLAGS += LIB_BUILDFOLDER=$(LIB_BUILDFOLDER)

LIBSBP_BUILDDIR=$(SWIFTNAV_ROOT)/libsbp/c/$(LIB_BUILDFOLDER)
LIBSWIFTNAV_BUILDDIR=$(SWIFTNAV_ROOT)/libswiftnav/$(LIB_BUILDFOLDER)
OPENAMP_BUILDDIR=$(SWIFTNAV_ROOT)/open-amp/$(LIB_BUILDFOLDER)

MAKEFLAGS += LIBSBP_BUILDDIR=$(LIBSBP_BUILDDIR)
MAKEFLAGS += LIBSWIFTNAV_BUILDDIR=$(LIBSWIFTNAV_BUILDDIR)
MAKEFLAGS += OPENAMP_BUILDDIR=$(OPENAMP_BUILDDIR)

FW_DEPS=$(LIBSBP_BUILDDIR)/src/libsbp-static.a \
        $(LIBSWIFTNAV_BUILDDIR)/src/libswiftnav-static.a

ifeq ($(PIKSI_HW),v3)
  CMAKEFLAGS += -DCMAKE_SYSTEM_PROCESSOR=cortex-a9
  CMAKEFLAGS += -DMAX_CHANNELS=31
  FW_DEPS += $(OPENAMP_BUILDDIR)/lib/libopen-amp.a
endif

.PHONY: all tests firmware docs .FORCE

all: firmware # tests
	@printf "BUILDING For target $(PIKSI_TARGET)\n"

firmware: $(FW_DEPS)
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
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	      -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-gcc-arm-embedded.cmake \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(LIBSBP_BUILDDIR) $(MAKEFLAGS)

$(LIBSWIFTNAV_BUILDDIR)/src/libswiftnav-static.a: .FORCE
	@printf "BUILD   libswiftnav for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(LIBSWIFTNAV_BUILDDIR); cd $(LIBSWIFTNAV_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	      -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-gcc-arm-embedded.cmake \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(LIBSWIFTNAV_BUILDDIR) $(MAKEFLAGS)

$(OPENAMP_BUILDDIR)/lib/libopen-amp.a:
	@printf "BUILD   open-amp for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(OPENAMP_BUILDDIR) ; cd $(OPENAMP_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	      -DCMAKE_TOOLCHAIN_FILE=../cmake/platforms/Toolchain-gcc-arm-embedded.cmake \
	      -DMACHINE=custom \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(OPENAMP_BUILDDIR) $(MAKEFLAGS)

clean:
	@printf "CLEAN   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS) clean
	@printf "CLEAN   libsbp\n"; \
	$(RM) -rf $(LIBSBP_BUILDDIR)
	@printf "CLEAN   libswiftnav\n"; \
	$(RM) -rf $(LIBSWIFTNAV_BUILDDIR)
	@printf "CLEAN   open-amp\n"; \
	$(RM) -rf $(OPENAMP_BUILDDIR)
	$(Q)for i in tests/*; do \
		if [ -d $$i ]; then \
			printf "CLEAN   $$i\n"; \
			$(MAKE) -C $$i $(MAKEFLAGS) clean || exit $?; \
		fi; \
	done

docs:
	$(MAKE) -C docs/diagrams
	doxygen docs/Doxyfile

.FORCE:
