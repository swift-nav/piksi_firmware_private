SWIFTNAV_ROOT := $(shell pwd)

MAKEFLAGS += SWIFTNAV_ROOT=$(SWIFTNAV_ROOT)
PFWP_COMPILER := arm-none-eabi-

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
  Q := @
  # Do not print "Entering directory ...".
  MAKEFLAGS += --no-print-directory
endif

ifneq (,$(findstring W32,$(shell uname)))
  CMAKEFLAGS += -G "MSYS Makefiles"
endif
CMAKEFLAGS += -DTHIRD_PARTY_INCLUDES_AS_SYSTEM=false

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

LIBSETTINGS_BUILDDIR=$(SWIFTNAV_ROOT)/libsettings/$(LIB_BUILDFOLDER)
OPENAMP_BUILDDIR=$(SWIFTNAV_ROOT)/open-amp/$(LIB_BUILDFOLDER)

MAKEFLAGS += LIBSETTINGS_BUILDDIR=$(LIBSETTINGS_BUILDDIR)
MAKEFLAGS += OPENAMP_BUILDDIR=$(OPENAMP_BUILDDIR)

FW_DEPS=compiler-version \
        $(LIBSETTINGS_BUILDDIR)/src/libsettings.a

ifeq ($(PIKSI_HW),v3)
  FW_DEPS += $(OPENAMP_BUILDDIR)/lib/libopen-amp.a
endif

CLANG_TIDY_INCLUDES = -I$(SWIFTNAV_ROOT)/include/ \
                      -I$(SWIFTNAV_ROOT)/src/ \
                      -I$(SWIFTNAV_ROOT)/src/utils/ \
                      -I$(SWIFTNAV_ROOT)/static_library/starling/pvt_common/include/ \
                      -I$(SWIFTNAV_ROOT)/static_library/starling/pvt_driver/include/ \
                      -I$(SWIFTNAV_ROOT)/static_library/starling/pvt_engine/include/ \
                      -I$(SWIFTNAV_ROOT)/static_library/starling/include \
                      -I$(SWIFTNAV_ROOT)/static_library/libpal/pal/include/ \
                      -I$(SWIFTNAV_ROOT)/static_library/libpal/pal++/include/ \
                      -I$(SWIFTNAV_ROOT)/static_library/libswiftnav/include/ \
                      -I$(SWIFTNAV_ROOT)/static_library/libsbp/include/ \
                      -I$(SWIFTNAV_ROOT)/static_library/libfec/include/ \
                      -I$(SWIFTNAV_ROOT)/libsettings/include/ \
                      -I$(SWIFTNAV_ROOT)/src/board/ \
                      -I$(SWIFTNAV_ROOT)/src/board/v3/ \
                      -I$(SWIFTNAV_ROOT)/src/board/v3/prod/ \
                      -I$(SWIFTNAV_ROOT)/src/board/v3/base/ \
                      -I$(SWIFTNAV_ROOT)/src/legacy_starling_util/include/ \
                      -isystem$(SWIFTNAV_ROOT)/mesta/stubs/

.PHONY: all tests firmware docs .FORCE
.SUFFIXES:

all: firmware
	@printf "BUILD finished for target $(PIKSI_TARGET)\n"

compiler-version:
	@printf "$(shell ${PFWP_COMPILER}gcc --version)\n"

firmware: $(FW_DEPS)
	@printf "BUILD   src for target $(PIKSI_TARGET)\n"; \
	$(MAKE) -r -C src $(MAKEFLAGS)

$(LIBSETTINGS_BUILDDIR)/src/libsettings.a:
	@printf "BUILD   libsettings for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(LIBSETTINGS_BUILDDIR); cd $(LIBSETTINGS_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=Release \
	      -DCMAKE_TOOLCHAIN_FILE=../../piksi-toolchain.cmake \
	      -DLIBSBP_SEARCH_PATH=$(SWIFTNAV_ROOT)/static_library/libsbp \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(LIBSETTINGS_BUILDDIR) $(MAKEFLAGS) settings

# Make starling dependent of swiftnav because otherwise both
# might build in parallel, and both trying to build swiftnav-common in parallel
# which leads to occasional failures.

$(OPENAMP_BUILDDIR)/lib/libopen-amp.a:
	@printf "BUILD   open-amp for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(OPENAMP_BUILDDIR) ; cd $(OPENAMP_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=Release \
	      -DCMAKE_TOOLCHAIN_FILE=../../piksi-toolchain.cmake \
	      -DMACHINE=custom \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(OPENAMP_BUILDDIR) $(MAKEFLAGS)

clean:
	@printf "CLEAN   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS) clean
	@printf "CLEAN   libsettings\n"; \
	$(RM) -rf $(LIBSETTINGS_BUILDDIR)
	@printf "CLEAN   open-amp\n"; \
	$(RM) -rf $(OPENAMP_BUILDDIR)
	@printf "CLEAN   tests\n"; \
	$(MAKE) -C tests clean

docs:
	$(MAKE) -C docs/diagrams
	doxygen docs/Doxyfile

clang-format-all:
	@echo "Auto formatting all C files under src/"
	clang-format-6.0 -i $$(git ls-files 'src/*.[ch]' 'mesta/*.[ch]')

clang-format-head:
	@echo "Auto formatting all staged lines"
	git-clang-format-6.0

clang-format-diff:
	@echo "Autoformatting all lines which differ from master"
	git-clang-format-6.0 master

clang-tidy-all:
	@echo "Checking all C files under src/"
	git ls-files -- 'src/*.[ch]' \
		| grep -E -v 'board|chibios|peripherals|system_monitor|syscalls|chconf' \
		| xargs -P 1 -I file clang-tidy-6.0 -quiet -export-fixes=fixes.yaml file -- $(CLANG_TIDY_FLAGS) $(CLANG_TIDY_INCLUDES)

run_tests:
	$(MAKE) -C tests
	./tests/run_tests

.FORCE:
