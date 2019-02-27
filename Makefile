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
LIBSETTINGS_BUILDDIR=$(SWIFTNAV_ROOT)/libsettings/$(LIB_BUILDFOLDER)
STARLING_BUILDDIR=$(SWIFTNAV_ROOT)/starling/$(LIB_BUILDFOLDER)
LIBSWIFTNAV_BUILDDIR=$(STARLING_BUILDDIR)/third_party/libswiftnav
OPENAMP_BUILDDIR=$(SWIFTNAV_ROOT)/open-amp/$(LIB_BUILDFOLDER)

MAKEFLAGS += LIBSBP_BUILDDIR=$(LIBSBP_BUILDDIR)
MAKEFLAGS += LIBSETTINGS_BUILDDIR=$(LIBSETTINGS_BUILDDIR)
MAKEFLAGS += STARLING_BUILDDIR=$(STARLING_BUILDDIR)
MAKEFLAGS += LIBSWIFTNAV_BUILDDIR=$(LIBSWIFTNAV_BUILDDIR)
MAKEFLAGS += OPENAMP_BUILDDIR=$(OPENAMP_BUILDDIR)

FW_DEPS=compiler-version \
        $(LIBSBP_BUILDDIR)/src/libsbp.a \
        $(LIBSETTINGS_BUILDDIR)/src/libsettings.a \
        $(STARLING_BUILDDIR)/src/libpvt-engine.a \
        $(STARLING_BUILDDIR)/src/libstarling.a \
        $(STARLING_BUILDDIR)/src/libstarling-util.a

ifeq ($(PIKSI_HW),v3)
  FW_DEPS += $(OPENAMP_BUILDDIR)/lib/libopen-amp.a
endif

CLANG_TIDY_INCLUDES = -I$(SWIFTNAV_ROOT)/include/ \
                      -I$(SWIFTNAV_ROOT)/src/ \
                      -I$(SWIFTNAV_ROOT)/src/utils/ \
                      -I$(SWIFTNAV_ROOT)/starling/include/ \
                      -I$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/include/ \
                      -I$(SWIFTNAV_ROOT)/libsbp/c/include/ \
                      -I$(SWIFTNAV_ROOT)/starling/libfec/include/ \
                      -I$(SWIFTNAV_ROOT)/libsettings/include/ \
                      -I$(SWIFTNAV_ROOT)/src/board/ \
                      -I$(SWIFTNAV_ROOT)/src/board/v3/ \
                      -I$(SWIFTNAV_ROOT)/src/board/v3/prod/ \
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

$(LIBSBP_BUILDDIR)/src/libsbp.a:
	@printf "BUILD   libsbp for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(LIBSBP_BUILDDIR); cd $(LIBSBP_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	      -DCMAKE_TOOLCHAIN_FILE=../../piksi-toolchain.cmake \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(LIBSBP_BUILDDIR) $(MAKEFLAGS)

$(LIBSETTINGS_BUILDDIR)/src/libsettings.a: $(LIBSBP_BUILDDIR)/src/libsbp.a
	@printf "BUILD   libsettings for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(LIBSETTINGS_BUILDDIR); cd $(LIBSETTINGS_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	      -DCMAKE_TOOLCHAIN_FILE=../../piksi-toolchain.cmake \
	      -DLIBSBP_SEARCH_PATH=$(SWIFTNAV_ROOT)/libsbp/c/ \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(LIBSETTINGS_BUILDDIR) $(MAKEFLAGS) settings

$(STARLING_BUILDDIR)/src/libpvt-engine.a: .FORCE \
                                        $(STARLING_BUILDDIR)/Makefile
	@printf "BUILD   pvt-engine for target $(PIKSI_TARGET)\n"; \
	$(MAKE) pvt-engine -C $(STARLING_BUILDDIR) $(MAKEFLAGS)

# Make starling dependent of swiftnav because otherwise both
# might build in parallel, and both trying to build swiftnav-common in parallel
# which leads to occasional failures.
$(STARLING_BUILDDIR)/src/libstarling.a: .FORCE \
                                             $(STARLING_BUILDDIR)/Makefile \
                                             $(STARLING_BUILDDIR)/src/libpvt-engine.a
	@printf "BUILD   libstarling for target $(PIKSI_TARGET)\n"; \
	$(MAKE) starling -C $(STARLING_BUILDDIR) $(MAKEFLAGS)

$(STARLING_BUILDDIR)/src/libstarling-util.a: .FORCE \
                                             $(STARLING_BUILDDIR)/Makefile
	@printf "BUILD   libstarling-util for target $(PIKSI_TARGET)\n"; \
	$(MAKE) starling-util -C $(STARLING_BUILDDIR) $(MAKEFLAGS)

$(STARLING_BUILDDIR)/Makefile:
	@printf "Run cmake for target $(STARLING_BUILDDIR)\n"; \
    mkdir -p $(STARLING_BUILDDIR); cd $(STARLING_BUILDDIR); \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
          -DCMAKE_TOOLCHAIN_FILE=../../piksi-toolchain.cmake \
          -DMAX_CHANNELS=81 \
          $(CMAKEFLAGS) ../

$(OPENAMP_BUILDDIR)/lib/libopen-amp.a:
	@printf "BUILD   open-amp for target $(PIKSI_TARGET)\n"; \
	mkdir -p $(OPENAMP_BUILDDIR) ; cd $(OPENAMP_BUILDDIR); \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	      -DCMAKE_TOOLCHAIN_FILE=../../piksi-toolchain.cmake \
	      -DMACHINE=custom \
	      $(CMAKEFLAGS) ../
	$(MAKE) -C $(OPENAMP_BUILDDIR) $(MAKEFLAGS)

clean:
	@printf "CLEAN   src\n"; \
	$(MAKE) -C src $(MAKEFLAGS) clean
	@printf "CLEAN   libsbp\n"; \
	$(RM) -rf $(LIBSBP_BUILDDIR)
	@printf "CLEAN   libsettings\n"; \
	$(RM) -rf $(LIBSETTINGS_BUILDDIR)
	@printf "CLEAN   starling\n"; \
	$(RM) -rf $(STARLING_BUILDDIR)
	@printf "CLEAN   open-amp\n"; \
	$(RM) -rf $(OPENAMP_BUILDDIR)
	@printf "CLEAN   tests\n"; \
	$(MAKE) -C tests clean

docs:
	$(MAKE) -C docs/diagrams
	doxygen docs/Doxyfile

clang-format-all:
	@echo "Auto formatting all C files under src/"
	clang-format-6.0 -i $$(git ls-files 'src/*.[ch]')

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
		| xargs -P 1 -I file clang-tidy-6.0 -export-fixes=fixes.yaml file -- $(CLANG_TIDY_FLAGS) $(CLANG_TIDY_INCLUDES)

run_tests:
	$(MAKE) -C tests
	./tests/run_tests

.FORCE:
