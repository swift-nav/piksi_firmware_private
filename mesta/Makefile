mesta:

ifeq ($(SWIFTNAV_ROOT),)
  SWIFTNAV_ROOT = ..
endif

OBJDIR := $(SWIFTNAV_ROOT)/mesta/build

clean:
	@rm -fr $(OBJDIR)
	@rm mesta

UDEF = '-Wno-unused' \
       '-DMAX_CHANNELS=73' \
       '-DSTARLING_MAX_CHANNEL_COUNT=150' \
       '-DMESTA_BUILD' \
       '-DPIKSI_RELEASE'

INCDIR = $(SWIFTNAV_ROOT) \
         $(SWIFTNAV_ROOT)/mesta/stubs/board/v3/nap \
         $(SWIFTNAV_ROOT)/mesta/stubs/board/v3 \
	 $(SWIFTNAV_ROOT)/mesta/stubs/board \
	 $(SWIFTNAV_ROOT)/mesta/stubs \
	 $(SWIFTNAV_ROOT)/src \
	 $(SWIFTNAV_ROOT)/src/cfg \
	 $(SWIFTNAV_ROOT)/src/utils \
	 $(SWIFTNAV_ROOT)/src/acq \
	 $(SWIFTNAV_ROOT)/src/decode \
	 $(SWIFTNAV_ROOT)/src/sbp \
	 $(SWIFTNAV_ROOT)/src/calc \
	 $(SWIFTNAV_ROOT)/mesta/stubs \
	 $(SWIFTNAV_ROOT)/mesta/stubs/libswiftnav \
	 $(SWIFTNAV_ROOT)/starling/include \
	 $(SWIFTNAV_ROOT)/starling/include/starling \
	 $(SWIFTNAV_ROOT)/starling/libfec/include \
	 $(SWIFTNAV_ROOT)/src/hal \
	 $(SWIFTNAV_ROOT)/libsbp/c/include \
	 $(SWIFTNAV_ROOT)/src/board/v3/prod \
	 $(SWIFTNAV_ROOT)/src/board/v3 \
	 $(SWIFTNAV_ROOT)/src/board \
	 $(SWIFTNAV_ROOT)/src/main \
	 $(SWIFTNAV_ROOT)/starling/src \
	 $(SWIFTNAV_ROOT)/starling/ \
	 $(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/include \
	 $(SWIFTNAV_ROOT)/libsettings/include \
	 $(SWIFTNAV_ROOT)/libsbp \
	 $(SWIFTNAV_ROOT)/libsbp/c/include/libsbp

IINCDIR  = $(patsubst %,-I%,$(INCDIR))

CSRC := $(SWIFTNAV_ROOT)/src/track/signals/track_gps_l2c.c \
        $(SWIFTNAV_ROOT)/src/track/signals/track_gal_e7.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_sbas_l1.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_qzss_l1ca.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_bds2_b11.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_bds2_b2.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_glo_l1of.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_glo_l2of.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_qzss_l2c.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_gal_e1.c \
	$(SWIFTNAV_ROOT)/src/track/signals/track_gps_l1ca.c \
	$(SWIFTNAV_ROOT)/src/track/track_cn0.c \
	$(SWIFTNAV_ROOT)/src/track/track_common.c \
	$(SWIFTNAV_ROOT)/src/track/track_profile_utils.c \
	$(SWIFTNAV_ROOT)/src/track/track_decode.c \
	$(SWIFTNAV_ROOT)/src/track/track_profile_loop.c \
	$(SWIFTNAV_ROOT)/src/track/track_profiles.c \
	$(SWIFTNAV_ROOT)/src/track/track_sid_db.c \
	$(SWIFTNAV_ROOT)/src/track/track_state.c \
	$(SWIFTNAV_ROOT)/src/track/track_utils.c \
	$(SWIFTNAV_ROOT)/src/track/track_api.c \
	$(SWIFTNAV_ROOT)/src/track/track_timer.c \
	$(SWIFTNAV_ROOT)/src/track/track_profile_corrs.c \
	$(SWIFTNAV_ROOT)/src/track/track_flags.c \
	$(SWIFTNAV_ROOT)/src/track/track_interface.c \
	$(SWIFTNAV_ROOT)/src/cn0_est/cn0_est_bl.c \
	$(SWIFTNAV_ROOT)/src/cn0_est/cn0_est_basic.c \
	$(SWIFTNAV_ROOT)/src/cn0_est/noise.c \
	$(SWIFTNAV_ROOT)/src/utils/filters/filter_lp1.c \
	$(SWIFTNAV_ROOT)/src/utils/lock_detector/lock_detector.c \
	$(SWIFTNAV_ROOT)/src/utils/track_loop/track_pll2_dll1.c \
	$(SWIFTNAV_ROOT)/src/utils/track_loop/track_pll3_fll2_dll1.c \
	$(SWIFTNAV_ROOT)/src/utils/track_loop/trk_loop_common.c \
	$(SWIFTNAV_ROOT)/src/utils/signal_db/signal_db.c \
	$(SWIFTNAV_ROOT)/src/utils/bit_sync/bit_sync.c \
	$(SWIFTNAV_ROOT)/src/utils/timing/timing.c \
	$(SWIFTNAV_ROOT)/src/utils/nav_bit_fifo/nav_bit_fifo.c \
	$(SWIFTNAV_ROOT)/src/utils/nav_data_sync/nav_data_sync.c \
	$(SWIFTNAV_ROOT)/src/utils/gnss_capabilities/gnss_capabilities.c \
	$(SWIFTNAV_ROOT)/src/utils/shm/shm.c \
	$(SWIFTNAV_ROOT)/src/utils/ephemeris/ephemeris.c \
	$(SWIFTNAV_ROOT)/src/utils/obs_bias/obs_bias.c \
	$(SWIFTNAV_ROOT)/src/utils/nmea/nmea.c \
	$(SWIFTNAV_ROOT)/src/utils/sbas_select/sbas_select.c \
	$(SWIFTNAV_ROOT)/src/utils/sv_visibility/sv_visibility.c \
	$(SWIFTNAV_ROOT)/src/utils/dum/dum.c \
	$(SWIFTNAV_ROOT)/src/utils/ext_events/ext_events.c \
	$(SWIFTNAV_ROOT)/src/utils/clock_filter/clock_filter.c \
	$(SWIFTNAV_ROOT)/src/hal/piksi_systime.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb_gnss_capabilities.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb_internal.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb_iono.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb_lgf.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb_utc.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb_almanac.c \
	$(SWIFTNAV_ROOT)/src/ndb/ndb_ephemeris.c \
	$(SWIFTNAV_ROOT)/src/sbp/sbp.c \
	$(SWIFTNAV_ROOT)/src/sbp/sbp_utils.c \
	$(SWIFTNAV_ROOT)/src/acq/manage.c \
	$(SWIFTNAV_ROOT)/src/reacq/reacq_manage.c \
	$(SWIFTNAV_ROOT)/src/reacq/reacq_sbp_utility.c \
	$(SWIFTNAV_ROOT)/src/reacq/scheduler.c \
	$(SWIFTNAV_ROOT)/src/reacq/scheduler_utils.c \
	$(SWIFTNAV_ROOT)/src/reacq/search_manager.c \
	$(SWIFTNAV_ROOT)/src/reacq/search_manager_utils.c \
	$(SWIFTNAV_ROOT)/src/reacq/task_generator.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/bds2_prns.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/gal_prns.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/prns.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/qzss_prns.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/soft_macq_main.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/soft_macq_mdbzp.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/soft_macq_serial.c \
	$(SWIFTNAV_ROOT)/src/soft_macq/soft_macq_utils.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_gps_l1ca.c \
	$(SWIFTNAV_ROOT)/src/decode/decode.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_gal_e7.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_common.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_glo_l2of.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_bds_b2.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_glo_l1of.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_gps_l2c.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_qzss_l1ca.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_sbas_l1.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_bds_b1.c \
	$(SWIFTNAV_ROOT)/src/decode/decode_gal_e1.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/nav_msg_bds.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/nav_msg_gal.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/cnav_msg.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/sbas_msg.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/nav_msg_glo.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/nav_msg.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/cnav_msg_storage.c \
	$(SWIFTNAV_ROOT)/src/nav_msg/cons_time_storage.c \
	$(SWIFTNAV_ROOT)/src/calc/calc_pvt_me.c \
	$(SWIFTNAV_ROOT)/src/calc/calc_nav_meas.c \
	$(SWIFTNAV_ROOT)/src/calc/calc_pvt_common.c \
	$(SWIFTNAV_ROOT)/src/board/v3/nap/nap_common.c \
	$(SWIFTNAV_ROOT)/src/board/v3/nap/track_channel.c \
	$(SWIFTNAV_ROOT)/src/board/v3/platform_signal.c \
	$(SWIFTNAV_ROOT)/src/board/v3/ndb_fs_access.c \
	$(SWIFTNAV_ROOT)/src/lib/fixed_fft_r2.c \
	$(SWIFTNAV_ROOT)/starling/src/starling/starling_platform.c \
	$(SWIFTNAV_ROOT)/starling/src/starling/starling_platform_mq.c \
	$(SWIFTNAV_ROOT)/starling/src/starling/starling_platform_semaphore.c \
	$(SWIFTNAV_ROOT)/starling/src/observation.c \
	$(SWIFTNAV_ROOT)/starling/src/cycle_slip.c \
	$(SWIFTNAV_ROOT)/starling/libfec/src/viterbi27.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/almanac.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/bits.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/coord_system.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/correct_iono_tropo.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/edc.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/ephemeris.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/fifo_byte.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/glo_map.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/glonass_phase_biases.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/gnss_time.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/ionosphere.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/linear_algebra.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/logging.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/logging_common.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/memcpy_s.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/nav_meas.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/set.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/shm.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/sid_set.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/signal.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/single_epoch_solver.c \
	$(SWIFTNAV_ROOT)/starling/third_party/libswiftnav/src/troposphere.c \
	$(SWIFTNAV_ROOT)/libsettings/src/settings.c \
	$(SWIFTNAV_ROOT)/libsettings/src/settings_util.c \
	$(SWIFTNAV_ROOT)/libsettings/src/request_state.c \
	$(SWIFTNAV_ROOT)/libsettings/src/setting_data.c \
	$(SWIFTNAV_ROOT)/libsettings/src/setting_type.c \
	$(SWIFTNAV_ROOT)/libsettings/src/setting_type_enum.c \
	$(SWIFTNAV_ROOT)/libsettings/src/setting_type_float.c \
	$(SWIFTNAV_ROOT)/libsettings/src/setting_type_int.c \
	$(SWIFTNAV_ROOT)/libsettings/src/setting_type_str.c \
	$(SWIFTNAV_ROOT)/mesta/stubs/stubs.c \
	$(SWIFTNAV_ROOT)/mesta/stubs/starling_platform_stubs.c \
	$(SWIFTNAV_ROOT)/mesta/main.c \


CXXSRC := $(SWIFTNAV_ROOT)/starling/src/starling/integration/starling_input_bridge.cc \
          $(SWIFTNAV_ROOT)/starling/src/starling/util/Mutex.cc \
          $(SWIFTNAV_ROOT)/starling/src/starling/util/ObsFifo.cc \

CXXFLAGS := -std=c++11

target = $(patsubst $(SWIFTNAV_ROOT)%$(suffix ${1}),$(OBJDIR)%.o,${1})

obj.c :=
obj.cc :=
define obj
  $(call target,${1}) : ${1} | ${OBJDIR}
  obj$(suffix ${1}) += $(call target,${1})
endef

define SOURCES
  $(foreach src,${1},$(eval $(call obj,${src})))
endef

$(eval $(call SOURCES,${CSRC}))
$(eval $(call SOURCES,${CXXSRC}))

mesta : ${obj.c} ${obj.cc}
	$(CXX) -o $@ $^ -lm

${obj.c} : % :
	@mkdir -p $(dir $@)
	$(CC) -c -O0 -g $(CFLAGS) $(IINCDIR) $(UDEF) -o $@ $^

${obj.cc} : % :
	@mkdir -p $(dir $@)
	$(CXX) -c $(CXXFLAGS) $(IINCDIR) $(UDEF) -o $@ $^

${OBJDIR} :
	mkdir $@

.PHONY: clean mesta
