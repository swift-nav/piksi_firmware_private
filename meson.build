project('mesta', 'c', meson_version: '>= 0.36.0')

compiler = meson.get_compiler('c')
compiler_id = compiler.get_id()
libm = compiler.find_library('m', required: true)

mesta_args = ['-Wno-unused']
mesta_args += ['-DMAX_CHANNELS=73']
mesta_args += ['-DSTARLING_MAX_CHANNEL_COUNT=150']
mesta_args += ['-DMESTA_BUILD']
mesta_args += ['-DPIKSI_RELEASE']

mesta_inc = include_directories(['apps/mesta/stubs/board/v3/nap',
                                 'apps/mesta/stubs/board/v3',
                                 'apps/mesta/stubs/board',
                                 'apps/mesta/stubs',
                                 'src',
                                 'src/cfg',
                                 'src/utils',
                                 'src/acq',
                                 'src/decode',
                                 'src/sbp',
                                 'src/calc',
                                 'apps/mesta/stubs',
                                 'apps/mesta/stubs/libswiftnav',
                                 'starling/include',
                                 'starling/include/starling',
                                 'starling/libfec/include',
                                 'src/hal',
                                 'libsbp/c/include',
                                 'src/board/v3/prod',
                                 'src/board/v3',
                                 'src/board',
                                 'src/main',
                                 'starling/src',
                                 'starling/',
                                 'starling/third_party/libswiftnav/include',
                                 'libsettings/include',
                                 'libsbp',
                                 'libsbp/c/include/libsbp'])

mesta_src = files()

mesta_src += files('src/track/signals/track_gps_l2c.c',
                   'src/track/signals/track_gal_e7.c',
                   'src/track/signals/track_sbas_l1.c',
                   'src/track/signals/track_qzss_l1ca.c',
                   'src/track/signals/track_bds2_b11.c',
                   'src/track/signals/track_bds2_b2.c',
                   'src/track/signals/track_gps_l2c.c',
                   'src/track/signals/track_glo_l1of.c',
                   'src/track/signals/track_glo_l2of.c',
                   'src/track/signals/track_qzss_l2c.c',
                   'src/track/signals/track_gal_e1.c',
                   'src/track/signals/track_gps_l1ca.c')

mesta_src += files('src/track/track_cn0.c',
            'src/track/track_common.c',
            'src/track/track_profile_utils.c',
            'src/track/track_decode.c',
            'src/track/track_profile_loop.c',
            'src/track/track_profiles.c',
            'src/track/track_sid_db.c',
            'src/track/track_state.c',
            'src/track/track_utils.c',
            'src/track/track_api.c',
            'src/track/track_timer.c',
            'src/track/track_profile_corrs.c',
            'src/track/track_flags.c',
            'src/track/track_interface.c')

mesta_src += files('src/cn0_est/cn0_est_bl.c',
                   'src/cn0_est/cn0_est_mm.c')

mesta_src += files('src/utils/filters/filter_lp1.c')
mesta_src += files('src/utils/lock_detector/lock_detector.c')
mesta_src += files('src/utils/track_loop/track_pll2_dll1.c',
                   'src/utils/track_loop/track_pll3_fll2_dll1.c',
                   'src/utils/track_loop/trk_loop_common.c')
mesta_src += files('src/utils/signal_db/signal_db.c')
mesta_src += files('src/utils/bit_sync/bit_sync.c')
mesta_src += files('src/utils/timing/timing.c')
mesta_src += files('src/utils/nav_bit_fifo/nav_bit_fifo.c')
mesta_src += files('src/utils/nav_data_sync/nav_data_sync.c')
mesta_src += files('src/utils/gnss_capabilities/gnss_capabilities.c')
mesta_src += files('src/utils/shm/shm.c')
mesta_src += files('src/utils/ephemeris/ephemeris.c')
mesta_src += files('src/utils/obs_bias/obs_bias.c')
mesta_src += files('src/utils/nmea/nmea.c')
mesta_src += files('src/utils/sbas_select/sbas_select.c')
mesta_src += files('src/utils/sv_visibility/sv_visibility.c')
mesta_src += files('src/utils/dum/dum.c')
mesta_src += files('src/utils/ext_events/ext_events.c')
mesta_src += files('src/utils/clock_filter/clock_filter.c')

mesta_src += files('src/hal/piksi_systime.c')

mesta_src += files('src/ndb/ndb.c',
                   'src/ndb/ndb_gnss_capabilities.c',
                   'src/ndb/ndb_internal.c',
                   'src/ndb/ndb_iono.c',
                   'src/ndb/ndb_lgf.c',
                   'src/ndb/ndb_utc.c',
                   'src/ndb/ndb_almanac.c',
                   'src/ndb/ndb_ephemeris.c')

mesta_src += files('src/sbp/sbp.c',
                   'src/sbp/sbp_utils.c')

mesta_src += files('src/acq/manage.c')

mesta_src += files('src/reacq/reacq_manage.c',
                   'src/reacq/reacq_sbp_utility.c',
                   'src/reacq/scheduler.c',
                   'src/reacq/scheduler_utils.c',
                   'src/reacq/search_manager.c',
                   'src/reacq/search_manager_utils.c',
                   'src/reacq/task_generator.c')

mesta_src += files('src/soft_macq/bds2_prns.c',
                   'src/soft_macq/gal_prns.c',
                   'src/soft_macq/prns.c',
                   'src/soft_macq/qzss_prns.c',
                   'src/soft_macq/soft_macq_main.c',
                   'src/soft_macq/soft_macq_mdbzp.c',
                   'src/soft_macq/soft_macq_serial.c',
                   'src/soft_macq/soft_macq_utils.c')

mesta_src += files('src/decode/decode_gps_l1ca.c',
               'src/decode/decode.c',
               'src/decode/decode_gal_e7.c',
               'src/decode/decode_common.c',
               'src/decode/decode_glo_l2of.c',
               'src/decode/decode_bds_b2.c',
               'src/decode/decode_glo_l1of.c',
               'src/decode/decode_gps_l2c.c',
               'src/decode/decode_qzss_l1ca.c',
               'src/decode/decode_sbas_l1.c',
               'src/decode/decode_bds_b1.c',
               'src/decode/decode_gal_e1.c')

mesta_src += files('src/nav_msg/nav_msg_bds.c',
                'src/nav_msg/nav_msg_gal.c',
                'src/nav_msg/cnav_msg.c',
                'src/nav_msg/sbas_msg.c',
                'src/nav_msg/nav_msg_glo.c',
                'src/nav_msg/nav_msg.c',
                'src/nav_msg/cnav_msg_storage.c',
                'src/nav_msg/cons_time_storage.c')

mesta_src += files('src/calc/calc_pvt_me.c',
                   'src/calc/calc_nav_meas.c',
                   'src/calc/calc_pvt_common.c')

mesta_src += files('src/board/v3/nap/nap_common.c',
                   'src/board/v3/nap/track_channel.c')

mesta_src += files('src/board/v3/platform_signal.c',
                   'src/board/v3/ndb_fs_access.c')

mesta_src += files('src/lib/fixed_fft_r2.c')

mesta_src += files('starling/src/starling/integration/starling_input_bridge.c')
mesta_src += files('starling/src/observation.c')
mesta_src += files('starling/src/cycle_slip.c')

mesta_src += files('starling/libfec/src/viterbi27.c')

mesta_src += files('starling/third_party/libswiftnav/src/almanac.c')
mesta_src += files('starling/third_party/libswiftnav/src/bits.c')
mesta_src += files('starling/third_party/libswiftnav/src/coord_system.c')
mesta_src += files('starling/third_party/libswiftnav/src/correct_iono_tropo.c')
mesta_src += files('starling/third_party/libswiftnav/src/edc.c')
mesta_src += files('starling/third_party/libswiftnav/src/ephemeris.c')
mesta_src += files('starling/third_party/libswiftnav/src/fifo_byte.c')
mesta_src += files('starling/third_party/libswiftnav/src/glo_map.c')
mesta_src += files('starling/third_party/libswiftnav/src/glonass_phase_biases.c')
mesta_src += files('starling/third_party/libswiftnav/src/gnss_time.c')
mesta_src += files('starling/third_party/libswiftnav/src/ionosphere.c')
mesta_src += files('starling/third_party/libswiftnav/src/linear_algebra.c')
mesta_src += files('starling/third_party/libswiftnav/src/logging.c')
mesta_src += files('starling/third_party/libswiftnav/src/logging_common.c')
mesta_src += files('starling/third_party/libswiftnav/src/memcpy_s.c')
mesta_src += files('starling/third_party/libswiftnav/src/nav_meas.c')
mesta_src += files('starling/third_party/libswiftnav/src/set.c')
mesta_src += files('starling/third_party/libswiftnav/src/shm.c')
mesta_src += files('starling/third_party/libswiftnav/src/sid_set.c')
mesta_src += files('starling/third_party/libswiftnav/src/signal.c')
mesta_src += files('starling/third_party/libswiftnav/src/single_epoch_solver.c')
mesta_src += files('starling/third_party/libswiftnav/src/troposphere.c')

mesta_src += files('libsettings/src/settings.c')
mesta_src += files('libsettings/src/registration_state.c')

mesta_src += files('apps/mesta/stubs/stubs.c')
mesta_src += files('apps/mesta/main.c')

mesta = executable('mesta', mesta_src,
                  c_args: mesta_args,
                  dependencies: libm,
                  include_directories: mesta_inc)
