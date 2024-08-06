/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_EPHEMERIS_HANDLER_H
#define LIBSWIFTNAV_EPHEMERIS_HANDLER_H

#include <pvt_common/containers/circular_buffer.h>
#include <pvt_common/containers/map.h>
#include <pvt_common/containers/set.h>
#include <pvt_common/optional.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sat_identifier.h>
#include <pvt_engine/sbas/sbas_corrections_manager.h>
#include <starling/build/config.h>
#include <swiftnav/ephemeris.h>

namespace pvt_engine {

constexpr double AVERAGE_GPS_ORBIT_HEIGHT = 2.02e7;
constexpr double AVERAGE_GLO_ORBIT_HEIGHT = 1.91e7;
constexpr double AVERAGE_GAL_ORBIT_HEIGHT = 2.3222e7;

static constexpr s32 MAX_CODE_TYPES_ON_SATELLITE = 4;

using SatelliteCodeSet =
    pvt_common::containers::Set<code_t, MAX_CODE_TYPES_ON_SATELLITE>;

inline const SatelliteCodeSet &get_default_GPS_IIR_codes() {
  // Type II-R satellites only emit L1CA and L2P
  static const SatelliteCodeSet default_GPS_IIR_codes = {CODE_GPS_L1CA,
                                                         CODE_GPS_L2P};
  return default_GPS_IIR_codes;
}

inline const SatelliteCodeSet &get_default_GPS_IIRM_codes() {
  // Type II-RM satellites emit L1CA, L2CM and L2P but not L5I
  static const SatelliteCodeSet default_GPS_IIRM_codes = {
      CODE_GPS_L1CA, CODE_GPS_L2CM, CODE_GPS_L2P};
  return default_GPS_IIRM_codes;
}

inline const SatelliteCodeSet &get_default_GPS_IIF_IIIA_codes() {
  // Type II-F and III-A satellites emit L1CA, L2CM, L2P and L5I
  static const SatelliteCodeSet default_GPS_IIF_IIIA_codes = {
      CODE_GPS_L1CA, CODE_GPS_L2CM, CODE_GPS_L2P, CODE_GPS_L5I};
  return default_GPS_IIF_IIIA_codes;
}

inline const SatelliteCodeSet &get_default_GLO_codes() {
  static const SatelliteCodeSet default_GLO_codes = {CODE_GLO_L1OF,
                                                     CODE_GLO_L2OF};
  return default_GLO_codes;
}

inline const SatelliteCodeSet &get_default_BDS2_codes() {
  static const SatelliteCodeSet default_BDS2_codes = {CODE_BDS2_B1,
                                                      CODE_BDS2_B2};
  return default_BDS2_codes;
}

inline const SatelliteCodeSet &get_default_BDS3_codes() {
  // CODE_BDS3_B1CI, CODE_BDS3_B7I and CODE_BDS3_B3I are all emitted, but not
  // yet supported by starling, so are omitted from this set.
  static const SatelliteCodeSet default_BDS3_codes = {
      CODE_BDS2_B1, CODE_BDS2_B2, CODE_BDS3_B5I};
  return default_BDS3_codes;
}

inline const SatelliteCodeSet &get_default_GAL_codes() {
  static const SatelliteCodeSet default_GAL_codes = {CODE_GAL_E1B, CODE_GAL_E7I,
                                                     CODE_GAL_E5I};
  return default_GAL_codes;
}

class SatelliteInformation {
 public:
  SatelliteInformation();
  SatelliteInformation(const satellite_orbit_type_t &sat_orbit_type,
                       const SatelliteCodeSet &enabled_codes);

  satellite_orbit_type_t get_satellite_orbit_type() const;
  const SatelliteCodeSet &get_enabled_codes() const;

 private:
  satellite_orbit_type_t sat_orbit_type_;
  SatelliteCodeSet enabled_codes_;
};

class SatelliteInformationHandler {
 public:
  SatelliteInformationHandler();

  satellite_orbit_type_t get_satellite_orbit_type(
      const SatIdentifier &sat_id) const;
  SatelliteCodeSet get_enabled_codes(const SatIdentifier &sat_id) const;
  pvt_common::containers::Set<FREQUENCY, MAX_FREQUENCY> get_enabled_frequencies(
      const SatIdentifier &sat_id) const;
  static sub_constellation_t get_sub_constellation(const SatIdentifier &sat);
  SatIdSet get_all_available_satellites(sub_constellation_t sub_constel) const;

 private:
  pvt_common::containers::Map<SatIdentifier, SatelliteInformation, cNumSat>
      satellite_info_;
};

constexpr int IODE_HISTORY_DEPTH = 3;
constexpr int EPH_KEY_HISTORY_DEPTH = 3;

using EphemerisKey = u16;

EphemerisKey make_ephemeris_key(const ephemeris_t &eph);

// callback function for acquiring exclusive lock
typedef void (*get_eph_lock_cb_t)(void *);
// callback function for releasing the lock
typedef void (*release_eph_lock_cb_t)(void *);

class EphemerisHandlerInterface {
 public:
  virtual ~EphemerisHandlerInterface() = default;
  virtual void clear() = 0;

  virtual void add_ephemeris(const ephemeris_t *ephe) = 0;
  virtual void add_ephemerides(size_t num_ephs,
                               const ephemeris_t *stored_ephs[]) = 0;

  virtual optional<SatPVA> calc_sat_pva(const gnss_signal_t &sid,
                                        const gps_time_t &t) const = 0;

  virtual optional<SatPVA> calc_sat_pva(
      const gnss_signal_t &sid, const gps_time_t &t,
      optional<EphemerisKey> preferred_key) const = 0;

  virtual void set_sbas_corrections_manager(
      ISBASCorrectionsManager *sbas_has_corrections) = 0;

  virtual satellite_orbit_type_t get_satellite_orbit_type(
      const SatIdentifier &sat_id) const = 0;
  virtual SatelliteCodeSet get_enabled_codes(
      const SatIdentifier &sat_id) const = 0;
  virtual pvt_common::containers::Set<FREQUENCY, MAX_FREQUENCY>
  get_enabled_frequencies(const SatIdentifier &sat_id) const = 0;
  virtual const SatelliteInformationHandler &get_satellite_information()
      const = 0;

  virtual optional<const ephemeris_t *> pick_ephemeris(
      const gnss_signal_t &sid, const gps_time_t &t,
      optional<EphemerisKey> preferred_key) const = 0;

  virtual optional<ephemeris_t> most_recent_ephemeris(
      const pvt_engine::SatIdentifier &sat_id) const = 0;
};

class EphemerisHandler : public EphemerisHandlerInterface {
 public:
  // non-locking version
  EphemerisHandler()
      : ephemeris_map_(),
        sbas_corrections_(),
        get_lock_cb_(nullptr),
        release_lock_cb_(nullptr),
        lock_cb_ctx_(nullptr),
        sat_information_handler() {}

  // locking version requires callback functions to the mutex implementation
  EphemerisHandler(get_eph_lock_cb_t get_lock_cb,
                   release_eph_lock_cb_t release_lock_cb, void *lock_cb_ctx)
      : ephemeris_map_(),
        sbas_corrections_(),
        get_lock_cb_(get_lock_cb),
        release_lock_cb_(release_lock_cb),
        lock_cb_ctx_(lock_cb_ctx),
        sat_information_handler() {}

  void clear() override;

  void add_ephemeris(const ephemeris_t *ephe) override;
  void add_ephemerides(size_t num_ephs,
                       const ephemeris_t *stored_ephs[]) override;

  optional<SatPVA> calc_sat_pva(const gnss_signal_t &sid,
                                const gps_time_t &t) const override;

  optional<SatPVA> calc_sat_pva(
      const gnss_signal_t &sid, const gps_time_t &t,
      optional<EphemerisKey> preferred_key) const override;

  void set_sbas_corrections_manager(
      ISBASCorrectionsManager *sbas_has_corrections) override;

  satellite_orbit_type_t get_satellite_orbit_type(
      const SatIdentifier &sat_id) const override;
  SatelliteCodeSet get_enabled_codes(
      const SatIdentifier &sat_id) const override;
  pvt_common::containers::Set<FREQUENCY, MAX_FREQUENCY> get_enabled_frequencies(
      const SatIdentifier &sat_id) const override;
  const SatelliteInformationHandler &get_satellite_information() const override;

  optional<const ephemeris_t *> pick_ephemeris(
      const gnss_signal_t &sid, const gps_time_t &t,
      optional<EphemerisKey> preferred_key) const override;

  optional<ephemeris_t> most_recent_ephemeris(
      const pvt_engine::SatIdentifier &sat_id) const override;

 private:
  using EphemerisBuffer =
      pvt_common::containers::CircularBuffer<ephemeris_t,
                                             EPH_KEY_HISTORY_DEPTH>;
  using EphemerisMap =
      pvt_common::containers::Map<SatIdentifier, EphemerisBuffer, cNumSat>;
  EphemerisMap ephemeris_map_;
  ISBASCorrectionsManager *sbas_corrections_;
  get_eph_lock_cb_t get_lock_cb_;
  release_eph_lock_cb_t release_lock_cb_;
  void *lock_cb_ctx_;
  SatelliteInformationHandler sat_information_handler;

  optional<SatPVA> calc_sat_pva(const ephemeris_t &eph,
                                const gps_time_t &t) const;

  // wrapper around the get/release lock callbacks
  class LockToken {
    release_eph_lock_cb_t release_cb_;
    void *ctx_;

   public:
    LockToken(get_eph_lock_cb_t get_lock_cb, release_eph_lock_cb_t release_cb,
              void *ctx)
        : release_cb_(release_cb), ctx_(ctx) {
      if (nullptr != get_lock_cb) {
        assert(nullptr != release_cb);
        get_lock_cb(ctx_);
      }
    }

    ~LockToken() {
      if (nullptr != release_cb_) {
        release_cb_(ctx_);
      }
    }

    LockToken(const LockToken &) = delete;
    LockToken(LockToken &&) = default;

    LockToken &operator=(const LockToken &) = delete;
    LockToken &operator=(LockToken &&) = default;
  };

  LockToken get_lock() const;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_EPHEMERIS_HANDLER_H
