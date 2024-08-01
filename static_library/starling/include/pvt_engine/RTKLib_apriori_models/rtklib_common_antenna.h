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

#ifndef LIBSWIFTNAV_COMMON_ANTENNA_H
#define LIBSWIFTNAV_COMMON_ANTENNA_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/pvt_types.h>
#include <swiftnav/gnss_time.h>
#include "rtklib_common_satellite_systems.h"

namespace pvt_engine {

constexpr s32 MAX_ANTENNA_LENGTH = 64;
constexpr s32 MAX_ELEVATION_BINS = 19;
constexpr s32 MAX_AZIMUTH_SATELLITE = 1;
constexpr s32 MAX_AZIMUTH_RECEIVER = 73;
constexpr s32 MAX_FREQUENCY_SATELLITE = 5;
constexpr s32 MAX_FREQUENCY_RECEIVER = 4;

enum class PCVFrequencyType {
  FREQUENCY_INVALID,
  FREQUENCY_G01,
  FREQUENCY_G02,
  FREQUENCY_G05,
  FREQUENCY_R01,
  FREQUENCY_R02,
  FREQUENCY_E01,
  FREQUENCY_E05,
  FREQUENCY_E06,
  FREQUENCY_E07,
  FREQUENCY_E08,
  FREQUENCY_C01,
  FREQUENCY_C02,
  FREQUENCY_C06,
  FREQUENCY_C07,
  FREQUENCY_J01,
  FREQUENCY_J02,
  FREQUENCY_J05,
  FREQUENCY_J06,
  FREQUENCY_S01
};

template <s32 NB_AZIMUTH>
using PCVMatrix = Eigen::Matrix<double, NB_AZIMUTH, MAX_ELEVATION_BINS>;
using PCVRow = Eigen::Matrix<double, 1, MAX_ELEVATION_BINS>;

template <s32 NB_AZIMUTH>
class PhaseCenter {
 public:
  PhaseCenter();

  s32 get_number_azimuth() const;
  Eigen::Vector3d get_offset() const;
  const PCVMatrix<NB_AZIMUTH> &get_variation() const;
  void set_offset(const double off[3]);
  void set_variation_elevation(s32 azimuth_bin,
                               const double var[MAX_ELEVATION_BINS]);

 private:
  Eigen::Vector3d offset;
  PCVMatrix<NB_AZIMUTH> variation;
};

template <s32 num_azimuth, s32 num_frequency>
using PhaseCorrectionMap =
    pvt_common::containers::Map<PCVFrequencyType, PhaseCenter<num_azimuth>,
                                num_frequency>;

using PCSatellite =
    PhaseCorrectionMap<MAX_AZIMUTH_SATELLITE, MAX_FREQUENCY_SATELLITE>;
using PCReceiver =
    PhaseCorrectionMap<MAX_AZIMUTH_RECEIVER, MAX_FREQUENCY_RECEIVER>;

template <typename T>
class pcv_t { /* antenna parameter type */
 public:
  pcv_t();

  s32 sat;                       /* satellite number (0:receiver) */
  s32 nb_azi;                    /* number of azimuth bins */
  char type[MAX_ANTENNA_LENGTH]; /* antenna type */
  char code[MAX_ANTENNA_LENGTH]; /* serial number or satellite code */
  char svn[5];                   /* space vehicle number */
  gps_time_t start_time;         /* valid start time */
  gps_time_t end_time;           /* valid end time */
  T phase_center;                /* phase center data */
};

using SatellitePCVMap =
    pvt_common::containers::Map<s32, pcv_t<PCSatellite>, NUM_SATS>;

optional<SatellitePCVMap> read_satellite_pcv(const char *file,
                                             const gps_time_t &time);
optional<pcv_t<PCReceiver>> read_station_pcv(const char *file,
                                             const char *ant_type);

template <typename T>
double interpolate_pcv(double ang, double azi, PCVFrequencyType freq,
                       const pcv_t<T> &pcv);

void satno2id(s32 sat, char *id);
s32 satsys(s32 sat, s32 *prn);
s32 satid2no(const char *id);
s32 sid2no(const gnss_signal_t &sid);
PCVFrequencyType code2pcvfreq(const code_t &code);
PCVFrequencyType basefreq2pcvfreq(const gnss_signal_t &sid,
                                  const FREQUENCY frequency);

/* It is possible that PCV calibrations are not available for a given system.
 * For instance, only GPS PCV calibrations are available. Instead of triggering
 * an error, try selecting a GPS PCV table in a nearby frequency band.
 */
template <class T>
PCVFrequencyType select_pcvfreq(const T &pcv, PCVFrequencyType freq) {
  /* if it is a valid frequency, return this one */
  if (pcv[freq]) {
    return freq;
  }

  PCVFrequencyType new_freq = PCVFrequencyType::FREQUENCY_INVALID;
  switch (freq) {
    case PCVFrequencyType::FREQUENCY_R01:
    case PCVFrequencyType::FREQUENCY_E01:
    case PCVFrequencyType::FREQUENCY_C01:
    case PCVFrequencyType::FREQUENCY_C02:
    case PCVFrequencyType::FREQUENCY_J01:
    case PCVFrequencyType::FREQUENCY_S01:
      new_freq = PCVFrequencyType::FREQUENCY_G01;
      break;
    case PCVFrequencyType::FREQUENCY_R02:
    case PCVFrequencyType::FREQUENCY_E06:
    case PCVFrequencyType::FREQUENCY_E07:
    case PCVFrequencyType::FREQUENCY_C06:
    case PCVFrequencyType::FREQUENCY_C07:
    case PCVFrequencyType::FREQUENCY_J02:
    case PCVFrequencyType::FREQUENCY_J06:
      new_freq = PCVFrequencyType::FREQUENCY_G02;
      break;
    case PCVFrequencyType::FREQUENCY_G05:
    case PCVFrequencyType::FREQUENCY_E05:
    case PCVFrequencyType::FREQUENCY_E08:
    case PCVFrequencyType::FREQUENCY_J05:
      new_freq = PCVFrequencyType::FREQUENCY_G05;
      if (!pcv[new_freq]) {
        new_freq = PCVFrequencyType::FREQUENCY_G02;
      }
      break;
    case PCVFrequencyType::FREQUENCY_INVALID:
    case PCVFrequencyType::FREQUENCY_G01:
    case PCVFrequencyType::FREQUENCY_G02:
    default:
      break;
  }
  return new_freq;
}

};  // namespace pvt_engine

#endif  // LIBSWIFTNAV_COMMON_ANTENNA_H
