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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_COMMON_TIDES_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_COMMON_TIDES_H

#include <pvt_common/eigen_custom.h>
#include <pvt_engine/pvt_return_codes.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>

constexpr s32 NUM_TIDE_LOADING_PARAMS = 6 * 11;

namespace pvt_engine {

struct erpd_t {    /* earth rotation parameter data type */
  double mjd;      /* mjd (days) */
  double xp, yp;   /* pole offset (rad) */
  double xpr, ypr; /* pole offset rate (rad/day) */
  double ut1_utc;  /* ut1-utc (s) */
  double lod;      /* length of day (s/day) */

  bool operator==(const erpd_t &rhs) const;
};

struct erp_t {            /* earth rotation parameter type */
  s16 n;                  /* number and max number of data */
  erpd_t data[WEEK_DAYS]; /* earth rotation parameter data */

  bool operator==(const erp_t &rhs) const;
};

u8 geterp(const erp_t &erp, const utc_tm &utc_time, double erpv[5]);
double get_station_correction(const Eigen::Vector3d &sat_pos,
                              const Eigen::Vector3d &coords,
                              const Eigen::Vector3d &corr_coords);

class ERPHandler {
 public:
  enum ERP_type { NONE, PREVIOUS_ULTRA_RAPIDS, ULTRA_RAPIDS, RAPIDS, FINALS };

  ERPHandler()
      : external_data_path(), current_time(), current_type(NONE), erp() {}
  PRC finderp(const gps_time_t &time);
  const erp_t &geterp() const { return erp; }
  void update_config(const std::string &external_data_path_);
  ERP_type get_type() const { return current_type; }

 private:
  PRC get_finals(const gps_time_t &time);
  PRC get_rapids(const gps_time_t &time);
  PRC get_ultra_rapids(const gps_time_t &time);
  PRC get_previous_ultra_rapids(const gps_time_t &time);
  void get_previous_ultra_rapid_time(const gps_time_t &time,
                                     s32 *six_hours_earlier_week,
                                     s32 *six_hours_earlier_day,
                                     s32 *six_hours_earlier_quarter_day);
  PRC read_and_set_erp(const char *file, const gps_time_t &time,
                       const ERP_type &type);

  std::string external_data_path;
  gps_time_t current_time;
  ERP_type current_type;
  erp_t erp;
};

class CommonTides {
 public:
  CommonTides();

  void sunmoonpos(const gps_time_t &time, const ERPHandler &erp,
                  Eigen::Vector3d *rsun, Eigen::Vector3d *rmoon, double *gmst);

 private:
  void eci2ecef(const utc_tm &utc_time, const double erpv[5],
                Eigen::Matrix3d *U, double *gmst);
  double utc_time_mjd_;
  double gmst_;
  Eigen::Matrix3d U_;
};

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_COMMON_TIDES_H
