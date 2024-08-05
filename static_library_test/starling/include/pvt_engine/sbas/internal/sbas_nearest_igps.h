/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_NEAREST_IGPS_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_NEAREST_IGPS_H

#include <math.h>

#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#include <pvt_common/containers/map.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sbas/internal/sbas_iono_pierce_point.h>
#include <pvt_engine/sbas/sbas.h>

#include <pvt_engine/optional.h>

namespace pvt_engine {

// Rounding direction
enum class RoundDirection {
  S5_W5,    // round to South by 5 degrees, to West by 5 degrees
  S5_W10,   // round to South by 5 degrees, to West by 10 degrees
  S10_W5,   // round to South by 10 degrees, to West by 5 degrees
  S10_W10,  // round to South by 10 degrees, to West by 10 degrees

  N5_W5,    // round to North by 5 degrees, to West by 5 degrees
  N5_W10,   // round to North by 5 degrees, to West by 10 degrees
  N10_W5,   // round to North by 10 degrees, to West by 5 degrees
  N10_W10,  // round to North by 10 degrees, to West by 10 degrees

  S5_E5,    // round to South by 5 degrees, to East by 5 degrees
  S5_E10,   // round to South by 5 degrees, to East by 10 degrees
  S10_E5,   // round to South by 10 degrees, to East by 5 degrees
  S10_E10,  // round to South by 10 degrees, to East by 10 degrees

  N5_E5,    // round to North by 5 degrees, to East by 5 degrees
  N5_E10,   // round to North by 5 degrees, to East by 10 degrees
  N10_E5,   // round to North by 10 degrees, to East by 5 degrees
  N10_E10,  // round to North by 10 degrees, to East by 10 degrees

  CN75_W10,  // set constant N75 and round to West by 10 degrees
  CN75_E10,  // set constant N75 and round to East by 10 degrees
  CS75_W10,  // set constant S75 and round to West by 10 degrees
  CS75_E10,  // set constant S75 and round to East by 10 degrees

  CN85_W30,  // set constant N85 and round to West by 30 degrees
  CN85_E30,  // set constant N85 and round to East by 30 degrees
  CS85_W30,  // set constant S85 and round to West by 30 degrees
  CS85_E30,  // set constant S85 and round to East by 30 degrees

  CN85_W90,  // set constant N85 and round to West by 90 degrees
  CN85_E90,  // set constant N85 and round to East by 90 degrees
  CS85_W90,  // set constant S85 and round to West by 90 degrees
  CS85_E90   // set constant S85 and round to East by 90 degrees
};

struct NormalizedPoint {
  double lat;  // [0..1] unitless
  double lon;  // [0..1] unitless
};

class RoundVariant;

class NearestIgps {
 public:
  NearestIgps(const PiercePoint &point, const IonoGrid &grid)
      : iono_grid(grid), pierce_point(point), mask(0) {}

  void select(const gps_time_t &epoch_time);
  double interpolate();
  bool usable() const;
  bool have_do_not_use() const;

 private:
  IonoGridPoint igp[4];
  const IonoGrid &iono_grid;
  PiercePoint pierce_point;

  // bit1(NW,igp[1])--bit0(NE,igp[0])
  //    |                |
  // bit2(SW,igp[2])--bit3(SE,igp[3])
  u8 mask;
  bool ne_missing() const { return (0 == (mask & 1)); }
  bool nw_missing() const { return (0 == (mask & 2)); }
  bool sw_missing() const { return (0 == (mask & 4)); }
  bool se_missing() const { return (0 == (mask & 8)); }

  void remove_not_monitored();
  void remove_aged(const gps_time_t &epoch_time);

  void select_common(const RoundVariant round_variant[],
                     size_t round_variant_size);
  void select_in_north_pole();
  void select_in_south_pole();
  void select_in_n75_n85();
  void select_in_s75_s85();
  void select_in_n60_n75_s60_s75(const RoundVariant common[],
                                 size_t common_size);
  void select_in_n60_s60(const RoundVariant common[], size_t common_size);

  bool pierce_point_in_triangle() const;
  NormalizedPoint normalize_point(const double &lat_deg,
                                  const double &lon_deg) const;

  void calc_igp_delays(double (&igp_delay_m)[4]) const;
  void calc_virtual_igp_n75_n85(double (&igp_delay_m)[4]);
  void calc_virtual_igp_s75_s85(double (&igp_delay_m)[4]);
  double interpolate_by_3() const;
  double interpolate_by_4();

  IonoGridPoint calc_igp(const RoundDirection &round_direction);
  s16 calc_igp_lon(const RoundDirection &round_direction);
  s8 calc_igp_lat(const RoundDirection &round_direction);
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_NEAREST_IGPS_H
