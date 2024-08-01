//
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#ifndef STARLING_SBP_IO_FUZER_HELPERS_H
#define STARLING_SBP_IO_FUZER_HELPERS_H

#include <pvt_common/coord_system.h>
#include <swiftnav/pvt_result.h>
#include <deque>
#include <map>
#include "starling/util/sbp/sbp_stdio.h"

namespace starling {
namespace fuzer {

// Convert a set of field values to a pvt_engine_result_t
void make_result_from_values(pvt_engine_result_t *result, const int &gps_week,
                             const double &gps_tow, int pos_mode,
                             const double &lat, const double &lon,
                             const double &height, const double &hacc,
                             const double &vacc, const u8 &num_sats,
                             const int &valid_threshold);

// Convert fields in pvt_engine_result_t into individual values
void make_values_from_result(const pvt_engine_result_t *result, double *lat,
                             double *lon, double *height, double *time,
                             double *h_accuracy, double *v_accuracy);

// Convert individual values to msg_pos_llh_t*
void make_pos_llh_from_values(msg_pos_llh_t *msg_pos_llh, const u32 &tow,
                              const double &lat, const double &lon,
                              const double &height, const u8 &n_sats,
                              const u8 &flags, const double &h_accuracy,
                              const double &v_accuracy);

// round u32 to nearest 100ms value. 100150 is rounded to 100200
u32 round_nearest_100ms(const u32 &tow);

void display_combined_inputs(const pvt_engine_result_t input_combined[],
                             const msg_pos_llh_t &msg);

template <typename K, typename V, int BufferSize>
struct CircularMapBuffer {
  typedef std::map<K, V> circular_map_t;
  typedef std::deque<K> deque_t;

  // must define iterators for find and put
  typename circular_map_t::iterator begin() { return map_.begin(); }
  typename circular_map_t::iterator end() { return map_.end(); }
  typename circular_map_t::size_type size() { return map_.size(); }

  // Add element and check size
  void put(K k, V v) {
    auto k_rounded = round_nearest_100ms(k);
    // no rounding if it's more than 16ms
    if (fabs((double)k_rounded - (double)k) > 16) {  // NOLINT
      std::clog << std::setprecision(9) << k / 1000. << std::fixed
                << " WEIRD TOW! IGNORE this obs " << std::endl;
      return;
    }
    map_.insert(std::make_pair(k_rounded, v));
    deque_.push_back(k_rounded);
    ensure_size();  // ensure the size of the map, and remove the last element
  }

  // find of map
  typename circular_map_t::iterator find(K k) { return map_.find(k); }

 private:
  circular_map_t map_;
  deque_t deque_;

  void ensure_size() {
    if (deque_.size() > BufferSize) {
      map_.erase(deque_.front());
      deque_.pop_front();
    }
  }
};

}  // namespace fuzer
}  // namespace starling

#endif  // STARLING_SBP_IO_FUZER_HELPERS_H
