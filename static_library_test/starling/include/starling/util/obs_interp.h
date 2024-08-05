/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef STARLING_OBS_INTERPOLATOR_H_
#define STARLING_OBS_INTERPOLATOR_H_

#include "starling/starling.h"

namespace starling {
namespace util {

/**************************************************************************
 * This is a little class used for observation interpolation.
 *************************************************************************/
class ObsInterpolator {
 public:
  ObsInterpolator() noexcept;

  /* this one copies base_pre to the local storage of the linear
   * interpolator */
  void set_previous_base(const obs_core_t &base_pre) noexcept;

  /* this one takes rover and base_post and triggers the linear interpolation.
   * If it succeeds, points base_i to the local storage of thelinear
   * interpolator, where the result of the linear interpolation between base_pre
   * and base_post at the rover time should be.
   */
  bool trigger(const obs_core_t &rover, const obs_core_t &base_post,
               const obs_core_t *&base_i) noexcept;

 private:
  static constexpr double max_interp_delay_s_ = 1.0;

  bool do_interpolation(const obs_core_t &rover, const obs_core_t &base_post);
  /* internal intep function */
  bool interp_internal(const obs_core_t &base_post, const double &dt_pre,
                       const double &dt_post);
  /* Storage. */
  obs_core_t base_i_;
};

}  // namespace util
}  // namespace starling

#endif /* STARLING_OBS_INTERPOLATOR_H_ */
