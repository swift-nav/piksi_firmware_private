/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef PVT_ENGINE_COMPILED_OTL_H
#define PVT_ENGINE_COMPILED_OTL_H

#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/otl_params.h>
#include <pvt_engine/ssr_grid.h>
#include <array>

namespace pvt_engine {

class OtlMetadata {
 public:
  OtlMetadata(const char *const name, double resolution,
              const BoundingBoxLatLonDeg &bounding_box, size_t num_lats,
              size_t num_lons);

  const char *name() const;
  double resolution() const;
  BoundingBoxLatLonDeg bounding_box() const;
  size_t num_lats() const;
  size_t num_lons() const;

 private:
  const char *const name_;
  const double resolution_;
  const BoundingBoxLatLonDeg bounding_box_;
  const size_t num_lats_;
  const size_t num_lons_;
};

using OtlParamsRawVec = std::array<float, NUM_TIDE_LOADING_PARAMS>;

class OtlDataset {
 public:
  OtlDataset(const OtlMetadata &metadata, const OtlParamsRawVec *dataset);
  const OtlParamsRawVec *dataset() const;
  const OtlMetadata &metadata() const;
  bool is_inside(const LatLonDeg &point) const;
  OtlParamsVec get_param_vec(const LatLonDeg &point) const;

 private:
  void find_nearest_indices(const LatLonDeg &point, size_t *lat_index,
                            size_t *lon_index) const;
  const OtlMetadata &metadata_;
  const OtlParamsRawVec *dataset_;
};

optional<OtlParamsVec> get_compiled_otl(const Eigen::Vector3d &position_ecef);
optional<OtlParamsVec> get_compiled_otl(const LatLonDeg &position_ll);
bool have_compiled_otl();

}  // namespace pvt_engine

#endif
