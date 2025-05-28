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

#ifndef STARLING_PVT_ENGINE_SRR_GRID_H
#define STARLING_PVT_ENGINE_SRR_GRID_H

#include <pvt_common/optional.h>
#include <pvt_engine/eigen_types.h>
#include <swiftnav/common.h>
#include <iterator>

namespace pvt_engine {

class LatLonDeg {
 public:
  LatLonDeg(double lat, double lon);

  double lat() const;
  double lon() const;

  double straight_line_distance(const LatLonDeg &other) const;

  bool operator==(const LatLonDeg &other) const;
  bool operator!=(const LatLonDeg &other) const;

 private:
  double lat_;
  double lon_;

  bool is_valid() const;
  void normalize();
};

LatLonDeg ecef_to_lat_lon_deg(const Eigen::Vector3d &ecef);

Eigen::Vector3d to_ecef(const LatLonDeg &ll_deg, double height = 0);

class BoundingBoxLatLonDeg {
 public:
  BoundingBoxLatLonDeg(const LatLonDeg &nw_corner, double width_deg,
                       double height_deg);

  bool is_inside(const LatLonDeg &point) const;
  bool is_overlapping(const BoundingBoxLatLonDeg &other_bb) const;

  LatLonDeg nw() const;
  LatLonDeg ne() const;
  LatLonDeg sw() const;
  LatLonDeg se() const;

  double width_deg() const;
  double height_deg() const;

 private:
  LatLonDeg nw_corner_;
  double width_deg_;
  double height_deg_;
};

class BoundingBoxUnitCoords {
  BoundingBoxUnitCoords(const BoundingBoxLatLonDeg &box,
                        const LatLonDeg &point);

 public:
  static optional<BoundingBoxUnitCoords> make_checked(
      const BoundingBoxLatLonDeg &box, const LatLonDeg &point);

  double x() const { return x_; }
  double y() const { return y_; }

 private:
  double x_;
  double y_;
};

class GridCoordinates {
 public:
  GridCoordinates(double x, double y);

  double x() const;
  double y() const;

  u8 x_int() const;
  u8 y_int() const;

 private:
  double x_;
  double y_;
};

const Eigen::Index MAX_SSR_GRID_POINTS = 100;

extern const Eigen::Index NW_INDEX;
extern const Eigen::Index NE_INDEX;
extern const Eigen::Index SW_INDEX;
extern const Eigen::Index SE_INDEX;

using GridIndices = pvt_engine::containers::StaticVector<Eigen::Index, 4>;

class SsrGrid {
 public:
  SsrGrid(const LatLonDeg &nw_corner, double row_spacing_deg,
          double col_spacing_deg, u16 rows, u16 cols);
  SsrGrid(const LatLonDeg &nw_corner, double row_spacing_deg,
          double col_spacing_deg, u16 rows, u16 cols, u32 tile_set_id,
          u32 tile_id);

  optional<u32> tile_set_id() const;
  optional<u32> tile_id() const;
  u16 rows() const;
  u16 cols() const;
  u32 num_points() const;
  double row_spacing_deg() const;
  double col_spacing_deg() const;
  LatLonDeg nw_corner() const;
  LatLonDeg se_corner() const;
  LatLonDeg tile_center() const;
  BoundingBoxLatLonDeg bounding_box() const;

  GridCoordinates lat_lon_to_grid_coords(const LatLonDeg &point) const;
  LatLonDeg grid_coords_to_lat_lon(const GridCoordinates &coords) const;

  bool is_inside_grid(const GridCoordinates &coords) const;
  bool is_inside_grid(const LatLonDeg &point) const;

  bool is_overlapping_grid(const BoundingBoxLatLonDeg &bb) const;

  BoundingBoxLatLonDeg lat_lon_to_bounding_box(const LatLonDeg &point) const;
  GridIndices lat_lon_to_bounding_grid_indices(const LatLonDeg &point) const;

  LatLonDeg index_to_lat_lon(Eigen::Index index) const;

  class iterator {
   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = LatLonDeg;
    using difference_type = s32;
    using pointer = const LatLonDeg *;
    using reference = LatLonDeg;

    explicit iterator(const SsrGrid &parent, Eigen::Index index = 0);
    iterator &operator++();
    iterator operator++(int);
    bool operator==(iterator other) const;
    bool operator!=(iterator other) const;
    reference operator*() const;

   private:
    const SsrGrid &parent_;
    Eigen::Index index_;
  };
  iterator begin() const;
  iterator end() const;

  bool operator==(const SsrGrid &other) const;
  bool operator!=(const SsrGrid &other) const;

 private:
  Eigen::Index lat_lon_to_index(const LatLonDeg &point) const;

  LatLonDeg nw_corner_;
  double row_spacing_deg_;
  double col_spacing_deg_;
  u16 rows_;
  u16 cols_;
  optional<uint32_t> tile_set_id_;
  optional<uint32_t> tile_id_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_SSR_GRID_H
