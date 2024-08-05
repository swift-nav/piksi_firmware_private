///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
///////////////////////////////////////////////////////////////////////////////

#ifndef STARLING_ANALYZER_METRIC_INTERFACE_H
#define STARLING_ANALYZER_METRIC_INTERFACE_H

#include <pvt_engine/eigen_types.h>
#include <array>
#include <cmath>
#include <iostream>
#include "swiftnav/common.h"

namespace starling {
namespace analyzer {
namespace AnalyzerData {

// Fixed-size (no templating) arrays for now
constexpr u8 NFieldsPerMetric = 5;

// Union-like class to store variant members
// We use that to allow static handling for metrics values
// that are inherently of mixed types. It also limits
// instances of undesirable casting.
struct Field {  // NOLINT

 private:
  union {
    bool val_bool;
    u32 val_u32;
    double val_double;
  };

 public:
  enum { BOOL, U32, DOUBLE } FieldTag;
  // We use "safe" functional "constructors" in lieu of generic ones for
  // more guarantee on final stored value type-safety
  static Field safe_construct_field_bool(const bool &val_) {
    auto foofield = Field();
    foofield.FieldTag = Field::BOOL;
    foofield.val_bool = val_;  // NOLINT
    return foofield;
  }

  static Field safe_construct_field_u32(const u32 &val_) {
    auto foofield = Field();
    foofield.FieldTag = Field::U32;
    foofield.val_u32 = val_;  // NOLINT
    return foofield;
  }

  static Field safe_construct_field_double(const double &val_) {
    auto foofield = Field();
    foofield.FieldTag = Field::DOUBLE;
    foofield.val_double = val_;  // NOLINT
    return foofield;
  }

  // Unless contructing the object or using extraction operator
  // User must use safe setters and getters
  const bool &safe_get_field_bool() {
    assert(FieldTag == Field::BOOL);
    return val_bool;  // NOLINT
  }

  const u32 &safe_get_field_u32() {
    assert(FieldTag == Field::U32);
    return val_u32;  // NOLINT
  }

  const double &safe_get_field_double() {
    assert(FieldTag == Field::DOUBLE);
    return val_double;  // NOLINT
  }

  void safe_set_field_bool(const bool &val_) {
    FieldTag = Field::BOOL;
    val_bool = val_;  // NOLINT
  }

  void safe_set_field_u32(const u32 &val_) {
    FieldTag = Field::U32;
    val_u32 = val_;  // NOLINT
  }

  void safe_set_field_double(const double &val_) {
    FieldTag = Field::DOUBLE;
    val_double = val_;  // NOLINT
  }

  void peek_me() {
    switch (this->FieldTag) {
      case Field::BOOL:
        std::cout << "val_bool = " << this->val_bool << "\n";  // NOLINT
        break;
      case Field::U32:
        std::cout << "val_u32 = " << this->val_u32 << "\n";  // NOLINT
        break;
      case Field::DOUBLE:
        std::cout << "val_double = " << this->val_double << "\n";  // NOLINT
        break;
      default:
        exit(1);
    }
  }

  friend std::ostream &operator<<(std::ostream &os, const Field &ff_) {
    switch (ff_.FieldTag) {
      case Field::BOOL:
        os << ff_.val_bool;  // NOLINT
        return os;
      case Field::U32:
        os << ff_.val_u32;  // NOLINT
        return os;
      case Field::DOUBLE:
        os << ff_.val_double;  // NOLINT
        return os;
      default:
        exit(1);
    }
  }

  bool isnan() {
    switch (this->FieldTag) {
      case Field::U32:
        return false;
      case Field::DOUBLE:
        return std::isnan(this->val_double);  // NOLINT
      case Field::BOOL:
        return false;
      default:
        exit(1);
    }
  }
};

// MetricInterface is a generic container of Metric fields and values.
// Every DR Analyzer MsG (in dr_analyzer_data_type) holds a reference
// to one MetricInterface. This allows Traffic Controller to "manage"
// all Analyzer MsGs and flush their fields easily into Output Streams
class MetricInterface {
 public:
  struct metric_metadata_t {
    // specialized name for metric handling further down the stream
    std::string metric_name_;
    std::array<std::string, NFieldsPerMetric> metric_field_names_;
  };

  MetricInterface() = default;

  explicit MetricInterface(const MetricInterface::metric_metadata_t &mm_)
      : metric_metadata_(mm_) {}

  MetricInterface(const std::array<std::string, NFieldsPerMetric> &names,
                  const std::string &metric_name)
      : metric_metadata_({metric_name, {names}}) {}

  MetricInterface(
      const pvt_engine::containers::StaticVector<Field, NFieldsPerMetric>
          &ftypes,
      const std::array<std::string, NFieldsPerMetric> &names,
      const std::string &metric_name)
      : metric_fields_(ftypes), metric_metadata_({metric_name, {names}}) {}

  auto at(const s32 &index_) {
    assert(index_ < NFieldsPerMetric);
    return metric_fields_[index_];
  }

  // TODO (guillaume) Might want to create
  // a set of explicitly-typed const and non-const iterators
  auto begin() { return metric_fields_.begin(); }

  auto end() { return metric_fields_.end(); }

  auto append(const Field &a_field_element) {
    return metric_fields_.append(a_field_element);
  }

  auto clear() { metric_fields_.clear(); }

  auto size() const { return metric_fields_.size(); }

  auto num_fields_metadata() const {
    return metric_metadata_.metric_field_names_.size();
  }

  // if metric_field_names_ is list-initialized with fewer elements than
  // NFieldsPerMetric, default ctor of string is called, i.e. an empty string
  auto num_nonempty_fields_metadata() const {
    return NFieldsPerMetric -
           std::count(metric_metadata_.metric_field_names_.begin(),
                      metric_metadata_.metric_field_names_.end(), "");
  }

  const std::string &get_metric_name() { return metric_metadata_.metric_name_; }

 protected:
  // Metric Identification Metadata
  metric_metadata_t metric_metadata_;

  // Metric Data
  pvt_engine::containers::StaticVector<Field, NFieldsPerMetric> metric_fields_;

 private:
};
}  // namespace AnalyzerData
}  // namespace analyzer
}  // namespace starling
#endif  // STARLING_ANALYZER_METRIC_INTERFACE_H
