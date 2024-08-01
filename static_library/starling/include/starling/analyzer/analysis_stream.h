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

#ifndef STARLING_STREAM_ANALYZER_H_
#define STARLING_STREAM_ANALYZER_H_

// About:
// This file provides the highest-possible level interface to a Dead Reckoning
// (DR) analyzer. End-user needs to create a specialized class with specific
// input and truth formats. Example: sbp and novatel. Or rtcm and septentrio.
// etc. Specialized classes are then passed to a AnalysisStream-derived
// interface. E.g. in dr_analyzer, the specialized class
// DrSbpNovatelStreamAnalyzer is passed to the DrSbpAnalyzer interface. a
// Configuration struct can store global config parameters. More specific config
// parameters are set via set_config in specialized classes (e.g.
// DrSbpNovatelStreamAnalyzer)

#include "sensorfusion/core/error_types.h"
#include "traffic_control_config.h"

namespace starling {
namespace analyzer {

// Implementations currently supported by the factory.
enum class Implementation {
  MOCK_ANALYZER,
  SBP_NOVATEL_ANALYZER,
  RTCM_NOVATEL_ANALYZER,        // TODO (guillaume)
  SEPTENTRIO_NOVATEL_ANALYZER,  // TODO (guillaume)
};

// Base Factory Interface for DR Analyzer
class AnalysisStream {
 protected:
  AnalysisStream() = default;

  // The Traffic Controller (noted TC) is one of the most important member of
  // Starling Analyzer. It is responsible for routing input and output data,
  // triggering metrics creation and has a unique user interface
  // (dr_traffic_control_config.h) to deal with in most cases.
  std::unique_ptr<TrafficControlConfig> dr_traffic_controller_;

  // STREAM SYSTEM SPECS (nothing stream-specific goes there, only cfg
  // parameters that apply to any stream cfg)
  struct Configuration {
    bool ignore_missing_truth;
    double
        truth_min_hoz_accuracy;  // e.g. EHPE column in novatel SPAN csv files
    std::string input_truth;
    std::string input_logs;
    std::string output;
  };

 public:
  virtual ~AnalysisStream() = default;

  virtual sensorfusion::MaybeError set_config(
      const AnalysisStream::Configuration &config __attribute__((unused))) {
    return sensorfusion::Error();
  };  // NOLINT

  virtual sensorfusion::MaybeError run_analyzer() noexcept {
    return sensorfusion::Error();
  }

  virtual sensorfusion::MaybeError run_until(const gps_time_t &now
                                             __attribute__((unused))) noexcept {
    return sensorfusion::Error();
  }  // NOLINT

  const AnalysisStream::Configuration get_config_from_gflags();
  // Factory method for generating solution instances.
  // This is the only part of the public API that allows you
  // to instantiate specialized objects with this factory interface.
  static std::unique_ptr<AnalysisStream> Create(
      const Implementation implementation);
};

}  // namespace analyzer
}  // namespace starling

#endif  // STARLING_STREAM_ANALYZER_H_
