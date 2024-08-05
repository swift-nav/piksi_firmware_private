/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_AMBIGUITY_H
#define LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_AMBIGUITY_H

#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/observation_model_ionosphere.h>
#include <pvt_engine/process_model_no_internal_state.h>

namespace pvt_engine {

constexpr s32 PROC_NOISE_MODELS = 3;

class ProcNoiseBase {
 public:
  explicit ProcNoiseBase(const ProcessModelConfiguration &config);
  virtual double get_process_noise(const double &delta_time,
                                   const gnss_signal_t &first,
                                   const gnss_signal_t &second) const = 0;
  virtual ~ProcNoiseBase() = default;

 protected:
  const ProcessModelConfiguration &config_;
};

class AmbiguityProcNoise : public ProcNoiseBase {
 public:
  explicit AmbiguityProcNoise(const ProcessModelConfiguration &config);
  double get_process_noise(const double &delta_time, const gnss_signal_t &first,
                           const gnss_signal_t &second) const override;
};

class SatOrbitClockProcNoise : public ProcNoiseBase {
 public:
  explicit SatOrbitClockProcNoise(const ProcessModelConfiguration &config);
  double get_process_noise(const double &delta_time, const gnss_signal_t &first,
                           const gnss_signal_t &second) const override;
};

class IonoProcNoise : public ProcNoiseBase {
 public:
  IonoProcNoise(const ProcessModelConfiguration &config,
                const CommonData &common_data,
                const ObservationModelIonosphere &iono_model);
  double get_process_noise(const double &delta_time, const gnss_signal_t &first,
                           const gnss_signal_t &second) const override;

 private:
  bool add_process_noise(const gnss_signal_t &first,
                         const gnss_signal_t &second) const;
  double calculate_decorrelation_factor() const;

  const CommonData &common_data_;

  const double decorrelation_factor_;

  const ObservationModelIonosphere &iono_model_;
};

struct ProcNoiseWrapper {
  ProcNoiseWrapper();
  explicit ProcNoiseWrapper(const ProcNoiseBase *proc_model);
  const ProcNoiseBase *proc_model_;
};

class ProcNoiseHandler {
 public:
  ProcNoiseHandler(const ProcessModelConfiguration &config,
                   const CommonData &common_data,
                   const ObservationModelIonosphere &iono_model);
  double get_process_noise(const double &delta_time, const gnss_signal_t &first,
                           const gnss_signal_t &second) const;

 private:
  AmbiguityProcNoise amb_proc_noise_;
  SatOrbitClockProcNoise sat_orbit_clock_proc_noise_;
  IonoProcNoise iono_proc_noise_;
  pvt_common::containers::StaticVector<ProcNoiseWrapper, PROC_NOISE_MODELS>
      proc_noise_models_;
};

class ProcessModelAmbiguity : public ProcessModelNoInternalState {
 public:
  explicit ProcessModelAmbiguity(
      const CommonData &common_data, ObservationModelInterface *obs_model,
      const ProcessModelConfiguration &config,
      OBSERVATION_MODEL_POSITION_MODE position_model_mode,
      const ObservationModelIonosphere &iono_model);
  ProcessModelAmbiguity &operator=(const ProcessModelAmbiguity & /*rhs*/) {
    return *this;
  }
  PRC initialize(const ProcessModelConfiguration &config,
                 OBSERVATION_MODEL_POSITION_MODE position_model_mode) override;

  // Returns the part of the F and Q matrix defined within this observation
  // model.
  PRC get_process_model(const double &delta_time, MatrixMaxStateDimd_t *F,
                        MatrixMaxStateDimd_t *Q,
                        DoF_container *dof_loss) const override;

 private:
  double get_dof_loss(const ProcNoiseHandler &proc_noise_handler,
                      const double &delta_time, const gnss_signal_t &sid) const;

  const CommonData &common_data_;
  const ObservationModelIonosphere &iono_model_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_PROCESS_MODEL_AMBIGUITY_H
