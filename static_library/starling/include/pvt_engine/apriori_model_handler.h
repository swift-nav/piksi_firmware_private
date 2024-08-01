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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_HANDLER_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_HANDLER_H

#include <pvt_engine/apriori_model_beidou_elevation_dependent_bias.h>
#include <pvt_engine/apriori_model_cn0_mask.h>
#include <pvt_engine/apriori_model_computed_doppler.h>
#include <pvt_engine/apriori_model_elevation.h>
#include <pvt_engine/apriori_model_general_relativity.h>
#include <pvt_engine/apriori_model_interface.h>
#include <pvt_engine/apriori_model_ionosphere.h>
#include <pvt_engine/apriori_model_loss_of_lock.h>
#include <pvt_engine/apriori_model_min_max_sats.h>
#include <pvt_engine/apriori_model_ocean_tide_loading.h>
#include <pvt_engine/apriori_model_phase_windup.h>
#include <pvt_engine/apriori_model_pole_tides.h>
#include <pvt_engine/apriori_model_receiver_antenna_offset.h>
#include <pvt_engine/apriori_model_satellite_antenna_offset.h>
#include <pvt_engine/apriori_model_sbas.h>
#include <pvt_engine/apriori_model_solid_earth_tides.h>
#include <pvt_engine/apriori_model_troposphere.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

class AprioriModelHandler {
 public:
  explicit AprioriModelHandler(const CommonData &common_data,
                               const AprioriModelConfiguration &config);

  AprioriModelHandler(const AprioriModelHandler &other);

  AprioriModelHandler &operator=(const AprioriModelHandler &other);

  PRC initialize(const AprioriModelTypes &model_types_,
                 const AprioriModelConfiguration &config);

  bool update_config(const AprioriModelConfiguration &config);

  void update_otl_params(const OtlParamsVec &params);

  bool otl_params_loaded() const;

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler);

 private:
  struct AprioriModel {
    AprioriModel() : model(nullptr), model_type_(INVALID_APRIORI_MODEL) {}

    AprioriModelInterface *model;
    APRIORI_MODEL_TYPE model_type_;
  };

  using AprioriModels =
      pvt_common::containers::StaticVector<AprioriModel, MAX_APRIORI_MODEL_NUM>;

  AprioriModelTypes get_model_types(const AprioriModels &models) const;

  void setup_models(const AprioriModelTypes &model_types);

  void clear_model_list();

  AprioriModel setup_model(const APRIORI_MODEL_TYPE model_type);

  AprioriModels models_;

  AprioriModelElevation elevation_model_;
  AprioriModelCn0Mask cn0_mask_model_;
  AprioriModelIonosphere ionosphere_model_;
  AprioriModelTroposphere troposphere_model_;
  AprioriModelOceanTideLoading ocean_tide_loading_model_;
  AprioriModelPhaseWindup phase_windup_model_;
  AprioriModelPoleTides pole_tides_model_;
  AprioriModelReceiverAntennaOffset receiver_antenna_offset_model_;
  AprioriModelSatelliteAntennaOffset satellite_antenna_offset_model_;
  AprioriModelSolidEarthTides solid_earth_tides_model_;
  AprioriModelMinMaxSats min_max_sat_model_;
  AprioriModelLossOfLock loss_of_lock_model_;
  AprioriModelComputedDoppler computed_doppler_model_;
  AprioriModelGeneralRelativity general_relativity_model_;
  AprioriModelSBAS sbas_model_;
  AprioriModelBeidouElevationDependentBias beidou_code_bias_model_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_HANDLER_H
