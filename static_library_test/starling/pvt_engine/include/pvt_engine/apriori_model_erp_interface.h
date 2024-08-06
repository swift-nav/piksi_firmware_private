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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_ERP_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_ERP_INTERFACE_H

#include <pvt_engine/apriori_model_interface.h>

namespace pvt_engine {

class AprioriModelErpInterface : public AprioriModelUsingCommonConfig {
 public:
  PRC initialize(const CommonAprioriModelConfiguration &config) override;
  bool update_config(const CommonAprioriModelConfiguration &config) override;

 protected:
  explicit AprioriModelErpInterface(
      const CommonAprioriModelConfiguration &config);

  ERPHandler erp_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_ERP_INTERFACE_H
