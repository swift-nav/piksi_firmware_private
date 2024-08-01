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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OCEAN_TIDE_LOADING_MODEL_H
#define LIBSWIFTNAV_PVT_ENGINE_OCEAN_TIDE_LOADING_MODEL_H

#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>

namespace pvt_engine {

class OceanTideLoadingModel {
 public:
  OceanTideLoadingModel()
      : date_{0, 0, 0, 0, 0},
        itmsave_{0, 0, 0, 0, 0},
        tdfrph_d_{0, 0, 0, 0, 0, 0},
        tdfrph_dd_{0, 0, 0, 0, 0, 0} {}

  void hardisp(const utc_tm &utc_time,
               const double odisp[NUM_TIDE_LOADING_PARAMS],
               Eigen::Vector3d *ned);

 private:
  int admint(double *ampin, const int *idtin, double *phin, double *amp,
             double *f, double *p, int nin, int *nout);
  int etutc(double year, double *delta);
  double eval(double y, int *nn, double *x, double *u, double *s);
  int juldat(int *it);
  int leap(const int *iy);
  int mday(const int *iy, int m);
  int recurs(double *x, int n, double *hc, int nf, double *om, double *scr);
  int shells(double *x, int *k, int n);
  int spline(int nn, double *x, double *u, double *s, double *a);
  int tdfrph(const int *idood, double *freq, double *phase);
  int toymd(int *it1, int *it2);

  int date_[5];
  int itmsave_[5];
  double tdfrph_d_[6];
  double tdfrph_dd_[6];
};

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_OCEAN_TIDE_LOADING_MODEL_H
