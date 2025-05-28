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

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_COMMON_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_COMMON_H

#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#include <pvt_common/containers/map.h>
#include <pvt_engine/azimuth_elevation.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sbas/sbas.h>

#include <pvt_common/optional.h>

namespace pvt_engine {

// Band 0 - Band 8: 201 * 8 + 200 = 1808
// Band 9: 72 (60N) + 0 (65N) + 36 (70N) + 0 (75N) + 8 (85N) = 116
// Band 10: 72 (60S) + 0 (65S) + 36 (70S) + 0 (75S) + 8 (85S) = 116
// Total: 1808 + 116 * 2 = 2040
// Note that the provided number is the maximum number of different
// IGP points covering the whole world as per MOPS.
// In practice, only a fraction of these points are going to be used.
// For example, we would only use probably half of the points in the bands 0-8,
const u16 cSbasMaxIgpsNum = 2040;  // max number of ionospheric grid points
const u8 cIgpIodiMax = 3;
const double cSbasMaxElectronDensityHeightM = 350e3;

// We can use longer time-out for SBAS ionosphere model, because latent SBAS
// ionosphere model is still better option for SPP than Klobuchar without
// updated parameters.
const u16 cSbasIonosphericCorrectionsTimeOutS = 3600;
const u8 cSbasIgpPerBand = 201;
const u8 cSbasIgpBandNum = 11;

const double cSbasMinIonoCellSizeDeg = 5;
const double cSbasMinIonoCellSizeM =
    cSbasMinIonoCellSizeDeg * (2 * WGS84_A * M_PI / 360);

const double cSbasIppMaxErrorM = 0.01;
const double cSbasIppMaxErrorDeg =
    (cSbasMinIonoCellSizeDeg / cSbasMinIonoCellSizeM) * cSbasIppMaxErrorM;
const double cSbasIppMaxErrorNorm = (cSbasIppMaxErrorM / cSbasMinIonoCellSizeM);

// Low bands are considered to be bands 0 to 8
// They have different characteristics in MOPS Table A-14.
const u8 cLowBandMax = 8;
const u8 cLowBandRowDim = 8;
// High bands are considered to be bands 9 to 10
// They have different characteristics in MOPS Table A-14.
const u8 cHighBandMax = 10;
const u8 cHighBandRowDim = 5;
const s16 cBand0To8LonStartDeg = -180;
const s16 cBand9LatStartDeg = 60;
const s16 cBand10LatStartDeg = -60;

const s8 cDelta5Deg = 5;
const s8 cLowBandLatDeltaLimitDeg = 55;
const s8 cHighBandLonDeltaDeg[cHighBandRowDim] = {5, 10, 10, 10, 30};

struct IonoMaskBands {
  s16 start_deg[cLowBandRowDim];
  u8 bit_idx[cLowBandRowDim];
};

// This table is used to compute IGP for given idx,
// implementing MOPS Table A-14.
const IonoMaskBands cImb[cSbasIgpBandNum] = {
    {{-75, -55, -75, -55, -75, -55, -75, -55},  // Band0
     {28, 51, 78, 101, 128, 151, 178, 201}},
    {{-85, -55, -75, -55, -75, -55, -75, -55},  // Band1
     {28, 51, 78, 101, 128, 151, 178, 201}},
    {{-75, -55, -75, -55, -75, -55, -75, -55},  // Band2
     {27, 50, 78, 101, 128, 151, 178, 201}},
    {{-75, -55, -85, -55, -75, -55, -75, -55},  // Band3
     {27, 50, 78, 101, 128, 151, 178, 201}},
    {{-75, -55, -75, -55, -75, -55, -75, -55},  // Band4
     {27, 50, 77, 100, 128, 151, 178, 201}},
    {{-75, -55, -75, -55, -85, -55, -75, -55},  // Band5
     {27, 50, 77, 100, 128, 151, 178, 201}},
    {{-75, -55, -75, -55, -75, -55, -75, -55},  // Band6
     {27, 50, 77, 100, 127, 150, 178, 201}},
    {{-75, -55, -75, -55, -75, -55, -85, -55},  // Band7
     {27, 50, 77, 100, 127, 150, 178, 201}},
    {{-75, -55, -75, -55, -75, -55, -75, -55},  // Band8
     {27, 50, 77, 100, 127, 150, 177, 200}},
    {{-180, -180, -180, -180, -180, 0, 0, 0},  // Band9
     {72, 108, 144, 180, 192, 0, 0, 0}},
    {{-180, -180, -180, -180, -170, 0, 0, 0},  // Band10
     {72, 108, 144, 180, 192, 0, 0, 0}}};

enum class IonoCorrectionValidity { VALID, NOT_MONITORED, DO_NOT_USE };

enum class PiercePointBand {
  NORTH_POLE,
  N75_N85,
  N60_N75,
  N60_S60,
  S60_S75,
  S75_S85,
  SOUTH_POLE
};

struct IonoGridPoint {
  s8 lat_deg;
  s16 lon_deg;
  bool operator<(const IonoGridPoint &other) const;
  bool operator==(const IonoGridPoint &other) const {
    return (other.lat_deg == lat_deg) && (other.lon_deg == lon_deg);
  }
  bool operator!=(const IonoGridPoint &other) const {
    return !(*this == other);
  }
};

struct IonoGridBand {
  IonoGridBand() : igp(), num_points(0), timestamp(GPS_TIME_UNKNOWN) {}
  IonoGridPoint igp[cSbasIgpPerBand];
  u8 num_points;         // each band can have 0 to 201 points set
  gps_time_t timestamp;  // to handle aging properly
};

struct IonoGridPointMask {
  IonoGridPointMask() : band() {}
  IonoGridBand band[cSbasIgpBandNum];
};

struct IonoGridPointDelay {
  float iono_delay_m;    // meters, on L1
  gps_time_t timestamp;  // to handle aging properly
  IonoCorrectionValidity validity;
};

using IonoGrid = pvt_common::containers::Map<IonoGridPoint, IonoGridPointDelay,
                                             cSbasMaxIgpsNum>;

optional<double> calc_L1_iono_correction(const IonoGrid &iono_grid,
                                         const gps_time_t &epoch_time,
                                         const Eigen::Vector3d &user_pos_llh,
                                         const AzimuthElevation &sat_az_el);

double normalize_longitude_W180_to_E180(double lon_deg);

u8 getbitu8(const u8 *buff, const u32 &pos, const u8 &len);
u16 getbitu16(const u8 *buff, const u32 &pos, const u8 &len);

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_COMMON_H
