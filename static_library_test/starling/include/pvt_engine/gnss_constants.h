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

#ifndef LIBSWIFTNAV_PVT_ENGINE_GNSS_CONSTANTS_H
#define LIBSWIFTNAV_PVT_ENGINE_GNSS_CONSTANTS_H

#include <cmath>

#include <pvt_engine/pvt_return_codes.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/signal.h>

#include <cassert>

namespace pvt_engine {

/** \defgroup gnss_constants GNSS Constants
 * Defines constants specified in the GNSS constellations' ICDs.
 * \{ */

namespace gps {
/** The GPS L1 center frequency in Hz. */
constexpr double cL1Hz = GPS_L1_HZ;

/** The GPS L2 center frequency in Hz. */
constexpr double cL2Hz = GPS_L2_HZ;

/** The official GPS value of Pi.
 * \note This is the value used by the CS to curve fit ephemeris parameters and
 * should be used in all ephemeris calculations. */
constexpr double cPi = GPS_PI;

/** Earth's rotation rate as defined in the ICD in rad / s
 * \note This is actually not identical to the usual WGS84 definition. */
constexpr double cOmegaeDot = GPS_OMEGAE_DOT;

/** Earth's Gravitational Constant as defined in the ICD in m^3 / s^2
 * \note This is actually not identical to the usual WGS84_GM definition. */
constexpr double cGM = GPS_GM;

/** The official GPS value of the speed of light in m / s.
 * \note This is the exact value of the speed of light in vacuum (by the
 * definition of meters). */
constexpr double cC = GPS_C;

/** The official GPS value of the relativistic clock correction coefficient F.
 */
constexpr double cF = GPS_F;

/** WGS84 semi-major axis of Earth in m */
constexpr double cAE = WGS84_A;

/** Second zonal harmonic of the geopotential */
constexpr double cJ02 = WGS84_J02;
}  // namespace gps

namespace glonass {
/** The GLO L1 center frequency in Hz. */
constexpr double cL1Hz = GLO_L1_HZ;

/** The GLO L2 center frequency in Hz. */
constexpr double cL2Hz = GLO_L2_HZ;

/** The value of Pi
 * \note There does not appear to be a special GLONASS value. */
constexpr double cPi = M_PI;

/** Earth's rotation rate as defined in the GLO ICD in rad / s  */
constexpr double cOmegaeDot = GLO_OMEGAE_DOT;

/** Earth's Gravitational Constant as defined in the GLO ICD in m^3 / s^2 */
constexpr double cGM = GLO_GM;

/** The speed of light in m / s.
 * \note This is the exact value of the speed of light in vacuum (by the
 * definition of meters). */
constexpr double cC = GPS_C;  // Same value as GPS

/** GLO semi-major axis of Earth
 * NOTE: there is define WGS84_A which is 6378137, differ than defined
 * in GLO ICD, refer A.3.1.2. */
constexpr double cAE = GLO_A_E;

/** Second zonal harmonic of the geopotential */
constexpr double cJ02 = GLO_J02;
}  // namespace glonass

namespace gnss {
/** Retrieves the value of Pi for the specified GNSS constellation.
 *
 * \param constellation The constellation to return the value for
 * \param pi Pointer to variable to return the value in
 * \return RC_S_OK if successful. Otherwise value was not set
 */
inline PRC pi(constellation_t constellation, double *pi) {
  assert(pi != nullptr);

  PRC rc(RC_S_OK);
  switch (constellation) {
    case CONSTELLATION_GPS:
      *pi = gps::cPi;
      break;

    case CONSTELLATION_GLO:
      *pi = glonass::cPi;
      break;

    case CONSTELLATION_SBAS:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_BDS:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      rc = RC_E_NOTOK;
      log_error("%s", prc::message(rc));
      assert(prc::success(rc));
      break;
  }
  return rc;
}

/** Retrieves the value of the Earth's rotation rate for the specified GNSS
 * constellation.
 *
 * \param constellation The constellation to return the value for
 * \param omegaeDot Pointer to variable to return the value in
 * \return RC_S_OK if successful. Otherwise value was not set
 */
inline PRC omegaeDot(constellation_t constellation, double *omegaeDot) {
  assert(omegaeDot != nullptr);

  PRC rc(RC_S_OK);
  switch (constellation) {
    case CONSTELLATION_GPS:
      *omegaeDot = gps::cOmegaeDot;
      break;

    case CONSTELLATION_GLO:
      *omegaeDot = glonass::cOmegaeDot;
      break;

    case CONSTELLATION_SBAS:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_BDS:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      rc = RC_E_NOTOK;
      log_error("%s", prc::message(rc));
      assert(prc::success(rc));
      break;
  }
  return rc;
}

/** Retrieves the value of the gravitational constant for the specified GNSS
 * constellation.
 *
 * \param constellation The constellation to return the value for
 * \param gm Pointer to variable to return the value in
 * \return RC_S_OK if successful. Otherwise value was not set
 */
inline PRC gm(constellation_t constellation, double *gm) {
  assert(gm != nullptr);

  PRC rc(RC_S_OK);
  switch (constellation) {
    case CONSTELLATION_GPS:
      *gm = gps::cGM;
      break;

    case CONSTELLATION_GLO:
      *gm = glonass::cGM;
      break;

    case CONSTELLATION_SBAS:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_BDS:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      rc = RC_E_NOTOK;
      log_error("%s", prc::message(rc));
      assert(prc::success(rc));
      break;
  }
  return rc;
}

/** Retrieves the value of the speed of light for the specified GNSS
 * constellation.
 *
 * \param constellation The constellation to return the value for
 * \param c Pointer to variable to return the value in
 * \return RC_S_OK if successful. Otherwise value was not set
 */
inline PRC c(constellation_t constellation, double *c) {
  assert(c != nullptr);

  PRC rc(RC_S_OK);
  switch (constellation) {
    case CONSTELLATION_GPS:
      *c = gps::cC;
      break;

    case CONSTELLATION_GLO:
      *c = glonass::cC;
      break;

    case CONSTELLATION_SBAS:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_BDS:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      rc = RC_E_NOTOK;
      log_error("%s", prc::message(rc));
      assert(prc::success(rc));
      break;
  }
  return rc;
}

/** Retrieves the value of the relativistic clock correction coefficient for
 * the specified GNSS constellation.
 *
 * \param constellation The constellation to return the value for
 * \param f Pointer to variable to return the value in
 * \return RC_S_OK if successful. Otherwise value was not set
 */
inline PRC f(constellation_t constellation, double *f) {
  assert(f != nullptr);

  PRC rc(RC_S_OK);
  switch (constellation) {
    case CONSTELLATION_GPS:
      *f = gps::cF;
      break;

    case CONSTELLATION_GLO:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_SBAS:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_BDS:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      rc = RC_E_NOTOK;
      log_error("%s", prc::message(rc));
      assert(prc::success(rc));
      break;
  }
  return rc;
}

/** Retrieves the value of the semi-major axis of Earth for
 * the specified GNSS constellation.
 *
 * \param constellation The constellation to return the value for
 * \param ae Pointer to variable to return the value in
 * \return RC_S_OK if successful. Otherwise value was not set
 */
inline PRC ae(constellation_t constellation, double *ae) {
  assert(ae != nullptr);

  PRC rc(RC_S_OK);
  switch (constellation) {
    case CONSTELLATION_GPS:
      *ae = gps::cAE;
      break;

    case CONSTELLATION_GLO:
      *ae = glonass::cAE;
      break;

    case CONSTELLATION_SBAS:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_BDS:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      rc = RC_E_NOTOK;
      log_error("%s", prc::message(rc));
      assert(prc::success(rc));
      break;
  }
  return rc;
}

/** Retrieves the value of the second zonal harmonic of the geopotential for
 * the specified GNSS constellation.
 *
 * \param constellation The constellation to return the value for
 * \param j02 Pointer to variable to return the value in
 * \return RC_S_OK if successful. Otherwise value was not set
 */
inline PRC j02(constellation_t constellation, double *j02) {
  assert(j02 != nullptr);

  PRC rc(RC_S_OK);
  switch (constellation) {
    case CONSTELLATION_GPS:
      *j02 = gps::cJ02;
      break;

    case CONSTELLATION_GLO:
      *j02 = glonass::cJ02;
      break;

    case CONSTELLATION_SBAS:
      rc = RC_E_NOT_IMPL;
      log_error("%s", prc::message(rc));
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_BDS:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
    default:
      rc = RC_E_NOTOK;
      log_error("%s", prc::message(rc));
      assert(prc::success(rc));
      break;
  }
  return rc;
}
}  // namespace gnss

// \}
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_GNSS_CONSTANTS_H
