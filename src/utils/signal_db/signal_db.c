/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "signal_db.h"

#include <assert.h>
#include <board.h>
#include <string.h>
#include <swiftnav/constants.h>

/** GPS L1 C/A carrier freq / code chipping rate
 * \note this is GPS_L1_HZ / GPS_CA_CHIPPING_RATE */
#define GPS_L1CA_CARR_TO_CODE (GPS_L1_HZ / GPS_CA_CHIPPING_RATE)

/** GPS L2C carrier freq / code chipping rate
 * \note this is GPS_L2_HZ / GPS_CA_CHIPPING_RATE */
#define GPS_L2C_CARR_TO_CODE (GPS_L2_HZ / GPS_CA_CHIPPING_RATE)

/** GPS L5 carrier freq / code chipping rate
 * \note this is GPS_L5_HZ / GPS_L5_CHIPPING_RATE */
#define GPS_L5_CARR_TO_CODE (GPS_L5_HZ / GPS_L5_CHIPPING_RATE)
/** GLO L1 carrier freq / code chipping rate
 * \note this is GLO_L1_HZ / GLO_CHIPPING_RATE */
#define GLO_L1_CARR_TO_CODE(fcn) \
  ((GLO_L1_HZ + (fcn)*GLO_L1_DELTA_HZ) / GLO_CA_CHIPPING_RATE)

/** GLO L2 carrier freq / code chipping rate
 * \note this is GLO_L2_HZ / GLO_CHIPPING_RATE */
#define GLO_L2_CARR_TO_CODE(fcn) \
  ((GLO_L2_HZ + (fcn)*GLO_L2_DELTA_HZ) / GLO_CA_CHIPPING_RATE)

/** SBAS L1 carrier to code ratio */
#define SBAS_L1CA_CARR_TO_CODE (SBAS_L1_HZ / SBAS_L1CA_CHIPPING_RATE)

/** SBAS L5 carrier to code ratio */
#define SBAS_L5_CARR_TO_CODE (SBAS_L5_HZ / SBAS_L5_CHIPPING_RATE)

/** Beidou2 B11 carrier to code ratio */
#define BDS2_B11_CARR_TO_CODE (BDS2_B11_HZ / BDS2_B11_CHIPPING_RATE)

/** Beidou2 B2 carrier to code ratio */
#define BDS2_B2_CARR_TO_CODE (BDS2_B2_HZ / BDS2_B2_CHIPPING_RATE)

/** Beidou3 B1C carrier to code ratio */
#define BDS3_B1C_CARR_TO_CODE (BDS3_B1C_HZ / BDS3_B1C_CHIPPING_RATE)

/** Beidou3 B3 carrier to code ratio */
#define BDS3_B3_CARR_TO_CODE (BDS3_B3_HZ / BDS3_B3_CHIPPING_RATE)

/** Beidou3 B2b carrier to code ratio */
#define BDS3_B7_CARR_TO_CODE (BDS3_B7_HZ / BDS3_B7_CHIPPING_RATE)

/** Beidou3 B2a carrier to code ratio */
#define BDS3_B5_CARR_TO_CODE (BDS3_B5_HZ / BDS3_B5_CHIPPING_RATE)

/** Galileo E1 carrier to code ratio */
#define GAL_E1_CARR_TO_CODE (GAL_E1_HZ / GAL_E1_CHIPPING_RATE)

/** Galileo E6 carrier to code ratio */
#define GAL_E6_CARR_TO_CODE (GAL_E6_HZ / GAL_E6_CHIPPING_RATE)

/** Galileo E5b carrier to code ratio */
#define GAL_E7_CARR_TO_CODE (GAL_E7_HZ / GAL_E7_CHIPPING_RATE)

/** Galileo E5a carrier to code ratio */
#define GAL_E5_CARR_TO_CODE (GAL_E5_HZ / GAL_E5_CHIPPING_RATE)

/** QZSS L1C/A carrier to code ratio */
#define QZS_L1CA_CARR_TO_CODE (QZS_L1_HZ / QZS_L1CA_CHIPPING_RATE)

/** QZSS L2C carrier to code ratio */
#define QZS_L2C_CARR_TO_CODE (QZS_L2_HZ / QZS_L1CA_CHIPPING_RATE)

/** \defgroup signal GNSS signal identifiers (SID)
 * \{ */

/** Table of global and constellation start indexes, monotonically increasing,
 * indexed by code. */
typedef struct {
  u16 constellation_start_index;
  u16 me_constellation_start_index;
  u16 global_start_index;
  u16 me_global_start_index;
  double carr_to_code;
} code_db_element_t;
static code_db_element_t code_db[CODE_COUNT] = {
    /** GPS */
    [CODE_GPS_L1CA] = {.carr_to_code = GPS_L1CA_CARR_TO_CODE},
    [CODE_GPS_L2CM] = {.carr_to_code = GPS_L2C_CARR_TO_CODE},
    [CODE_GPS_L2CL] = {.carr_to_code = GPS_L2C_CARR_TO_CODE},

    /** SBAS */
    [CODE_SBAS_L1CA] = {.carr_to_code = SBAS_L1CA_CARR_TO_CODE},

    /** Galileo  */
    [CODE_GAL_E1B] = {.carr_to_code = GAL_E1_CARR_TO_CODE},
    [CODE_GAL_E7I] = {.carr_to_code = GAL_E7_CARR_TO_CODE},
    [CODE_GAL_E5I] = {.carr_to_code = GAL_E5_CARR_TO_CODE},

    /** Beidou */
    [CODE_BDS2_B1] = {.carr_to_code = BDS2_B11_CARR_TO_CODE},
    [CODE_BDS2_B2] = {.carr_to_code = BDS2_B2_CARR_TO_CODE},

    /** QZS L1C/A has all the same characteristics as GPS L1 C/A */
    [CODE_QZS_L1CA] = {.carr_to_code = QZS_L1CA_CARR_TO_CODE},
    [CODE_AUX_QZS] = {.carr_to_code = QZS_L1CA_CARR_TO_CODE},
    [CODE_QZS_L2CM] = {.carr_to_code = QZS_L2C_CARR_TO_CODE},
    [CODE_QZS_L2CL] = {.carr_to_code = QZS_L2C_CARR_TO_CODE}};

/** Table of sv constellation indexes. */
typedef struct {
  u16 sat_start;
  u16 start_index;
} constellation_table_element_t;
static constellation_table_element_t constellation_table[CONSTELLATION_COUNT] =
    {[CONSTELLATION_GPS] = {GPS_FIRST_PRN, 0},
     [CONSTELLATION_SBAS] = {SBAS_FIRST_PRN, NUM_SATS_GPS},
     [CONSTELLATION_GLO] = {GLO_FIRST_PRN, NUM_SATS_GPS + NUM_SATS_SBAS},
     [CONSTELLATION_BDS] = {BDS_FIRST_PRN,
                            NUM_SATS_GPS + NUM_SATS_SBAS + NUM_SATS_GLO},
     [CONSTELLATION_QZS] = {QZS_FIRST_PRN,
                            NUM_SATS_GPS + NUM_SATS_SBAS + NUM_SATS_GLO +
                                NUM_SATS_BDS},
     [CONSTELLATION_GAL] = {GAL_FIRST_PRN,
                            NUM_SATS_GPS + NUM_SATS_SBAS + NUM_SATS_GLO +
                                NUM_SATS_BDS + NUM_SATS_QZS}};

/** Number of signals for each code which are supported on
 * the current hardware platform. */
static const u16 code_signal_counts[CODE_COUNT] = {
    [CODE_GPS_L1CA] = PLATFORM_SIGNAL_COUNT_GPS_L1CA,
    [CODE_GPS_L2CM] = PLATFORM_SIGNAL_COUNT_GPS_L2C,
    [CODE_GPS_L5X] = PLATFORM_SIGNAL_COUNT_GPS_L5,
    [CODE_GPS_L1P] = PLATFORM_SIGNAL_COUNT_GPS_L1P,
    [CODE_GPS_L2P] = PLATFORM_SIGNAL_COUNT_GPS_L2P,
    [CODE_SBAS_L1CA] = PLATFORM_SIGNAL_COUNT_SBAS_L1CA,
    [CODE_GLO_L1OF] = PLATFORM_SIGNAL_COUNT_GLO_L1OF,
    [CODE_GLO_L2OF] = PLATFORM_SIGNAL_COUNT_GLO_L2OF,
    [CODE_BDS2_B1] = PLATFORM_SIGNAL_COUNT_BDS2_B1,
    [CODE_BDS2_B2] = PLATFORM_SIGNAL_COUNT_BDS2_B2,
    [CODE_QZS_L1CA] = PLATFORM_SIGNAL_COUNT_QZS_L1CA,
    [CODE_QZS_L2CM] = PLATFORM_SIGNAL_COUNT_QZS_L2C,
    [CODE_QZS_L5X] = PLATFORM_SIGNAL_COUNT_QZS_L5,
    [CODE_GAL_E1B] = PLATFORM_SIGNAL_COUNT_GAL_E1,
    [CODE_GAL_E5I] = PLATFORM_SIGNAL_COUNT_GAL_E5,
    [CODE_GAL_E7I] = PLATFORM_SIGNAL_COUNT_GAL_E7};

/** Number of ME signals for each code which are supported on
 * the current hardware platform. */
static const u16 me_code_signal_counts[CODE_COUNT] = {
    [CODE_GPS_L1CA] = PLATFORM_SIGNAL_COUNT_GPS_L1CA,
    [CODE_GPS_L2CM] = PLATFORM_SIGNAL_COUNT_GPS_L2C,
    [CODE_GPS_L5X] = PLATFORM_SIGNAL_COUNT_GPS_L5,
    [CODE_GPS_L1P] = PLATFORM_SIGNAL_COUNT_GPS_L1P,
    [CODE_GPS_L2P] = PLATFORM_SIGNAL_COUNT_GPS_L2P,
    [CODE_SBAS_L1CA] = PLATFORM_SIGNAL_COUNT_SBAS_L1CA,
    [CODE_GLO_L1OF] = PLATFORM_FREQ_COUNT_GLO_L1OF,
    [CODE_GLO_L2OF] = PLATFORM_FREQ_COUNT_GLO_L2OF,
    [CODE_BDS2_B1] = PLATFORM_SIGNAL_COUNT_BDS2_B1,
    [CODE_BDS2_B2] = PLATFORM_SIGNAL_COUNT_BDS2_B2,
    [CODE_QZS_L1CA] = PLATFORM_SIGNAL_COUNT_QZS_L1CA,
    [CODE_QZS_L2CM] = PLATFORM_SIGNAL_COUNT_QZS_L2C,
    [CODE_QZS_L5X] = PLATFORM_SIGNAL_COUNT_QZS_L5,
    [CODE_GAL_E1B] = PLATFORM_SIGNAL_COUNT_GAL_E1,
    [CODE_GAL_E5I] = PLATFORM_SIGNAL_COUNT_GAL_E5,
    [CODE_GAL_E7Q] = PLATFORM_SIGNAL_COUNT_GAL_E7};

/** Initialize the signal module. */
void signal_db_init(void) {
  /* Populate constellation start index */
  u16 constellation_start_indexes[CONSTELLATION_COUNT];
  u16 me_constellation_start_indexes[CONSTELLATION_COUNT];
  memset(constellation_start_indexes, 0, sizeof(constellation_start_indexes));
  memset(me_constellation_start_indexes,
         0,
         sizeof(me_constellation_start_indexes));

  for (code_t code = 0; code < CODE_COUNT; code++) {
    constellation_t constellation = code_to_constellation(code);
    code_db[code].constellation_start_index =
        constellation_start_indexes[constellation];
    code_db[code].me_constellation_start_index =
        me_constellation_start_indexes[constellation];

    constellation_start_indexes[constellation] += code_signal_counts[code];
    me_constellation_start_indexes[constellation] +=
        me_code_signal_counts[code];
  }

  /* Populate global start index */
  u16 global_start_index = 0;
  u16 me_global_start_index = 0;

  for (code_t code = 0; code < CODE_COUNT; code++) {
    code_db[code].global_start_index = global_start_index;
    code_db[code].me_global_start_index = me_global_start_index;

    global_start_index += code_signal_counts[code];
    me_global_start_index += me_code_signal_counts[code];
  }
}

/** Convert a global signal index to a gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param global_index    Global signal index in [0, PLATFORM_SIGNAL_COUNT).
 *
 * \return gnss_signal_t corresponding to global_index.
 */
gnss_signal_t sid_from_global_index(u16 global_index) {
  for (code_t code = 0; code < CODE_COUNT; code++) {
    if (global_index <
        code_db[code].global_start_index + code_signal_counts[code]) {
      return sid_from_code_index(
          code, global_index - code_db[code].global_start_index);
    }
  }
  log_error("global_index %d is outside range", global_index);
  assert(!"Invalid global index");
  return construct_sid(CODE_INVALID, 0);
}

/** Convert a code-specific ME signal index to a me_gnss_signal_t.
 *
 * \param code          Code to use.
 * \param me_code_index ME code-specific signal index in
 *                      [0, ACQ_TRACK_COUNT_\<code\>).
 *
 * \return me_gnss_signal_t corresponding to code and code_index.
 */
me_gnss_signal_t mesid_from_code_index(code_t code, u16 me_code_index) {
  assert(code_valid(code));
  assert(me_code_index < code_to_sig_count(code));

  constellation_t cons = code_to_constellation(code);
  u16 sat = constellation_table[cons].sat_start + me_code_index;
  return construct_mesid(code, sat);
}

/** Convert a global ME signal index to a me_gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param me_global_index    Global ME signal index in [0,
 * PLATFORM_ACQ_TRACK_COUNT).
 *
 * \return me_gnss_signal_t corresponding to me_global_index.
 */
me_gnss_signal_t mesid_from_global_index(u16 me_global_index) {
  for (code_t code = 0; code < CODE_COUNT; code++) {
    if (me_global_index <
        code_db[code].me_global_start_index + me_code_signal_counts[code]) {
      return mesid_from_code_index(
          code, me_global_index - code_db[code].me_global_start_index);
    }
  }

  assert(!"Invalid global index");
  return construct_mesid(CODE_INVALID, 0);
}

/** Convert a constellation-specific signal index to a gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param constellation         Constellation to use.
 * \param constellation_index   Constellation-specific signal index in
 *                              [0, PLATFORM_SIGNAL_COUNT_\<constellation\>).
 *
 * \return gnss_signal_t corresponding to constellation and constellation_index.
 */
gnss_signal_t sid_from_constellation_index(constellation_t constellation,
                                           u16 constellation_index) {
  for (code_t code = 0; code < CODE_COUNT; code++) {
    if (code_to_constellation(code) == constellation) {
      if (constellation_index <
          code_db[code].constellation_start_index + code_signal_counts[code]) {
        return sid_from_code_index(
            code,
            constellation_index - code_db[code].constellation_start_index);
      }
    }
  }

  assert(!"Invalid constellation index");
  return construct_sid(CODE_INVALID, 0);
}

/** Return the global ME signal index for a me_gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param mesid   me_gnss_signal_t to use.
 *
 * \return Global ME signal index in [0, PLATFORM_ACQ_TRACK_COUNT).
 */
u16 mesid_to_global_index(const me_gnss_signal_t mesid) {
  assert(code_supported(mesid.code));
  return code_db[mesid.code].me_global_start_index + mesid_to_code_index(mesid);
}

/** Return the constellation-specific signal index for a gnss_signal_t.
 *
 * \note This function only accounts for codes supported on the current
 *       hardware platform.
 *
 * \param sid   gnss_signal_t to use.
 *
 * \return Constellation-specific signal index in
 *         [0, PLATFORM_SIGNAL_COUNT_\<constellation\>).
 */
u16 sid_to_constellation_index(gnss_signal_t sid) {
  assert(code_supported(sid.code));
  return code_db[sid.code].constellation_start_index + sid_to_code_index(sid);
}

/** Determine if a gnss_signal_t is valid and supported on the current
 * hardware platform.
 *
 * \param sid   gnss_signal_t to use.
 *
 * \return true if sid is valid and supported, false otherwise.
 */
bool sid_supported(gnss_signal_t sid) {
  /* Verify general validity */
  if (!sid_valid(sid)) return false;

  /* Verify that the code is supported on this platform */
  if (!code_supported(sid.code)) return false;

  return true;
}

/** Determine if a code is valid and supported on the current
 * hardware platform.
 *
 * \param code  Code to use.
 *
 * \return true if code is valid and supported, false otherwise.
 */
bool code_supported(code_t code) {
  /* Verify general validity */
  if (!code_valid(code)) return false;

  /* Verify that the code is supported on this platform */
  if (code_signal_counts[code] == 0) return false;

  return true;
}

/** Return the minimum Doppler value for a code induced by TCXO error.
 *
 * \param code The code to use.
 * \return Minimum Doppler value [Hz]
 */
float code_to_tcxo_doppler_min(code_t code) {
  assert(code_valid(code));

  float doppler;

  /* use worst case Doppler */
  doppler = -TCXO_FREQ_OFFSET_MAX_PPM * GLO_L1_TCXO_PPM_TO_HZ;

  return doppler;
}

/** Return the maximum Doppler value for a code induced by TCXO error.
 *
 * \param code The code to use.
 * \return Maximum Doppler value [Hz]
 */
float code_to_tcxo_doppler_max(code_t code) {
  assert(code_valid(code));

  float doppler;

  /* use worst case Doppler */
  doppler = TCXO_FREQ_OFFSET_MAX_PPM * GLO_L1_TCXO_PPM_TO_HZ;

  return doppler;
}

/** Convert a SV signal index to a gnss_signal_t.
 *
 * \param sv_index      SV signal index in [0, NUM_SATS)
 *
 * \return gnss_signal_t corresponding to sv_index with first valid code.
 */
gnss_signal_t sv_index_to_sid(u16 sv_index) {
  assert(sv_index < NUM_SATS);
  assert(constellation_table[0].start_index == 0);

  gnss_signal_t sid = {0, CODE_INVALID};

  /* find the constellation for this sv index */
  constellation_t cons = CONSTELLATION_COUNT - 1;
  while (constellation_table[cons].start_index > sv_index) {
    cons--;
  }
  assert(constellation_valid(cons));

  /* find the first valid and supported code for this constellation */
  for (code_t code = 0; code < CODE_COUNT; code++) {
    if (code_supported(code) && code_to_constellation(code) == cons) {
      sid = construct_sid(code,
                          sv_index - constellation_table[cons].start_index +
                              constellation_table[cons].sat_start);
      assert(sid_valid(sid));
      return sid;
    }
  }
  log_debug("Could not generate SID for constellation %d, sv index %u",
            cons,
            sv_index);
  return sid;
}

/** Return the SV index for a gnss_signal_t.
 *
 * \param sid   gnss_signal_t to use.
 *
 * \return SV index in [0, NUM_SATS).
 */
u16 sid_to_sv_index(gnss_signal_t sid) {
  assert(sid_valid(sid));
  constellation_t cons = sid_to_constellation(sid);
  u16 sv_index = constellation_table[cons].start_index + sid.sat -
                 constellation_table[cons].sat_start;
  assert(sv_index < NUM_SATS);
  return sv_index;
}

/** Return carrier frequency channel for a me_gnss_signal_t
 * \param mesid me_gnss_signal_t to use
 * \return The frequency channel [Hz]
 */
double mesid_to_carr_fcn_hz(const me_gnss_signal_t mesid) {
  assert(mesid_valid(mesid));

  double carr_fcn_hz = 0;
  if (CODE_GLO_L1OF == mesid.code) {
    carr_fcn_hz = (mesid.sat - GLO_FCN_OFFSET) * GLO_L1_DELTA_HZ;
  } else if (CODE_GLO_L2OF == mesid.code) {
    carr_fcn_hz = (mesid.sat - GLO_FCN_OFFSET) * GLO_L2_DELTA_HZ;
  }
  return carr_fcn_hz;
}

/** Get the constellation to which a me_gnss_signal_t belongs.
 *
 * \param mesid   me_gnss_signal_t to use.
 *
 * \return Constellation to which mesid belongs.
 */
constellation_t mesid_to_constellation(const me_gnss_signal_t mesid) {
  return code_to_constellation(mesid.code);
}

/** ME signal comparison function. */
int mesid_compare(const me_gnss_signal_t a, const me_gnss_signal_t b) {
  /* Signal code are not sorted in order per constellation
   * (e.g. GLO L1C ~ 3 and GPS L2P ~ 6).
   * As some of our functions relies on comparing ordered sets of signals,
   * this can cause issues.
   * Therefore, in this function, we enforce the ordering per
   * constellation/frequency/satellite */
  if ((code_valid(a.code)) && code_valid(b.code)) {
    if (mesid_to_constellation(a) == mesid_to_constellation(b)) {
      if (code_equiv(a.code, b.code)) {
        return a.sat - b.sat;
      }
      return a.code - b.code;
    }
    return mesid_to_constellation(a) - mesid_to_constellation(b);
  }
  return a.code - b.code;
}

bool mesid_is_equal(const me_gnss_signal_t a, const me_gnss_signal_t b) {
  return mesid_compare(a, b) == 0;
}

/** Construct a me_gnss_signal_t.
 *
 * \note This function does not check the validity of the resulting signal.
 *
 * \param code  Code to use.
 * \param sat   Satellite identifier to use.
 *
 * \return me_gnss_signal_t corresponding to the specified arguments.
 */
me_gnss_signal_t construct_mesid(code_t code, u16 sat) {
  me_gnss_signal_t mesid = {.code = code, .sat = sat};
  return mesid;
}

/** Construct a gnss_signal_t from input me_gnss_signal_t.
 *
 * \param mesid        ME signal to use.
 * \param glo_slot_id  GLO orbital slot.
 *
 * \return gnss_signal_t corresponding to the specified argument.
 */
gnss_signal_t mesid2sid(const me_gnss_signal_t mesid, u16 glo_slot_id) {
  assert(mesid_valid(mesid));
  gnss_signal_t sid;
  if (IS_GLO(mesid)) {
    assert(glo_slot_id_is_valid(glo_slot_id));
    sid = construct_sid(mesid.code, glo_slot_id);
  } else {
    sid = construct_sid(mesid.code, mesid.sat);
  }
  return sid;
}

/** Print a string representation of a me_gnss_signal_t.
 *
 * \param s     Buffer of capacity n to which the string will be written.
 * \param n     Capacity of buffer s.
 * \param mesid me_gnss_signal_t to use.
 *
 * \return Number of characters written to s, excluding the terminating null.
 */
int mesid_to_string(char *s, int n, const me_gnss_signal_t mesid) {
  assert(n >= MESID_STR_LEN_MAX);
  return sat_code_to_string(
      s, MESID_SUFFIX_LENGTH, /* suffix = */ " ME ", mesid.sat, mesid.code);
}

/** Determine if a me_gnss_signal_t corresponds to a known code and
 * ME satellite identifier.
 *
 * \param mesid   me_gnss_signal_t to use.
 *
 * \return true if mesid exists, false otherwise.
 */
bool mesid_valid(const me_gnss_signal_t mesid) {
  if (!code_valid(mesid.code)) {
    return false;
  }

  u16 me_sig_count = code_to_sig_count(mesid.code);
  constellation_t cons = code_to_constellation(mesid.code);
  u16 sat_start = constellation_table[cons].sat_start;
  if ((mesid.sat < sat_start) || (mesid.sat >= sat_start + me_sig_count)) {
    log_debug_mesid(mesid,
                    "mesid.sat %u sat_start %u me_sig_count %u",
                    mesid.sat,
                    sat_start,
                    me_sig_count);
    return false;
  }

  return true;
}

/** Return the code-specific signal index for a me_gnss_signal_t.
 *
 * \param mesid me_gnss_signal_t to use.
 *
 * \return Code-specific signal index in [0, SIGNAL_COUNT_\<code\>).
 */
u16 mesid_to_code_index(const me_gnss_signal_t mesid) {
  assert(mesid_valid(mesid));
  constellation_t cons = code_to_constellation(mesid.code);
  u16 sat_start = constellation_table[cons].sat_start;
  return mesid.sat - sat_start;
}

/** Return the carrier frequency for a mesid.
 *
 * \param mesid  me_gnss_signal_t to use.
 * \return carrier frequency
 */
double mesid_to_carr_freq(const me_gnss_signal_t mesid) {
  code_t code = mesid.code;
  assert(code_valid(code));
  /* Map GLO mesid.sat [1 - 14] -> GLO FCN [-7 - +6] */
  s8 fcn = mesid.sat - GLO_FCN_OFFSET;
  if (CODE_GLO_L1OF == code) {
    return GLO_L1_HZ + fcn * GLO_L1_DELTA_HZ;
  }
  if (CODE_GLO_L2OF == code) {
    return GLO_L2_HZ + fcn * GLO_L2_DELTA_HZ;
  }
  /* there is no difference between mesid and sid for GPS */
  gnss_signal_t sid = construct_sid(mesid.code, mesid.sat);
  return sid_to_carr_freq(sid);
}

/** Return the [carrier freq / code chip rate] for a mesid.
 *
 * \param mesid  me_gnss_signal_t to use.
 * \return [carrier freq / code chip rate]
 */
double mesid_to_carr_to_code(const me_gnss_signal_t mesid) {
  code_t code = mesid.code;
  assert(code_valid(code));
  /* Map GLO mesid.sat [1 - 14] -> GLO FCN [-7 - +6] */
  s8 fcn = mesid.sat - GLO_FCN_OFFSET;
  if (CODE_GLO_L1OF == code) {
    return GLO_L1_CARR_TO_CODE(fcn);
  }
  if (CODE_GLO_L2OF == code) {
    return GLO_L2_CARR_TO_CODE(fcn);
  }
  double carr_to_code = code_db[mesid.code].carr_to_code;
  assert(carr_to_code > 0);
  return carr_to_code;
}

/* \} */
