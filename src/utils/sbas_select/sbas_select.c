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

#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/signal.h>

#include "sbas_select.h"

#include <assert.h>
#include <math.h>

/* Hysteresis for SBAS coverage area borders to avoid switching SBAS systems
 * back and forth if user's position is closed to a border in degrees */
#define SBAS_SELECT_LON_HYST_DEG 1

/* Do not change SBAS provider if user is closer to a pole than this distance.
   50km was chosen somewhat arbitrary. It is the distance to horizon if LGF
   is at altitudes above 150m. */
#define SBAS_SELECT_LAT_AT_POLE_HYST_KM 50

#define LAT_DEG_PER_KM (360 / (2 * (WGS84_A / 1000.) * M_PI))
#define SBAS_SELECT_LAT_AT_POLE_HYST_DEG \
  (SBAS_SELECT_LAT_AT_POLE_HYST_KM * LAT_DEG_PER_KM)

typedef struct {
  s16 lat_deg; /**< Latitude [deg] of SBAS coverage area border */
  s16 lon_deg; /**< Longitude [deg] of SBAS coverage area border */
} point_coord_t;

typedef struct {
  sbas_system_t sbas;     /**< SBAS type */
  point_coord_t *borders; /**< Pointer to array of SBAS coverage area borders */
} sbas_coverage_t;

/* The SBAS range below must follow these rules:
 * 1. A border can be in the range from -180+SBAS_SELECT_LON_HYST_DEG deg
 *    to 180-SBAS_SELECT_LON_HYST_DEG longitude deg
 * 2. Right border has greater longitude
 */

/** WAAS coverage area based on http://www.nstb.tc.faa.gov/24Hr_WaasLPV.htm,
 *  for the 1st phase of the implementation the area defined by 2 longitudes --
 *  left and right borders. Latitude is not used for now. */
static point_coord_t waas_range[] = {
    {0, -180}, {0, -50},
};

/** EGNOS coverage area based on
 *  https://egnos-user-support.essp-sas.eu/new_egnos_ops/egnos_system_realtime,
 *  for the 1st phase of the implementation the area defined by 2 longitudes --
 *  left and right borders. Latitude is not used for now */
static point_coord_t egnos_range[] = {
    {0, -50}, {0, 40},
};

/** MSAS coverage area based on
 *  http://www.unoosa.org/pdf/icg/2011/icg-6/1-1.pdf,
 *  for the 1st phase of the implementation the area defined by 2 longitudes --
 *  left and right borders. Latitude is not used for now */
static point_coord_t msas_range[] = {
    {0, 100}, {0, 160},
};

/** GAGAN coverage area based on
 *  http://www.insidegnss.com/auto/janfeb16-GAGAN.pdf pg.45,
 *  for the 1st phase of the implementation the area defined by 2 longitudes --
 *  left and right borders. Latitude is not used for now */
static point_coord_t gagan_range[] = {
    {0, 40}, {0, 100},
};

static const sbas_coverage_t sbas_coverage[] = {
    {SBAS_WAAS, waas_range},
    {SBAS_EGNOS, egnos_range},
    {SBAS_GAGAN, gagan_range},
    {SBAS_MSAS, msas_range},
};

/** SBAS system currently in use */
static sbas_system_t used_sbas = SBAS_UNKNOWN;

/** Convert SBAS system provider ID (type) to a descriptive name */
static const char *get_sbas_name(sbas_system_t sbas_type) {
  switch (sbas_type) {
    case SBAS_WAAS:
      return "WAAS";
    case SBAS_EGNOS:
      return "EGNOS";
    case SBAS_GAGAN:
      return "GAGAN";
    case SBAS_MSAS:
      return "MSAS";
    case SBAS_UNKNOWN:
    case SBAS_COUNT:
    default:
      break;
  }
  return "UNKNOWN";
}

/**
 * Helper function. Finds index of sbas_map array corresponds to SBAS.
 * \param[in] sbas SBAS type
 * \return Index of sbas_map array
 */
static u8 get_sbas_area_index(sbas_system_t sbas) {
  if (SBAS_UNKNOWN == sbas) {
    return 0;
  }
  u8 i;
  for (i = 0; i < ARRAY_SIZE(sbas_coverage); i++) {
    if (sbas_coverage[i].sbas == sbas) {
      return i;
    }
  }
  assert(!"Incorrect SBAS map or SBAS type");
  return 0;
}

/**
 * Checks if user's longitude is between two longitudes of border parameter.
 * \param[in] border Pointer to border array
 * \param lon_deg user's position longitude [-180 deg .. +180 deg]
 * \param hyst_deg longitude hysteresis width to apply [deg]
 * \return true if the user's position is in the region, otherwise false
 */
static bool point_in_region(const point_coord_t *border,
                            const double lon_deg,
                            const double hyst_deg) {
  double west_deg = border[0].lon_deg - hyst_deg;
  double east_deg = border[1].lon_deg + hyst_deg;
  if ((west_deg <= lon_deg) && (lon_deg <= east_deg)) {
    return true;
  }
  /* check if lon_deg from [-180 .. (-180 + hyst_deg)] fall into the region */
  if ((west_deg <= (lon_deg + 360)) && ((lon_deg + 360) <= east_deg)) {
    return true;
  }
  /* check if lon_deg from [(180 - hyst_deg) .. 180] fall into the region */
  return ((west_deg <= (lon_deg - 360)) && ((lon_deg - 360) <= east_deg));
}

/**
 * The function calculates what SBAS system is available
 * depending on user position.
 * \param[in] lgf LGF structure
 * \return SBAS type corresponding to user's position
 */
sbas_system_t sbas_select_provider(const last_good_fix_t *lgf) {
  assert(lgf != NULL);
  if (lgf->position_quality == POSITION_UNKNOWN) {
    return SBAS_UNKNOWN;
  }

  double lgf_lat_deg = lgf->position_solution.pos_llh[0] * R2D;
  if (double_within(fabs(lgf_lat_deg), 90., SBAS_SELECT_LAT_AT_POLE_HYST_DEG)) {
    /* LGF is close to a pole, where longitudes can change rapidly.
       If first LGF is acquired close to a pole, then no SBAS provider is in use
       In this case we want to start using some SBAS provider and
       stick to it until LGF leaves the #SBAS_SELECT_LAT_AT_POLE_HYST_DEG radius
       area from the pole. */
    if (used_sbas != SBAS_UNKNOWN) {
      return used_sbas;
    }
  }
  double lgf_lon_deg = lgf->position_solution.pos_llh[1] * R2D;
  assert(-180 <= lgf_lon_deg);
  assert(lgf_lon_deg <= 180);
  /* Check all SBAS systems if user position is in its coverage area.
   * Start checking from currently using SBAS system. If user still in it
   * just return, don't check others to avoid unwanted SBAS switch */
  u8 i = get_sbas_area_index(used_sbas);
  for (u8 j = 0; j < ARRAY_SIZE(sbas_coverage); j++) {
    double hyst_deg =
        (used_sbas == sbas_coverage[i].sbas) ? SBAS_SELECT_LON_HYST_DEG : 0;
    /* check if user position is in SBAS area under testing */
    if (point_in_region(sbas_coverage[i].borders, lgf_lon_deg, hyst_deg)) {
      if (sbas_coverage[i].sbas != used_sbas) {
        log_info("SBAS system changed: %s -> %s",
                 get_sbas_name(used_sbas),
                 get_sbas_name(sbas_coverage[i].sbas));
        used_sbas = sbas_coverage[i].sbas;
      }
      /* SBAS area found */
      return used_sbas;
    }
    /* if user is not in area in question, take next one */
    i++;
    if (i > ARRAY_SIZE(sbas_coverage) - 1) {
      /* we reached max index of the array, get 0th */
      i = 0;
    }
  }
  /* User is not in any SBAS coverage area */
  used_sbas = SBAS_UNKNOWN;
  return SBAS_UNKNOWN;
}

/**
 * The function returns hardcoded PRN mask depending on SBAS system
 * \param sbas SBAS type
 * \return 32-bit mask, where bit 0 is responsible for PRN 120,
 *         bit 18 - PRN 138, bits 19 through 31 not used
 */
u32 sbas_select_prn_mask(sbas_system_t sbas) {
  u32 mask = 0;
  if (SBAS_UNKNOWN == sbas) {
    return mask;
  }
  const u8 *prn_list = get_sbas_prn_list(sbas);
  for (u8 i = 0; i < MAX_SBAS_SATS_PER_SYSTEM; i++) {
    if (prn_list[i] >= SBAS_FIRST_PRN) {
      mask |= 1 << (prn_list[i] - SBAS_FIRST_PRN);
    }
  }
  return mask;
}
