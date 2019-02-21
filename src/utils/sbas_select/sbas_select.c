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

#include "sbas_select.h"

#include <assert.h>
#include <math.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/signal.h>

/* Hysteresis for SBAS coverage area borders to avoid switching SBAS systems
 * back and forth if user's position is closed to a border in degrees */
#define SBAS_SELECT_LATLON_HYST_DEG 1

/* Do not change SBAS provider if user is closer to a pole than this distance.
   50km was chosen somewhat arbitrary.
   */
#define SBAS_SELECT_LAT_POLAR_REGION_KM 50

#define LAT_DEG_PER_KM (360 / (2 * (WGS84_A / 1000.) * M_PI))
#define SBAS_SELECT_LAT_POLAR_REGION_DEG \
  (SBAS_SELECT_LAT_POLAR_REGION_KM * LAT_DEG_PER_KM)

typedef struct {
  s16 lat_deg; /**< Latitude [deg] of SBAS coverage area border */
  s16 lon_deg; /**< Longitude [deg] of SBAS coverage area border */
} point_coord_t;

typedef struct {
  sbas_system_t sbas;     /**< SBAS type */
  point_coord_t *borders; /**< Pointer to array of SBAS coverage area borders */
} sbas_coverage_t;

/* The SBAS rectangular range below must follow these rules:
 * 1. Longitude border can be in the range from -180 deg to 180 deg
 * 2. Eastern  border has greater longitude
 * 3. Latitude border can be in the range from -90 deg to 90 deg
 * 4. Northern border border has greater latitude
 * 5. In calculation border can be extended by SBAS_SELECT_LATLON_HYST_DEG
 *    for hysteresis purposes to avoid jumping between systems
 * 6. Polar regions need special handling, north pole implemented
 */

/** WAAS coverage area based on http://www.nstb.tc.faa.gov/24Hr_WaasLPV.htm,
 *  we use rectangular definition covering the iono correction grid plus
 *  some margin and rounded to closest 10 deg. */
static point_coord_t waas_range[] = {
    {90, -180},
    {0, -50},
};

/** EGNOS coverage area based on
 *  https://egnos-user-support.essp-sas.eu/new_egnos_ops/egnos_system_realtime,
 *  we use rectangular definition covering the iono correction grid plus
 *  some margin and rounded to closest 10 deg. */
static point_coord_t egnos_range[] = {
    {90, -50},
    {0, 40},
};

/** GAGAN coverage area based on
 *  http://www.insidegnss.com/auto/janfeb16-GAGAN.pdf pg.45,
 *  we use rectangular definition covering the iono correction grid plus
 *  some margin and rounded to closest 10 deg. */
static point_coord_t gagan_range[] = {
    {50, 40},
    {-20, 100},
};

/** MSAS coverage area based on
 *  http://www.unoosa.org/pdf/icg/2011/icg-6/1-1.pdf,
 *  we use rectangular definition covering the iono correction grid plus
 *  some margin and rounded to closest 10 deg. */
static point_coord_t msas_range[] = {
    {60, 100},
    {20, 160},
};

static const sbas_coverage_t sbas_coverage[] = {
    {SBAS_WAAS, waas_range},
    {SBAS_EGNOS, egnos_range},
    {SBAS_GAGAN, gagan_range},
    {SBAS_MSAS, msas_range},
};

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
    case SBAS_NONE:
      return "NONE";
    case SBAS_COUNT:
    default:
      break;
  }
  assert(!"Incorrect SBAS type");
  return "NONE";
}

/**
 * Helper function. Finds index of sbas_map array corresponds to SBAS.
 * \param[in] sbas SBAS type
 * \return Index of sbas_map array
 */
static u8 get_sbas_area_index(sbas_system_t sbas) {
  if (SBAS_NONE == sbas) {
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
 * \param lat_deg user's position latitude [-90 deg .. +90 deg]
 * \param lon_deg user's position longitude [-180 deg .. +180 deg]
 * \param hyst_deg hysteresis width to apply to latitude and longitude [deg]
 * \return true if the user's position is in the region, otherwise false
 */
static bool point_in_region(const point_coord_t *border,
                            const double lat_deg,
                            const double lon_deg,
                            const double hyst_deg) {
  /* Check latitude for the normal case */
  double north_deg = border[0].lat_deg + hyst_deg;
  double south_deg = border[1].lat_deg - hyst_deg;
  if ((lat_deg > north_deg) || (lat_deg < south_deg)) {
    return false;
  }
  /* Note: north pole will have special handling outside this function */

  /* Latitude ok, check longitude starting from the normal case */
  double west_deg = border[0].lon_deg - hyst_deg;
  double east_deg = border[1].lon_deg + hyst_deg;
  if ((west_deg <= lon_deg) && (lon_deg <= east_deg)) {
    return true;
  }
  /* Check if lon_deg from [-180 .. (-180 + hyst_deg)] fall into the region */
  if ((west_deg <= (lon_deg + 360)) && ((lon_deg + 360) <= east_deg)) {
    return true;
  }
  /* Check if lon_deg from [(180 - hyst_deg) .. 180] fall into the region */
  if ((west_deg <= (lon_deg - 360)) && ((lon_deg - 360) <= east_deg)) {
    return true;
  }

  /* All longitude checks failed */
  return false;
}

static sbas_system_t update_used_sbas(const sbas_system_t new,
                                      sbas_system_t *current) {
  if (new != *current) {
    log_info("SBAS system changed: %s -> %s",
             get_sbas_name(*current),
             get_sbas_name(new));
    *current = new;
  }
  return *current;
}

/**
 * The function calculates what SBAS system is available
 * depending on user position.
 * \param[in] lgf LGF structure
 * \return SBAS type corresponding to user's position
 */
sbas_system_t sbas_select_provider(const last_good_fix_t *lgf) {
  /* SBAS system currently in use */
  static sbas_system_t used_sbas = SBAS_NONE;

  assert(lgf != NULL);

  if (POSITION_UNKNOWN == lgf->position_quality) {
    return update_used_sbas(SBAS_NONE, &used_sbas);
  }

  double lgf_lat_deg = lgf->position_solution.pos_llh[0] * R2D;
  assert(-90 <= lgf_lat_deg && lgf_lat_deg <= 90);

  /* Design note: code doesn't apply special handling for south pole due
   * none of the systems extending there at the moment. Once support is needed
   * both poles need to be handled separately to cover the (artificial) case
   * where receiver is moved from one pole to another without getting fix in
   * between.
   */
  if (double_within(lgf_lat_deg, 90.0, SBAS_SELECT_LAT_POLAR_REGION_DEG)) {
    /* LGF is close to north pole, where longitudes can change rapidly.
       If first LGF is acquired close to a pole and no SBAS provider is in
       use, we want to start using some SBAS provider and stick to
       it until LGF leaves the #SBAS_SELECT_LAT_POLAR_REGION_DEG radius area
       from the pole. WAAS is a default if no other system has been set yet.
       This also covers the hysteresis for latitude near pole. */

    if (SBAS_NONE == used_sbas) {
      update_used_sbas(SBAS_WAAS, &used_sbas);
    }

    return used_sbas;
  }

  double lgf_lon_deg = lgf->position_solution.pos_llh[1] * R2D;
  assert(-180 <= lgf_lon_deg && lgf_lon_deg <= 180);

  /* Test current selection against user position using hysteresis */
  if (SBAS_NONE != used_sbas) {
    u8 idx = get_sbas_area_index(used_sbas);
    if (point_in_region(sbas_coverage[idx].borders,
                        lgf_lat_deg,
                        lgf_lon_deg,
                        SBAS_SELECT_LATLON_HYST_DEG)) {
      /* SBAS area unchanged */
      return used_sbas;
    }
  }

  for (u8 i = 0; i < ARRAY_SIZE(sbas_coverage); ++i) {
    /* Helper pointer for readability */
    const sbas_coverage_t *it = &sbas_coverage[i];

    /* Test current iteration against user position using zero hysteresis */
    if (point_in_region(it->borders, lgf_lat_deg, lgf_lon_deg, 0)) {
      /* SBAS area found */
      return update_used_sbas(it->sbas, &used_sbas);
    }
  }

  /* User is not in any SBAS coverage area */
  return update_used_sbas(SBAS_NONE, &used_sbas);
}

/**
 * The function returns hardcoded PRN mask depending on SBAS system
 * \param sbas SBAS type
 * \return 32-bit mask, where bit 0 is responsible for PRN 120,
 *         bit 18 - PRN 138, bits 19 through 31 not used
 */
u32 sbas_select_prn_mask(sbas_system_t sbas) {
  u32 mask = 0;
  if (SBAS_NONE == sbas) {
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
