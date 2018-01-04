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

#include <libswiftnav/signal.h>

#include "sbas_select.h"

#include <assert.h>
#include <math.h>

/** PRN mask for WAAS, https://wiki2.org/en/WAAS#Space_segment */
#define SBAS_SELECT_WAAS_MASK                                      \
  ((1 << (122 - SBAS_FIRST_PRN)) | (1 << (131 - SBAS_FIRST_PRN)) | \
   (1 << (133 - SBAS_FIRST_PRN)) | (1 << (134 - SBAS_FIRST_PRN)) | \
   (1 << (135 - SBAS_FIRST_PRN)) | (1 << (138 - SBAS_FIRST_PRN)))

/** PRN mask for EGNOS,
 *  https://egnos-user-support.essp-as.eu/new_egnos_ops/egnos_system_realtime*/
#define SBAS_SELECT_EGNOS_MASK                                     \
  ((1 << (120 - SBAS_FIRST_PRN)) | (1 << (123 - SBAS_FIRST_PRN)) | \
   (1 << (136 - SBAS_FIRST_PRN)))

/** PRN mask for GAGAN,
 *  http://www.insidegnss.com/auto/janfeb16-GAGAN.pdf (pg. 44) */
#define SBAS_SELECT_GAGAN_MASK                                     \
  ((1 << (127 - SBAS_FIRST_PRN)) | (1 << (128 - SBAS_FIRST_PRN)) | \
   (1 << (132 - SBAS_FIRST_PRN)))

/** PRN mask for MSAS,
 *  https://wiki2.org/en/Multi-functional_Satellite_Augmentation_System */
#define SBAS_SELECT_MSAS_MASK \
  ((1 << (129 - SBAS_FIRST_PRN)) | (1 << (137 - SBAS_FIRST_PRN)))

/* Hysteresis for SBAS coverage area borders to avoid switching SBAS systems
 * back and forth if user's position is closed to a border in degrees */
#define SBAS_SELECT_HYST_DEG 1

typedef struct {
  s16 lat_deg; /**< Latitude [deg] of SBAS coverage area border */
  s16 lon_deg; /**< Longitude [deg] of SBAS coverage area border */
} point_coord_t;

typedef struct {
  sbas_type_t sbas;       /**< SBAS type */
  point_coord_t *borders; /**< Pointer to array of SBAS coverage area borders */
} sbas_coverage_t;

/* The SBAS range below must follow these rules:
 * 1. A border can be in the range from -180+SBAS_SELECT_HYST_DEG deg
 *    to 180-SBAS_SELECT_HYST_DEG longitude deg
 * 2. Right border has greater longitude
 */

/** WAAS coverage area based on http://www.nstb.tc.faa.gov/24Hr_WaasLPV.htm,
 *  for the 1st phase of the implementation the area defined by 2 longitudes --
 *  left and right borders. Latitude is not used for now
 *  -180 and 180 deg are exception, so don't use hysteresis for the border,
 *  substract it in advance */
static point_coord_t waas_range[] = {
    {0, -180 + SBAS_SELECT_HYST_DEG}, {0, -50},
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
static sbas_type_t used_sbas = SBAS_UNKNOWN;

/**
 * Helper function. Finds index of sbas_map array corresponds to SBAS.
 * \param[in] sbas SBAS type
 * \return Index of sbas_map array
 */
static u8 get_sbas_area_index(sbas_type_t sbas) {
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
 * The function checks if point is in region.
 * \param[in] border Pointer to border array
 * \param[in] lat  latitude [deg] of point under test
 * \param[in] lon  longitude [deg] of point under test
 * \return true if the point in the region, otherwise false
 */
static bool point_in_region(const point_coord_t *border,
                            const double lat_deg,
                            const double lon_deg) {
  /* for the first phase just check if user's position in between two longitudes
   */
  (void)lat_deg;
  double tmp_lon = lon_deg;
  if (lon_deg > 0.f && fabs(180.f - lon_deg) < 1.f / 3600.f) {
    /* handle an exception: 180 deg is same as -180 deg
     * (actually check that difference between user's longitude and 180 deg is
     * less than 1 sec) */
    tmp_lon = -180.f;
  }
  return ((double)(border[0].lon_deg - SBAS_SELECT_HYST_DEG) <= tmp_lon) &&
         ((double)(border[1].lon_deg + SBAS_SELECT_HYST_DEG) >= tmp_lon);
}

/**
 * The function calculates what SBAS system is available
 * depending on user position.
 * \param[in] lgf LGF structure
 * \return SBAS type corresponding to user's position
 */
sbas_type_t sbas_select_provider(const last_good_fix_t *lgf) {
  assert(lgf != NULL);
  if (lgf->position_quality == POSITION_UNKNOWN) {
    return SBAS_UNKNOWN;
  }
  /* Check all SBAS systems if user position is in its coverage area.
   * Start checking from currently using SBAS system. If user still in it
   * just return, don't check others to avoid unwanted SBAS switch */
  u8 i = get_sbas_area_index(used_sbas);
  /* go through all hardcoded SBAS area */
  u8 j;
  for (j = 0; j < ARRAY_SIZE(sbas_coverage); j++) {
    /* check if user position is in SBAS area under testing */
    if (point_in_region(sbas_coverage[i].borders,
                        lgf->position_solution.pos_llh[0] * R2D,
                        lgf->position_solution.pos_llh[1] * R2D)) {
      if (sbas_coverage[i].sbas != used_sbas) {
        log_info("SBAS system changed: old %" PRId8 ", new %" PRId8,
                 (s8)used_sbas,
                 (s8)sbas_coverage[i].sbas);
        /* update used SBAS info */
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
u32 sbas_select_prn_mask(sbas_type_t sbas) {
  switch ((s8)sbas) {
    case SBAS_UNKNOWN:
      return 0;
    case SBAS_WAAS:
      return SBAS_SELECT_WAAS_MASK;
    case SBAS_EGNOS:
      return SBAS_SELECT_EGNOS_MASK;
    case SBAS_MSAS:
      return SBAS_SELECT_MSAS_MASK;
    case SBAS_GAGAN:
      return SBAS_SELECT_GAGAN_MASK;
    default:
      assert(!"SBAS not supported");
  }
}
