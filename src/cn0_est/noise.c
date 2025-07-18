/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <ch.h>
#include <math.h>
#include <swiftnav/signal.h>

#include "acq/manage.h"
#include "board/nap/nap_common.h"
#include "main/main.h"
#include "signal_db/signal_db.h"

#define NOISE_ALPHA 0.005f
#define NOISE_MAX_AGE_MS 2000 /* re-estimate the noise with this rate */
#define NOISE_DEFAULT 20000.0f

static MUTEX_DECL(cn0_mutex);

static u32 mask_all[CODE_COUNT] = {0};
static u32 mask_noise[CODE_COUNT] = {0};
static bool has_noise_estimate[CODE_COUNT] = {0};
static float noise[CODE_COUNT] = {0};

static void lock(void) { chMtxLock(&cn0_mutex); }
static void unlock(void) { chMtxUnlock(&cn0_mutex); }

static int find_nontracked_sat(code_t code, u32 msk) {
  unsigned sig_count = code_to_sig_count(code);

  for (unsigned i = 0; i < sig_count; i++) {
    if (0 == (msk & (1u << i))) {
      return i + code_to_sat_start(code);
    }
  }
  return -1;
}

static void start_noise_estimation(void) {
  static const code_t used_codes[] = {CODE_GPS_L1CA,
                                      CODE_GPS_L2CM,
                                      CODE_GPS_L5I,
                                      CODE_GLO_L1OF,
                                      CODE_GLO_L2OF,
                                      CODE_GAL_E1B,
                                      CODE_GAL_E7I,
                                      CODE_GAL_E5I,
                                      CODE_BDS2_B1,
                                      CODE_BDS2_B2,
                                      CODE_BDS3_B5I};
  for (int i = 0; i < (int)ARRAY_SIZE(used_codes); i++) {
    code_t code = used_codes[i];
    if (!code_supported(code)) {
      continue;
    }
    lock();
    u32 msk_all = mask_all[code];
    u32 msk_noise = mask_noise[code];
    unlock();

    if (msk_noise) {
      continue; /* there is still noise tracker running */
    }

    int sat = find_nontracked_sat(code, msk_all);
    if (sat < 0) {
      continue; /* no sats to track */
    }

    u32 bit = 1u << (sat - code_to_sat_start(code));
    lock();
    mask_noise[code] |= bit;
    unlock();

    tracking_startup_params_t startup_params = {
        .mesid = construct_mesid(code, (u16)sat),
        .glo_slot_id = GLO_ORBIT_SLOT_UNKNOWN,
        .sample_count = nap_timing_count(),
        .doppler_hz = 1567.f, /* an arbitrary prime number */
        .code_phase = 131.f,  /* an arbitrary prime number */
        .chips_to_correlate = code_to_chip_count(code),
        .cn0_init = -1.f}; /* this is for noise estimation only */

    tracking_startup_request(&startup_params);
  }
}

void noise_calc(code_t code, u8 cn0_ms, s32 I, s32 Q) {
  assert(code_valid(code));
  float n = ((float)I * I + (float)Q * Q) / (float)cn0_ms;
  if (has_noise_estimate[code]) {
    noise[code] += (n - noise[code]) * NOISE_ALPHA;
  } else {
    noise[code] = n;
    has_noise_estimate[code] = true;
  }
}

float noise_get_estimation(code_t code) {
  assert(code_valid(code));

  DO_EACH_MS(NOISE_MAX_AGE_MS, start_noise_estimation(););

  float n = NOISE_DEFAULT;
  if (has_noise_estimate[code]) {
    n = noise[code];
  }

  if (CODE_SBAS_L1CA == code) {
    return noise_get_estimation(CODE_GPS_L1CA);
  }
  return n;
}

void noise_update_mesid_status(me_gnss_signal_t mesid, bool intrack) {
  assert(mesid_valid(mesid));

  u32 bit = 1u << (mesid.sat - code_to_sat_start(mesid.code));

  lock();
  if (intrack) {
    mask_all[mesid.code] |= bit;
  } else {
    mask_all[mesid.code] &= ~bit;
    mask_noise[mesid.code] &= ~bit;
  }
  unlock();
}
