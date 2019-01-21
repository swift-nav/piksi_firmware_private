#include <assert.h>
#include <ch.h>
#include <math.h>
#include <swiftnav/signal.h>
#include "acq/manage.h"
#include "signal_db/signal_db.h"

#define NOISE_ALPHA 0.001
#define NOISE_MAX_AGE_MS 5000 /* re-estimate the noise with this rate */

static MUTEX_DECL(cn0_mutex);

static int init_done = 0;

static u64 ms[CODE_COUNT] = {[CODE_GPS_L1CA] = NOISE_MAX_AGE_MS,
                             [CODE_GPS_L2CM] = NOISE_MAX_AGE_MS,

                             [CODE_GLO_L1OF] = NOISE_MAX_AGE_MS,
                             [CODE_GLO_L2OF] = NOISE_MAX_AGE_MS,

                             [CODE_GAL_E1B] = NOISE_MAX_AGE_MS,
                             [CODE_GAL_E7I] = NOISE_MAX_AGE_MS,

                             [CODE_BDS2_B1] = NOISE_MAX_AGE_MS,
                             [CODE_BDS2_B2] = NOISE_MAX_AGE_MS,

                             [CODE_SBAS_L1CA] = NOISE_MAX_AGE_MS};
static u32 mask[CODE_COUNT] = {0};
static int has_estimated_noise[CODE_COUNT] = {0};
static double noise[CODE_COUNT] = {[CODE_GPS_L1CA] = 5.,
                                   [CODE_GPS_L2CM] = 5.,

                                   [CODE_GLO_L1OF] = 5.,
                                   [CODE_GLO_L2OF] = 5.,

                                   [CODE_GAL_E1B] = 5.,
                                   [CODE_GAL_E7I] = 1.,

                                   [CODE_BDS2_B1] = 5.,
                                   [CODE_BDS2_B2] = 5.,

                                   [CODE_SBAS_L1CA] = 5.};
static double scale_factor[CODE_COUNT] = {[CODE_GPS_L1CA] = 5.3,
                                          [CODE_GPS_L2CM] = 6.5,

                                          [CODE_GLO_L1OF] = 5.7,
                                          [CODE_GLO_L2OF] = 6.7,

                                          [CODE_GAL_E1B] = 6.6,
                                          [CODE_GAL_E7I] = 7.8,

                                          [CODE_BDS2_B1] = 6.7,
                                          [CODE_BDS2_B2] = 6.7,

                                          [CODE_SBAS_L1CA] = 1.};

static void lock(void) { chMtxLock(&cn0_mutex); }
static void unlock(void) { chMtxUnlock(&cn0_mutex); }

static int find_untracked_sat(code_t code, u32 msk) {
  u16 gnss = code_to_constellation(code);
  unsigned sat_count = constellation_to_sat_count(gnss);

  for (unsigned i = 0; i < sat_count; i++) {
    if (0 == (msk & (1u << i))) {
      return i + 1;
    }
  }
  return -1;
}

static void start_tracker_for_noise_estimation(code_t code) {
  lock();
  u32 msk = mask[code];
  unlock();

  int sat = find_untracked_sat(code, msk);
  if (sat < 0) {
    return; /* no sats to track */
  }

  tracking_startup_params_t startup_params = {
      .mesid = construct_mesid(code, (u16)sat),
      .glo_slot_id = GLO_ORBIT_SLOT_UNKNOWN,
      .sample_count = nap_timing_count(),
      .carrier_freq = 1567.f, /* an arbitrary prime number */
      .code_phase = 131.f,    /* an arbitrary prime number */
      .chips_to_correlate = code_to_chip_count(code),
      .cn0_init = -1.f}; /* this is for noise estimation only */

  tracking_startup_request(&startup_params);
}

void cn0_noise_update_estimate(code_t code, u8 cn0_ms, s32 I, s32 Q) {
  assert(code_valid(code));
  double n = ((double)I * I + (double)Q * Q) / (double)cn0_ms;
  if (has_estimated_noise[code]) {
    noise[code] += (n - noise[code]) * NOISE_ALPHA;
  } else {
    noise[code] = n;
    has_estimated_noise[code] = 1;
  }
}

float cn0_noise_get_estimate(code_t code) {
  assert(code_valid(code));

  if (!init_done) {
    static const code_t used_codes[] = {CODE_GPS_L1CA,
                                        CODE_GPS_L2CM,
                                        CODE_GLO_L1OF,
                                        CODE_GLO_L2OF,
                                        CODE_GAL_E1B,
                                        CODE_GAL_E7I,
                                        CODE_BDS2_B1,
                                        CODE_BDS2_B2};
    init_done = 1;
    for (int i = 0; i < (int)ARRAY_SIZE(used_codes); i++) {
      start_tracker_for_noise_estimation(used_codes[i]);
    }
  }

  if (CODE_SBAS_L1CA == code) {
    return scale_factor[CODE_SBAS_L1CA] * noise[CODE_GPS_L1CA];
  }

  ms[code] += 10; /* the API is called roughly each 10ms */
  bool old_noise_estimation = (ms[code] >= NOISE_MAX_AGE_MS);
  if (old_noise_estimation) {
    start_tracker_for_noise_estimation(code);
    ms[code] = 0;
  }

  return scale_factor[code] * noise[code];
}

void cn0_noise_update_mesid_status(me_gnss_signal_t mesid, bool intrack) {
  assert(mesid_valid(mesid));

  u32 bit = 1u << (mesid.sat - code_to_sat_start(mesid.code));

  lock();
  if (intrack) {
    mask[mesid.code] |= bit;
  } else {
    mask[mesid.code] &= bit;
  }
  unlock();
}
