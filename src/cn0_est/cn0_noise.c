#include <assert.h>
#include <ch.h>
#include <math.h>
#include <swiftnav/signal.h>
#include "acq/manage.h"
#include "signal_db/signal_db.h"

#define NOISE_ALPHA 0.001
#define NOISE_MAX_AGE_MS 5000 /* re-estimate the noise with this rate */

static MUTEX_DECL(cn0_mutex);

static u64 ms[CODE_COUNT] = {0};
static u32 mask[CODE_COUNT] = {0};
static double noise[CODE_COUNT] = {0};

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
  log_info("Ar %d %d", code, sat);
}

void cn0_noise_estimate(code_t code, u8 cn0_ms, s32 I, s32 Q) {
  assert(code_valid(code));
  double i = I / (double)cn0_ms;
  double q = Q / (double)cn0_ms;
  double n = i * i + q * q;
  if (noise[code] <= 0) {
    noise[code] = n;
  } else {
    noise[code] += (n - noise[code]) * NOISE_ALPHA;
  }
}

float cn0_noise_get_estimate(code_t code) {
  assert(code_valid(code));

  if (CODE_SBAS_L1CA == code) {
    return cn0_noise_get_estimate(CODE_GPS_L1CA);
  }

  bool no_noise_estimation = (noise[code] <= 0);

  ms[code] += 10; /* the API is called roughly each 10ms */
  bool old_noise_estimation = (ms[code] > NOISE_MAX_AGE_MS);

  if (no_noise_estimation || old_noise_estimation) {
    start_tracker_for_noise_estimation(code);
    ms[code] = 0;
  }

  if (no_noise_estimation) {
    noise[code] = 151.f;
  }
  float n = sqrt(noise[code]);

  if (old_noise_estimation) {
    log_info("A %d noise = %.1f", code, n);
  }

  return 1100. * n;
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
