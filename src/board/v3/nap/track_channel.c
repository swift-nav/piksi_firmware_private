/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nap/track_channel.h"
#include "filters/filter_common.h"
#include "main.h"
#include "me_constants.h"
#include "nap/nap_common.h"
#include "nap_constants.h"
#include "nap_hw.h"
#include "signal_db/signal_db.h"
#include "soft_macq/gal_prns.h"
#include "soft_macq/prns.h"
#include "timing/timing.h"
#include "utils/gnss_capabilities/gnss_capabilities.h"

#include <ch.h>

#include <swiftnav/bits.h>
#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#include <assert.h>
#include <inttypes.h>
#include <string.h>

#define TIMING_COMPARE_DELTA_MIN (1e-3 * NAP_TRACK_SAMPLE_RATE_Hz)

#define NAP_TRACK_CARRIER_FREQ_WIDTH 32
#define NAP_TRACK_CARRIER_PHASE_FRACTIONAL_WIDTH 32
#define NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH 32

#define NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ \
  (((u64)1 << NAP_TRACK_CARRIER_FREQ_WIDTH) / (double)NAP_TRACK_SAMPLE_RATE_Hz)

#define NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE \
  ((u64)1 << NAP_TRACK_CARRIER_PHASE_FRACTIONAL_WIDTH)

#define NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ \
  (NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP / (double)NAP_TRACK_SAMPLE_RATE_Hz)

#define NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP \
  ((u64)1 << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

#define SET_NAP_CORR_LEN(len) (len - 1)

/** Internal tracking channel state */
static struct nap_ch_state {
  me_gnss_signal_t mesid;     /**< Channel ME sid */
  u8 spacing;                 /**< Correlator spacing in samples */
  u32 length[2];              /**< Correlation length in samples of Fs */
  s32 carr_pinc[2];           /**< Carrier phase increment */
  u32 code_pinc[2];           /**< Code phase increment */
  s16 length_adjust;          /**< Adjust the length the next time around */
  u64 sw_code_phase;          /**< Reckoned code phase */
  s64 sw_carr_phase;          /**< Reckoned carrier phase */
  double reckoned_carr_phase; /**< Reckoned carrier phase */
  double code_phase_rate[2];  /**< Code phase rates */
  double fcn_freq_hz;         /**< GLO FCN frequency shift (0 for GPS) */
} nap_ch_desc[MAX_CHANNELS];

/** Compute the correlation length in the units of sampling frequency samples.
 * \param chips_to_correlate The number of chips to correlate over.
 * \param cp_start_frac_units Initial code phase in NAP units.
 * \param cp_rate_units Code phase rate.
 * \return The correlation length in NAP units
 */
static u32 calc_length_samples(u32 chips_to_correlate,
                               u32 cp_start_frac_units,
                               u32 cp_rate_units) {
  u64 cp_end_units = chips_to_correlate * NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  u64 cp_units = cp_end_units - (s32)cp_start_frac_units;
  u32 samples = round(cp_units / (double)cp_rate_units);
  return samples;
}

/** Look-up NAP constellation and band code for the given ME signal ID.
 * \param mesid ME signal ID.
 * \return NAP constellation and band code.
 */
static u8 mesid_to_nap_code(const me_gnss_signal_t mesid) {
  u8 ret = ~0;
  switch ((s8)mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_QZS_L1CA:
      ret = NAP_TRK_CODE_GPS_L1;
      break;
    case CODE_SBAS_L1CA:
      ret = NAP_TRK_CODE_SBAS_L1;
      break;
    case CODE_GPS_L2CM:
    case CODE_QZS_L2CM:
      ret = NAP_TRK_CODE_GPS_L2;
      break;
    case CODE_GLO_L1OF:
      ret = NAP_TRK_CODE_GLO_G1;
      break;
    case CODE_GLO_L2OF:
      ret = NAP_TRK_CODE_GLO_G2;
      break;
    case CODE_BDS2_B1:
      ret = NAP_TRK_CODE_BDS_B1;
      break;
    case CODE_BDS2_B2:
      ret = NAP_TRK_CODE_BDS_B2;
      break;
    case CODE_GPS_L1P:
    case CODE_GPS_L2P:
      assert(!"Unsupported SID");
      break;
    case CODE_GAL_E1B:
#if defined CODE_GAL_E1_SUPPORT && CODE_GAL_E1_SUPPORT > 0
      ret = NAP_TRK_CODE_GAL_E1;
#else
      assert(!"Invalid code");
#endif /* CODE_GAL_E1_SUPPORT*/
      break;
    case CODE_GAL_E7I:
      ret = NAP_TRK_CODE_GAL_E7;
      break;
    default:
      assert(!"Invalid code");
      break;
  }
  return ret;
}

/** Compute the number of samples per code chip.
 * \param code GNSS code identifier.
 * \return Number of samples per code chip.
 */
static double calc_samples_per_chip(double code_phase_rate) {
  return (double)NAP_TRACK_SAMPLE_RATE_Hz / code_phase_rate;
}

void nap_track_init(u8 channel,
                    const me_gnss_signal_t mesid,
                    u64 ref_timing_count,
                    float doppler_freq_hz,
                    double code_phase,
                    u32 chips_to_correlate) {
  assert((mesid.code == CODE_GPS_L1CA) || (mesid.code == CODE_GPS_L2CM) ||
         (mesid.code == CODE_GPS_L5I) || (mesid.code == CODE_SBAS_L1CA) ||
         (mesid.code == CODE_GLO_L1OF) || (mesid.code == CODE_GLO_L2OF) ||
         (mesid.code == CODE_BDS2_B1) || (mesid.code == CODE_BDS2_B2) ||
         (mesid.code == CODE_QZS_L1CA) || (mesid.code == CODE_QZS_L2CM) ||
         (mesid.code == CODE_QZS_L5I) || (mesid.code == CODE_GAL_E1B) ||
         (mesid.code == CODE_GAL_E7I) || (mesid.code == CODE_GAL_E5I));

  swiftnap_tracking_wr_t *t = &NAP->TRK_CH_WR[channel];
  struct nap_ch_state *s = &nap_ch_desc[channel];

  if (mesid.code == CODE_BDS2_B2) {
    log_debug("C%02" PRIu8 " channel %" PRIu8 " t %" PRIxPTR " s %" PRIxPTR,
              mesid.sat,
              channel,
              (uintptr_t)t,
              (uintptr_t)s);
  }
  if (mesid.code == CODE_GAL_E1B) {
    log_debug("E%02" PRIu8 " e1bc channel %" PRIu8 " t %" PRIxPTR
              " s %" PRIxPTR,
              mesid.sat,
              channel,
              (uintptr_t)t,
              (uintptr_t)s);
  }
  if (mesid.code == CODE_GAL_E7I) {
    log_debug("E%02" PRIu8 " e5bIQ channel %" PRIu8 " t %" PRIxPTR
              " s %" PRIxPTR,
              mesid.sat,
              channel,
              (uintptr_t)t,
              (uintptr_t)s);
  }

  if (swiftnap_code_map[channel] != mesid_to_nap_code(mesid)) {
    log_error_mesid(
        mesid, "Tracking channel %u doesn't support this signal.", channel);
    return;
  }

  memset(s, 0, sizeof(*s));
  s->fcn_freq_hz = mesid_to_carr_fcn_hz(mesid);
  s->mesid = mesid;

  /* Set correlator spacing */
  s->spacing = NAP_EPL_SPACING_SAMPLES - 1;

  /* code and carrier frequency */
  double carrier_freq_hz = mesid_to_carr_freq(mesid);
  double chip_rate =
      (1.0 + doppler_freq_hz / carrier_freq_hz) * code_to_chip_rate(mesid.code);

  /* Spacing between VE and P correlators */
  s16 delta_samples = NAP_VEP_SPACING_SAMPLES;

  /* MIC_COMMENT: nap_track_update_init() so that nap_track_update()
   * does not have to branch for the special "init" situation */
  /* Chip rate */
  s->code_phase_rate[1] = s->code_phase_rate[0] = chip_rate;
  u32 cp_rate_units = round(chip_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);
  s->code_pinc[1] = s->code_pinc[0] = cp_rate_units;
  t->CODE_PINC = cp_rate_units;
  /* Integration length */
  u32 length = calc_length_samples(chips_to_correlate, 0, cp_rate_units);
  s->length[1] = s->length[0] = length;
  if ((length < NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MIN_MS)) ||
      (length > NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MAX_MS))) {
    log_error_mesid(s->mesid,
                    "Wrong inital NAP correlation length: "
                    "(%" PRIu32 " %" PRIu32 " %" PRIu32 " %lf)",
                    chips_to_correlate,
                    cp_rate_units,
                    length,
                    chip_rate);
  }
  t->CORR_SET = ((u32)(s->spacing) << NAP_TRK_CH_CORR_SET_SPACING_Pos) |
                SET_NAP_CORR_LEN(length);
  s->length_adjust = delta_samples;
  /* Carrier phase rate */
  double carrier_dopp_hz = -(s->fcn_freq_hz + doppler_freq_hz);
  s32 carr_pinc = round(carrier_dopp_hz * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  s->carr_pinc[1] = s->carr_pinc[0] = carr_pinc;
  t->CARR_PINC = carr_pinc;

  /* Adjust first integration length due to correlator spacing */
  /* was absorbed using `delta_samples` in nap_track_update() above */

  /* get the code rollover point in samples */
  u64 tc_codestart = ref_timing_count - delta_samples -
                     (s32)round(code_phase * calc_samples_per_chip(chip_rate));

  nap_track_enable(channel);

  COMPILER_BARRIER();

  /* Set up timing compare */
  u32 code_chips = code_to_chip_count(mesid.code);
  double code_samples = (double)code_chips * calc_samples_per_chip(chip_rate);

  bool symbol_synced = !code_requires_direct_acq(mesid.code);
  u32 num_codes = 1;
  if (symbol_synced) {
    /* symbol synced code phase must remain symbol synced after propagation */
    if (CODE_GLO_L2OF == mesid.code) {
      /* GLO L2OF has the same symbol (meander) length as GLO L1OF */
      num_codes = GLO_L1CA_SYMBOL_LENGTH_MS / GLO_PRN_PERIOD_MS;
    } else if (CODE_GPS_L2CM == mesid.code) {
      num_codes = GPS_L2C_SYMBOL_LENGTH_MS / GPS_L2CM_PRN_PERIOD_MS;
    } else if (CODE_QZS_L2CM == mesid.code) {
      num_codes = QZS_L2C_SYMBOL_LENGTH_MS / QZS_L2CM_PRN_PERIOD_MS;
    } else if (CODE_BDS2_B2 == mesid.code) {
      if (bds_d2nav(mesid)) {
        num_codes = BDS2_B11_D2NAV_SYMBOL_LENGTH_MS / BDS2_B11_SYMB_LENGTH_MS;
      } else {
        num_codes = BDS2_B11_D1NAV_SYMBOL_LENGTH_MS / BDS2_B11_SYMB_LENGTH_MS;
      }
    } else if (is_gal(mesid.code)) {
      /* default num_codes = 1 */
    } else {
      assert(0);
    }
  }

  chSysLock();

  /* get a reasonable deadline to which propagate to */
  u64 tc_min_propag = NAP->TIMING_COUNT + TIMING_COMPARE_DELTA_MIN;
  /* extend tc_min_propag - cannot use helper function in syslock */
  tc_min_propag += (tc_codestart >> 32) << 32;
  if (tc_min_propag < tc_codestart) {
    tc_min_propag += (1ULL << 32);
  }

  u32 samples_diff = tc_min_propag - tc_codestart;
  u32 tmp = (u32)floor((double)samples_diff / code_samples);
  assert(num_codes);
  num_codes *= (1 + (tmp / num_codes));

  u64 tc_next_rollover =
      tc_codestart + (u64)floor(0.5 + (double)num_codes * code_samples);

  u8 index = 0;
  if (mesid.code == CODE_GPS_L2CM) {
    index = (num_codes % GPS_L2CL_PRN_START_POINTS);
  }

#if defined CODE_GAL_E1_SUPPORT && CODE_GAL_E1_SUPPORT > 0
  if (mesid.code == CODE_GAL_E1B) {
    index = mesid.sat - 1;
    for (u16 k = 0; k < GAL_E1B_PRN_BYTES; k++) {
      NAP->TRK_GAL_E1_MEMCFG =
          (1 << (18 + channel - NAP_FIRST_GAL_E1_CHANNEL)) | (0 << 17) |
          (k << 8) | gal_e1b_codes[index][k];
    }
    for (u16 k = 0; k < GAL_E1C_PRN_BYTES; k++) {
      NAP->TRK_GAL_E1_MEMCFG =
          (1 << (18 + channel - NAP_FIRST_GAL_E1_CHANNEL)) | (1 << 17) |
          (k << 8) | gal_e1c_codes[index][k];
    }
  } else {
#endif /* CODE_GAL_E1_SUPPORT */
    NAP->TRK_CODE_LFSR0_INIT = mesid_to_lfsr0_init(mesid);
    NAP->TRK_CODE_LFSR0_RESET = mesid_to_lfsr0_init(mesid);
    NAP->TRK_CODE_LFSR0_LAST = mesid_to_lfsr0_last(mesid);

    NAP->TRK_CODE_LFSR1_INIT = mesid_to_lfsr1_init(mesid, index);
    NAP->TRK_CODE_LFSR1_RESET = mesid_to_lfsr1_init(mesid, 0);
    NAP->TRK_CODE_LFSR1_LAST = mesid_to_lfsr1_last(mesid);

    if (mesid.code == CODE_GAL_E5I) {
      index = mesid.sat - 1;
      NAP->TRK_SEC_CODE[3] = getbitu(gal_e5q_sec_codes[index], 0, 4);
      NAP->TRK_SEC_CODE[2] = getbitu(gal_e5q_sec_codes[index], 4, 32);
      NAP->TRK_SEC_CODE[1] = getbitu(gal_e5q_sec_codes[index], 36, 32);
      NAP->TRK_SEC_CODE[0] = getbitu(gal_e5q_sec_codes[index], 68, 32);
    } else if (mesid.code == CODE_GAL_E7I) {
      index = mesid.sat - 1;
      NAP->TRK_SEC_CODE[3] = getbitu(gal_e7q_sec_codes[index], 0, 4);
      NAP->TRK_SEC_CODE[2] = getbitu(gal_e7q_sec_codes[index], 4, 32);
      NAP->TRK_SEC_CODE[1] = getbitu(gal_e7q_sec_codes[index], 36, 32);
      NAP->TRK_SEC_CODE[0] = getbitu(gal_e7q_sec_codes[index], 68, 32);
    }
#if defined CODE_GAL_E1_SUPPORT && CODE_GAL_E1_SUPPORT > 0
  }
#endif /* CODE_GAL_E1_SUPPORT */

  /* port FCN-induced NCO phase to a common receiver clock point */
  s->reckoned_carr_phase = (s->fcn_freq_hz) *
                           (tc_next_rollover % FCN_NCO_RESET_COUNT) /
                           NAP_TRACK_SAMPLE_RATE_Hz;

  tc_next_rollover &= 0xFFFFFFFF;

  NAP->TRK_TIMING_COMPARE = tc_next_rollover;
  chSysUnlock();

  /* Sleep until compare match */
  s32 tc_delta;
  while ((tc_delta = (tc_next_rollover - NAP->TIMING_COUNT)) >= 0) {
    systime_t sleep_time =
        floor(CH_CFG_ST_FREQUENCY * tc_delta / NAP_TRACK_SAMPLE_RATE_Hz);

    /* The next system tick will always occur less than the nominal tick period
     * in the future, so sleep for an extra tick. */
    chThdSleep(1 + sleep_time / 2);
  }
}

void nap_track_update(u8 channel,
                      double doppler_freq_hz,
                      double chip_rate,
                      u32 chips_to_correlate,
                      bool has_pilot_sync) {
  swiftnap_tracking_wr_t *t = &NAP->TRK_CH_WR[channel];
  struct nap_ch_state *s = &nap_ch_desc[channel];

  /* CHIP RATE --------------------------------------------------------- */
  u32 code_phase_frac = (u32)s->sw_code_phase + s->code_pinc[0] * s->length[0];

  s->code_phase_rate[1] = s->code_phase_rate[0];
  s->code_phase_rate[0] = chip_rate;

  u32 code_units = round(chip_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);
  s->code_pinc[1] = s->code_pinc[0];
  s->code_pinc[0] = code_units;

  t->CODE_PINC = code_units;

  /* INTEGRATION LENGTH ------------------------------------------------ */
  u32 length =
      calc_length_samples(chips_to_correlate, code_phase_frac, code_units);
  length += s->length_adjust;
  s->length_adjust = 0;
  s->length[1] = s->length[0];
  s->length[0] = length;

  t->CORR_SET =
      ((u32)has_pilot_sync << NAP_TRK_CH_CORR_SET_SEC_CODE_ENABLE_Pos) |
      ((u32)(s->spacing) << NAP_TRK_CH_CORR_SET_SPACING_Pos) |
      SET_NAP_CORR_LEN(length);

  if ((length < NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MIN_MS)) ||
      (length > NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MAX_MS))) {
    log_warn_mesid(s->mesid,
                   "Wrong NAP correlation length: "
                   "(%" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %lf)",
                   chips_to_correlate,
                   code_phase_frac,
                   code_units,
                   length,
                   chip_rate);
  }

  /* CARRIER (+FCN) FREQ ---------------------------------------------- */
  /* Note: s->fcn_freq_hz is non zero for Glonass only */
  double carrier_freq_hz = -(s->fcn_freq_hz + doppler_freq_hz);

  s32 carr_pinc = round(carrier_freq_hz * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  s->carr_pinc[1] = s->carr_pinc[0];
  s->carr_pinc[0] = carr_pinc;

  t->CARR_PINC = carr_pinc;
}

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))

/* see if not enforcing `const volatile` leads to better compiler optimization
 * below */
typedef struct {
  u32 STATUS;
  u32 TIMING_SNAPSHOT;
  s16 CORR16[12];
} tracking_rd_t;

void nap_track_read_results(u8 channel,
                            u32 *count_snapshot,
                            corr_t corrs[],
                            double *code_phase_prompt,
                            double *carrier_phase) {
  tracking_rd_t trk_ch;
  swiftnap_tracking_rd_t *t = &NAP->TRK_CH_RD[channel];
  struct nap_ch_state *s = &nap_ch_desc[channel];
  volatile u32 corr_val = 0;

  trk_ch.STATUS = t->STATUS;

  trk_ch.TIMING_SNAPSHOT = t->TIMING_SNAPSHOT;
  *count_snapshot = trk_ch.TIMING_SNAPSHOT;

  /* pilot/data correlator values in sequence E-P-L-E-P-L */
  for (u8 i = 0; i < 6; i++) {
    corr_val = t->CORR;
    corrs[i].I = (s16)(corr_val & 0xFFFF);
    corrs[i].Q = (s16)((corr_val >> 16) & 0xFFFF);
  }

  /* Spacing between VE and P correlators */
  double prompt_offset =
      NAP_VEP_SPACING_SAMPLES / calc_samples_per_chip(s->code_phase_rate[1]);

  /* Code and carrier phase reckoning */
  s64 carr_phase_incr = ((s64)s->length[1]) * s->carr_pinc[1];
  u64 code_phase_incr = ((u64)s->length[1]) * s->code_pinc[1];

  s->sw_carr_phase += carr_phase_incr;
  s->sw_code_phase += code_phase_incr;

  s->sw_code_phase =
      ((s->sw_code_phase >> 32) % code_to_chip_count(s->mesid.code) << 32) |
      (s->sw_code_phase & 0xFFFFFFFF);

  *code_phase_prompt =
      ((double)s->sw_code_phase) / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP -
      prompt_offset;

  if (*code_phase_prompt < 0) {
    *code_phase_prompt += code_to_chip_count(s->mesid.code);
  }

  s->reckoned_carr_phase +=
      ((double)carr_phase_incr) / NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE +
      s->fcn_freq_hz * (s->length[1] / NAP_TRACK_SAMPLE_RATE_Hz);

  *carrier_phase = (s->reckoned_carr_phase);

#ifndef PIKSI_RELEASE
  if (GET_NAP_TRK_CH_STATUS_CORR_OVERFLOW(trk_ch.STATUS)) {
    log_warn_mesid(s->mesid,
                   "Tracking correlator overflow VE:[%+7" PRIi32 ":%+7" PRIi32
                   "] E:[%+7" PRIi32 ":%+7" PRIi32 "] P:[%+7" PRIi32
                   ":%+7" PRIi32 "] L:[%+7" PRIi32 ":%+7" PRIi32
                   "] VL:[%+7" PRIi32 ":%+7" PRIi32 "]",
                   corrs[3].I,
                   corrs[3].Q,
                   corrs[0].I,
                   corrs[0].Q,
                   corrs[1].I,
                   corrs[1].Q,
                   corrs[2].I,
                   corrs[2].Q,
                   corrs[4].I,
                   corrs[4].Q);
  }

  /* Check carrier phase reckoning */
  u8 sw_carr_phase = (s->sw_carr_phase >> 29) & 0x3F;
  u8 hw_carr_phase = GET_NAP_TRK_CH_STATUS_CARR_PHASE_INT(trk_ch.STATUS)
                         << NAP_TRK_CH_STATUS_CARR_PHASE_FRAC_Len |
                     GET_NAP_TRK_CH_STATUS_CARR_PHASE_FRAC(trk_ch.STATUS);
  if (sw_carr_phase != hw_carr_phase) {
    log_error_mesid(s->mesid,
                    "Carrier reckoning: SW=%" PRIu8 ".%" PRIu8 ", HW=%" PRIu8
                    ".%" PRIu8,
                    (sw_carr_phase >> 3),
                    (sw_carr_phase & 0x7),
                    (hw_carr_phase >> 3),
                    (hw_carr_phase & 0x7));
  }

  /* Check code phase reckoning */
  u8 sw_code_phase = (s->sw_code_phase >> 29) & 0x3F;
  u8 hw_code_phase = GET_NAP_TRK_CH_STATUS_CODE_PHASE_INT(trk_ch.STATUS)
                         << NAP_TRK_CH_STATUS_CODE_PHASE_FRAC_Len |
                     GET_NAP_TRK_CH_STATUS_CODE_PHASE_FRAC(trk_ch.STATUS);
  if (sw_code_phase != hw_code_phase) {
    log_error_mesid(s->mesid,
                    "Code reckoning: SW=%" PRIu8 ".%" PRIu8 ", HW=%" PRIu8
                    ".%" PRIu8,
                    (sw_code_phase >> 3),
                    (sw_code_phase & 0x7),
                    (hw_code_phase >> 3),
                    (hw_code_phase & 0x7));
  }
#endif /* PIKSI_RELEASE */
}

void nap_track_enable(u8 channel) {
  if (channel < 32) {
    NAP->TRK_CONTROL[0] |= (1 << channel);
  } else if (channel < 64) {
    NAP->TRK_CONTROL[1] |= (1 << (channel - 32));
  } else {
    NAP->TRK_CONTROL[2] |= (1 << (channel - 64));
  }
}

void nap_track_disable(u8 channel) {
  if (channel < 32) {
    NAP->TRK_CONTROL[0] &= ~(1 << channel);
  } else if (channel < 64) {
    NAP->TRK_CONTROL[1] &= ~(1 << (channel - 32));
  } else {
    NAP->TRK_CONTROL[2] &= ~(1 << (channel - 64));
  }
}

bool nap_track_supports(u8 channel, const me_gnss_signal_t mesid) {
  return swiftnap_code_map[channel] == mesid_to_nap_code(mesid);
}
