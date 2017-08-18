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
#include "main.h"
#include "nap/nap_common.h"
#include "nap_constants.h"
#include "nap_hw.h"
#include "signal.h"
#include "timing.h"
#include "track.h"

#include <ch.h>

#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/prns.h> /* to expose sid_to_init_g1() declaration */
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <assert.h>
#include <string.h>

#define TIMING_COMPARE_DELTA_MIN (1e-3 * NAP_TRACK_SAMPLE_RATE_Hz) /*   1ms */
#define TIMING_COMPARE_DELTA_MAX               \
  (100e-3 * NAP_TRACK_SAMPLE_RATE_Hz) /* 100ms \
                                         */

/* NAP track channel parameters. */
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

#define GET_NAP_CORR_LEN (1 + GET_NAP_TRK_CH_CONTROL_LENGTH(t->CONTROL))
#define SET_NAP_CORR_LEN(len) \
  (SET_NAP_TRK_CH_CONTROL_LENGTH(t->CONTROL, len - 1))

/** Structure is used to define spacing between two correlators */
typedef struct {
  u8 chips : 3;   /**< Correlator spacing in chips. */
  u8 samples : 6; /**< Correlator spacing in samples. */
} nap_spacing_t;

/** Internal tracking channel state */
static struct nap_ch_state {
  me_gnss_signal_t mesid;    /**< Channel ME sid */
  nap_spacing_t spacing[4];  /**< Correlator spacing. */
  double code_phase_rate[2]; /**< Code phase rates. */
  /* The frequency shift due to GLO FCNs [Hz]. Set to zero for GPS. */
  double fcn_freq_hz;
  /** Doppler induced carrier phase.
      Does not include FCN induced carrier phase change. */
  double reckoned_carr_phase;
  u32 length[2];      /**< Correlation length in samples of Fs */
  s32 carr_pinc[2];   /**< Carrier phase increment */
  u64 reckon_counter; /**< First carrier phase has to be read from NAP */
  s64 sw_carr_phase;  /**< Debug reckoned carrier phase */
  s16 length_adjust;  /**< Adjust the length the next time around */
} nap_ch_desc[MAX_CHANNELS];

/** Internal tracking channel capability = supported code */
static u8 nap_ch_capability[MAX_CHANNELS];

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
  /* cp_start_frac_units is reinterpreted as a signed value. This works
   * because NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH is equal to 32 */
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
  switch (mesid.code) {
    case CODE_GPS_L1CA:
    case CODE_SBAS_L1CA:
      ret = NAP_TRK_CODE_GPS_L1;
      break;
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
      ret = NAP_TRK_CODE_GPS_L2;
      break;
    case CODE_GLO_L1CA:
      ret = NAP_TRK_CODE_GLO_G1;
      break;
    case CODE_GLO_L2CA:
      ret = NAP_TRK_CODE_GLO_G2;
      break;
    case CODE_GPS_L1P:
    case CODE_GPS_L2P:
      assert(!"Unsupported SID");
      break;
    case CODE_INVALID:
    case CODE_COUNT:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    default:
      assert(!"Invalid code");
      break;
  }
  return ret;
}

/** Convert spacing structure to NAP offset register value.
 * \param spacing Correlator spacing.
 * \return NAP offfset register value.
 */
static u16 spacing_to_nap_offset(nap_spacing_t spacing) {
  return ((u16)(spacing.chips & NAP_TRK_SPACING_CHIPS_Msk)
          << NAP_TRK_SPACING_CHIPS_Pos) |
         ((spacing.samples & NAP_TRK_SPACING_SAMPLES_Msk)
          << NAP_TRK_SPACING_SAMPLES_Pos);
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
         (mesid.code == CODE_GPS_L2CL) || (mesid.code == CODE_GLO_L1CA) ||
         (mesid.code == CODE_GLO_L2CA));

  swiftnap_tracking_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_desc[channel];

  if (nap_ch_capability[channel] != mesid_to_nap_code(mesid)) {
    log_error_mesid(
        mesid, "Tracking channel %u doesn't support this signal.", channel);
    return;
  }

  memset(s, 0, sizeof(*s));
  s->fcn_freq_hz = mesid_to_carr_fcn_hz(mesid);
  s->mesid = mesid;

  /* Correlator spacing: VE -> E */
  if (IS_GLO(mesid)) {
    s->spacing[0] = (nap_spacing_t){.chips = NAP_VE_E_SPACING_CHIPS,
                                    .samples = NAP_VE_E_GLO_SPACING_SAMPLES};
  } else {
    s->spacing[0] = (nap_spacing_t){.chips = NAP_VE_E_SPACING_CHIPS,
                                    .samples = NAP_VE_E_GPS_SPACING_SAMPLES};
  }

  /* Correlator spacing: E -> P (samples only) */
  s->spacing[1] = (nap_spacing_t){.chips = 0, .samples = NAP_SPACING_SAMPLES};

  /* Correlator spacing: P -> L (samples only) */
  s->spacing[2] = (nap_spacing_t){.chips = 0, .samples = NAP_SPACING_SAMPLES};

  /* Correlator spacing: L -> VL */
  s->spacing[3] = (nap_spacing_t){.chips = NAP_SPACING_CHIPS,
                                  .samples = NAP_SPACING_SAMPLES};

  /* NOTE: NAP_TRK_CH_CONTROL_SAT field is ignored for GLO satellites.
   * NAP does not need GLO PRN as code is same for all GLO satellites. */
  u8 prn = mesid.sat - GPS_FIRST_PRN;
  t->CONTROL = SET_NAP_TRK_CH_CONTROL_SAT(t->CONTROL, prn);

  /* Set correlator spacing */
  t->SPACING =
      (spacing_to_nap_offset(s->spacing[0]) << NAP_TRK_CH_SPACING_OFFSET0_Pos) |
      (spacing_to_nap_offset(s->spacing[1]) << NAP_TRK_CH_SPACING_OFFSET1_Pos) |
      (spacing_to_nap_offset(s->spacing[2]) << NAP_TRK_CH_SPACING_OFFSET2_Pos) |
      (spacing_to_nap_offset(s->spacing[3]) << NAP_TRK_CH_SPACING_OFFSET3_Pos);

  /* code and carrier frequency */
  double carrier_freq_hz = mesid_to_carr_freq(mesid);
  double chip_rate =
      (1.0 + doppler_freq_hz / carrier_freq_hz) * code_to_chip_rate(mesid.code);

  /* Spacing between VE and P correlators */
  s16 delta_samples = s->spacing[0].samples + s->spacing[1].samples +
                      round((s->spacing[0].chips + s->spacing[1].chips) *
                            calc_samples_per_chip(chip_rate));

  /* Delay L2CL code phase by 1 chip to accommodate zero in the L2CM slot */
  /* Initial correlation length for L2CL is thus 1 chip shorter,
   * since first L2CM chip is skipped */
  if (mesid.code == CODE_GPS_L2CL) {
    code_phase += 1.0f;
    delta_samples -= calc_samples_per_chip(chip_rate);
  }

  /* MIC_COMMENT: nap_track_update_init() so that nap_track_update()
   * does not have to branch for the special "init" situation */
  /* Chip rate */
  s->code_phase_rate[1] = s->code_phase_rate[0] = chip_rate;
  u32 cp_rate_units = round(chip_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);
  t->CODE_PINC = cp_rate_units;
  /* Integration length */
  u32 length = calc_length_samples(chips_to_correlate, 0, cp_rate_units);
  s->length[1] = s->length[0] = length;
  if ((length < NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MIN_MS)) ||
      (length > NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MAX_MS))) {
    log_warn_mesid(s->mesid,
                   "Wrong inital NAP correlation length: "
                   "(%" PRIu32 " %" PRIu32 " %" PRIu32 " %lf)",
                   chips_to_correlate,
                   cp_rate_units,
                   length,
                   chip_rate);
  }
  t->CONTROL = SET_NAP_CORR_LEN(length);
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
  chSysLock();
  /* get a reasonable deadline to which propagate to */
  u64 tc_min_propag = NAP->TIMING_COUNT + TIMING_COMPARE_DELTA_MIN;
  /* extend tc_min_propag - cannot use helper function in syslock */
  tc_min_propag += (tc_codestart >> 32) << 32;
  if (tc_min_propag < tc_codestart) {
    tc_min_propag += (1ULL << 32);
  }

  u32 samples_diff = tc_min_propag - tc_codestart;
  u32 code_chips, num_codes;
  u64 tc_next_rollover;
  double code_samples;
  u8 index = 0;
  if (mesid.code == CODE_GPS_L2CL) {
    code_chips = GPS_L2CL_PRN_CHIPS_PER_INTERVAL;
    code_samples = (double)code_chips * calc_samples_per_chip(chip_rate);
    num_codes = 1 + (u32)floor((double)samples_diff / code_samples);
    index = (num_codes % GPS_L2CL_PRN_START_POINTS);
  } else {
    code_chips = code_to_chip_count(mesid.code);
    code_samples = (double)code_chips * calc_samples_per_chip(chip_rate);
    num_codes = 1 + (u32)floor((double)samples_diff / code_samples);
  }
  tc_next_rollover =
      tc_codestart + (u64)floor(0.5 + (double)num_codes * code_samples);

  NAP->TRK_CODE_INT_INIT = index * code_chips;
  NAP->TRK_CODE_FRAC_INIT = 0;

  NAP->TRK_CODE_INT_MAX = code_to_chip_count(mesid.code) - 1;

  NAP->TRK_CODE_LFSR0_INIT = mesid_to_init_g1(mesid, index);
  NAP->TRK_CODE_LFSR0_RESET = mesid_to_init_g1(mesid, 0);
  NAP->TRK_CODE_LFSR1_INIT = mesid_to_init_g2(mesid);
  NAP->TRK_CODE_LFSR1_RESET = mesid_to_init_g2(mesid);

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
                      u8 corr_spacing) {
  (void)corr_spacing; /* This is always written as 0, for now */

  swiftnap_tracking_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_desc[channel];

  /* CHIP RATE --------------------------------------------------------- */
  /* MIC_COMMENT: the use of s->code_phase_rate[2] is pretty limited:
   * just converts samples to chips in nap_track_read_results() but
   * the difference between the nominal one and the real (with Doppler)
   * one will produce a sub-mm difference.. so what is its point? */
  /* MIC_COMMENT: do we need to read from NAP the length in the else below?
   * we should be able to use the reckoned value, but I am not sure if
   * this should be s->length[1] or s->length[0].. it probably does not
   * matter much in a tracking loop scenario.. */
  /* MIC_COMMENT: so I'd probably remove this s->code_phase_rate[2] and use
   * a s->code_pinc[2] to reckon code increments */
  u32 code_phase_frac = t->CODE_PHASE_FRAC + t->CODE_PINC * (s->length[0]);
  s->code_phase_rate[1] = s->code_phase_rate[0];
  s->code_phase_rate[0] = chip_rate;

  u32 code_units = round(chip_rate * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);

  t->CODE_PINC = code_units;

  /* INTEGRATION LENGTH ------------------------------------------------ */
  u32 length =
      calc_length_samples(chips_to_correlate, code_phase_frac, code_units);
  length += s->length_adjust;
  s->length_adjust = 0;
  s->length[1] = s->length[0];
  s->length[0] = length;

  t->CONTROL = SET_NAP_CORR_LEN(length);

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

void nap_track_read_results(u8 channel,
                            u32 *count_snapshot,
                            corr_t corrs[],
                            double *code_phase_prompt,
                            double *carrier_phase) {
  static swiftnap_tracking_t trk_ch;
  swiftnap_tracking_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_desc[channel];
  s64 hw_carr_phase;

  /* Read track channel data
   * NOTE: Compiler couldn't optimize MEMCPY_S over AXI so using regular memcpy
   */
  memcpy(&trk_ch, t, sizeof(swiftnap_tracking_t));

  if (GET_NAP_TRK_CH_STATUS_CORR_OVERFLOW(trk_ch.STATUS)) {
    log_warn_mesid(
        s->mesid, "Tracking correlator overflow on channel %d", channel);
  }

  /* E correlator */
  corrs[0].I = (s16)(trk_ch.CORR1 & 0xFFFF);
  corrs[0].Q = (s16)((trk_ch.CORR1 >> 16) & 0xFFFF);

  /* P correlator */
  corrs[1].I = (s16)(trk_ch.CORR2 & 0xFFFF);
  corrs[1].Q = (s16)((trk_ch.CORR2 >> 16) & 0xFFFF);

  /* L correlator */
  corrs[2].I = (s16)(trk_ch.CORR3 & 0xFFFF);
  corrs[2].Q = (s16)((trk_ch.CORR3 >> 16) & 0xFFFF);

  /* VE correlator */
  corrs[3].I = (s16)(trk_ch.CORR0 & 0xFFFF);
  corrs[3].Q = (s16)((trk_ch.CORR0 >> 16) & 0xFFFF);

  /* VL correlator */
  corrs[4].I = (s16)(trk_ch.CORR4 & 0xFFFF);
  corrs[4].Q = (s16)((trk_ch.CORR4 >> 16) & 0xFFFF);

  *count_snapshot = trk_ch.TIMING_SNAPSHOT;

  if (s->reckon_counter < 1) {
    hw_carr_phase = ((s64)trk_ch.CARR_PHASE_INT << 32) | trk_ch.CARR_PHASE_FRAC;
    s->sw_carr_phase = hw_carr_phase;
    s->reckoned_carr_phase +=
        ((double)hw_carr_phase) / NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE;
    log_debug_mesid(
        s->mesid,
        "init carr phase %.6lf",
        (double)hw_carr_phase / NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE);
  } else {
    s64 phase_incr = ((s64)s->length[1]) * (s->carr_pinc[1]);
    s->reckoned_carr_phase +=
        ((double)phase_incr) / NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE;
#ifndef PIKSI_RELEASE
    s->sw_carr_phase += phase_incr;
    hw_carr_phase = ((s64)trk_ch.CARR_PHASE_INT << 32) | trk_ch.CARR_PHASE_FRAC;
    if (s->sw_carr_phase != hw_carr_phase) {
      log_error_mesid(
          s->mesid,
          "%12llu reckon err SW %+.9lf  HW %+.9lf DIFF %+.9lf",
          s->reckon_counter,
          (double)s->sw_carr_phase / NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE,
          (double)hw_carr_phase / NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE,
          ((double)s->sw_carr_phase / NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE) -
              ((double)hw_carr_phase /
               NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE));
      s->sw_carr_phase = hw_carr_phase;
    }
#endif /* PIKSI_RELEASE */
  }
  s->reckoned_carr_phase +=
      s->fcn_freq_hz * (s->length[1] / NAP_TRACK_SAMPLE_RATE_Hz);
  s->reckon_counter++;

  *carrier_phase = -(s->reckoned_carr_phase);

  /* Spacing between VE and P correlators */
  double prompt_offset = s->spacing[0].chips + s->spacing[1].chips +
                         (s->spacing[0].samples + s->spacing[1].samples) /
                             calc_samples_per_chip(s->code_phase_rate[1]);

  u64 nap_code_phase =
      ((u64)trk_ch.CODE_PHASE_INT << 32) | trk_ch.CODE_PHASE_FRAC;

  /* Correct code phase with spacing between VE and P correlators */
  *code_phase_prompt =
      (double)nap_code_phase / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP -
      prompt_offset;

  if (*code_phase_prompt < 0) {
    *code_phase_prompt += code_to_chip_count(s->mesid.code);
  }
}

void nap_track_enable(u8 channel) {
  if (channel < 32) {
    NAP->TRK_CONTROL0 |= (1 << channel);
  } else {
    NAP->TRK_CONTROL1 |= (1 << (channel - 32));
  }
}

void nap_track_disable(u8 channel) {
  if (channel < 32) {
    NAP->TRK_CONTROL0 &= ~(1 << channel);
  } else {
    NAP->TRK_CONTROL1 &= ~(1 << (channel - 32));
  }
}

void nap_scan_channels() {
  for (u8 channel = 0; channel < nap_track_n_channels; ++channel) {
    swiftnap_tracking_t *t = &NAP->TRK_CH[channel];
    nap_ch_capability[channel] = GET_NAP_TRK_CH_STATUS_CODE(t->STATUS);
  }
}

bool nap_track_supports(u8 channel, const me_gnss_signal_t mesid) {
  return nap_ch_capability[channel] == mesid_to_nap_code(mesid);
}
