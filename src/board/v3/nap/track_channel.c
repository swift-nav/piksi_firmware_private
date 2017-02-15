/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nap_constants.h"
#include "nap_hw.h"
#include "nap/nap_common.h"
#include "nap/track_channel.h"
#include "track.h"
#include "main.h"

#include <ch.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>
#include <libswiftnav/prns.h>   /* to expose sid_to_init_g1() declaration */

#include <assert.h>
#include <string.h>


#define TIMING_COMPARE_DELTA_MIN (  1e-3 * NAP_TRACK_SAMPLE_RATE_Hz) /*   1ms */
#define TIMING_COMPARE_DELTA_MAX (100e-3 * NAP_TRACK_SAMPLE_RATE_Hz) /* 100ms */

/* NAP track channel parameters. */
#define NAP_TRACK_CARRIER_FREQ_WIDTH              32
#define NAP_TRACK_CARRIER_PHASE_FRACTIONAL_WIDTH  32
#define NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH     32

#define NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ       \
  (((u64)1 << NAP_TRACK_CARRIER_FREQ_WIDTH) / (double)NAP_TRACK_SAMPLE_RATE_Hz)

#define NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE   \
  ((u64)1 << NAP_TRACK_CARRIER_PHASE_FRACTIONAL_WIDTH)

#define NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ    \
  (NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP / (double)NAP_TRACK_SAMPLE_RATE_Hz)

#define NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP       \
  ((u64)1 << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH)

/** Structure is used to define spacing between two correlators */
typedef struct {
  u8 chips:3;   /**< Correlator spacing in chips (<= 7) */
  u8 samples:6; /**< Correlator spacing in samples (<= 23)*/
} nap_spacing_t;

enum steps {
  STEP0 = 1 << 0,
  STEP1 = 1 << 1,
  STEP2 = 1 << 2,
  STEP3 = 1 << 3,
  STEP4 = 1 << 4,
  STEP5 = 1 << 5,
  STEP6 = 1 << 6,
  STEP7 = 1 << 7,
  STEP8 = 1 << 8,
  STEP9 = 1 << 9,
  STEP10 = 1 << 10,
  STEP11 = 1 << 11,
  STEP12 = 1 << 12,
  STEP13 = 1 << 13,
  STEP14 = 1 << 14,
  STEP15 = 1 << 15
};

/** Internal tracking channel state */
static struct nap_ch_state {
  bool init;                   /**< Initializing channel. */
  bool first_interrupt;         /**< The first interrup flag */
  u16 steps;
  gnss_signal_t sid;           /**< Channel sid */
  code_t code;                 /**< GNSS code identifier. */
  nap_spacing_t spacing[4];    /**< Correlator spacing. */
  double code_phase_rate[2];   /**< Code phase rates. */

  u8 channel;
  u32 ref_timing_count;
  float carrier_freq;
  float code_phase;
  u32 chips_to_correlate;
} nap_ch_state[NAP_MAX_N_TRACK_CHANNELS];

/** Compute the correlation length in the units of sampling frequency samples.
 * \param chips_to_correlate The number of chips to correlate over.
 * \param cp_start_frac_units Initial code phase in NAP units.
 * \param cp_rate_units Code phase rate.
 * \return The correlation length in NAP units
 */
static u32 calc_length_samples(u32 chips_to_correlate, u32 cp_start_frac_units,
                               u32 cp_rate_units)
{
  u64 cp_end_units = chips_to_correlate * NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  /* cp_start_frac_units is reinterpreted as a signed value. This works
   * because NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH is equal to 32 */
  u64 cp_units = cp_end_units - (s32)cp_start_frac_units;
  u32 samples = round(cp_units / (double)cp_rate_units);

  return samples;
}

/** Looks-up RF frontend channel for the given signal ID.
 * \param sid Signal ID.
 * \return RF front-end channel number.
 */
u8 sid_to_rf_frontend_channel(gnss_signal_t sid)
{
  u8 ret = ~0;
  switch (sid.code) {
  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    ret = NAP_RF_FRONTEND_CHANNEL_1;
    break;
  case CODE_GPS_L2CM:
    ret = NAP_RF_FRONTEND_CHANNEL_4;
    break;
  case CODE_GLO_L1CA:
  case CODE_GLO_L2CA:
  case CODE_GPS_L1P:
  case CODE_GPS_L2P:
    assert(!"Unsupported SID");
    break;
  case CODE_INVALID:
  case CODE_COUNT:
  default:
    assert(!"Invalid code");
    break;
  }
  return ret;
}

/** Looks-up NAP constellation and band code for the given signal ID.
 * \param sid Signal ID.
 * \return NAP constallation and band code.
 */
u8 sid_to_nap_code(gnss_signal_t sid)
{
  u8 ret = ~0;
  switch (sid.code) {
  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    ret = NAP_CODE_GPS_L1CA_SBAS_L1CA;
    break;
  case CODE_GPS_L2CM:
    ret = NAP_CODE_GPS_L2CM;
    break;
  case CODE_GLO_L1CA:
  case CODE_GLO_L2CA:
  case CODE_GPS_L1P:
  case CODE_GPS_L2P:
    assert(!"Unsupported SID");
    break;
  case CODE_INVALID:
  case CODE_COUNT:
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
static u16 spacing_to_nap_offset(nap_spacing_t spacing)
{
  return ((u16)(spacing.chips & NAP_TRK_SPACING_CHIPS_Msk)
      << NAP_TRK_SPACING_CHIPS_Pos) |
      ((spacing.samples & NAP_TRK_SPACING_SAMPLES_Msk)
      << NAP_TRK_SPACING_SAMPLES_Pos);
}

/** Compute the number of samples per code chip.
 * \param code GNSS code identifier.
 * \return Number of samples per code chip.
 */
static double calc_samples_per_chip(double code_phase_rate)
{
  return (double)NAP_TRACK_SAMPLE_RATE_Hz / code_phase_rate;
}

void nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                   float carrier_freq, float code_phase, u32 chips_to_correlate)
{
  assert((sid.code == CODE_GPS_L1CA) || (sid.code == CODE_GPS_L2CM));

  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];

  s->channel = channel;
  s->ref_timing_count = ref_timing_count;
  s->carrier_freq = carrier_freq;
  s->code_phase = code_phase;
  s->chips_to_correlate = chips_to_correlate;

  s->first_interrupt = true;
  s->steps = STEP0;

  s->sid = sid;

  /* Correlator spacing: VE -> E */
  s->spacing[0] = (nap_spacing_t){.chips = NAP_VE_E_SPACING_CHIPS,
                                  .samples = NAP_VE_E_SPACING_SAMPLES};

  /* Correlator spacing: E -> P (samples only) */
  s->spacing[1] = (nap_spacing_t){.chips = 0,
                                  .samples = NAP_SPACING_SAMPLES};

  /* Correlator spacing: P -> L (samples only) */
  s->spacing[2] = (nap_spacing_t){.chips = 0,
                                  .samples = NAP_SPACING_SAMPLES};

  /* Correlator spacing: L -> VL */
  s->spacing[3] = (nap_spacing_t){.chips = NAP_SPACING_CHIPS,
                                  .samples = NAP_SPACING_SAMPLES};

  u16 control;
  u8 prn = sid.sat - GPS_FIRST_PRN;

  /* PRN code */
  control = (prn << NAP_TRK_CONTROL_SAT_Pos) & NAP_TRK_CONTROL_SAT_Msk;
  /* RF frontend channel */
  control |= (sid_to_rf_frontend_channel(sid) << NAP_TRK_CONTROL_FRONTEND_Pos) &
             NAP_TRK_CONTROL_FRONTEND_Msk;
  /* Constellation and band for tracking */
  control |= (sid_to_nap_code(sid) << NAP_TRK_CONTROL_CODE_Pos) &
             NAP_TRK_CONTROL_CODE_Msk;

  t->CONTROL = control;

  /* We always start at zero code phase */
  t->CODE_INIT_INT = 0;
  t->CODE_INIT_FRAC = 0;
  t->CODE_INIT_G1 = sid_to_init_g1(sid);
  t->CODE_INIT_G2 = 0x3ff;

  /* Set correlator spacing */
  t->SPACING = (spacing_to_nap_offset(s->spacing[0]) <<
      NAP_TRK_SPACING_OFFSET0_Pos) |
      (spacing_to_nap_offset(s->spacing[1]) <<
      NAP_TRK_SPACING_OFFSET1_Pos) |
      (spacing_to_nap_offset(s->spacing[2]) <<
      NAP_TRK_SPACING_OFFSET2_Pos) |
      (spacing_to_nap_offset(s->spacing[3]) <<
      NAP_TRK_SPACING_OFFSET3_Pos);

  double code_phase_rate = (1.0 + carrier_freq / code_to_carr_freq(sid.code)) *
      code_to_chip_rate(sid.code);

  s->init = true;
  s->sid = sid;
  s->code_phase_rate[0] = code_phase_rate;
  s->code_phase_rate[1] = code_phase_rate;

  nap_track_update(channel, carrier_freq, code_phase_rate,
      chips_to_correlate, 0);

  u32 length = t->LENGTH;

  if ((length < NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MIN_MS)) ||
      (length > NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MAX_MS))) {
    log_warn_sid(s->sid,
                 "Wrong NAP init correlation length: "
                 "(%" PRIu32 ", %f, %lf %" PRIu32 ")",
                 length, carrier_freq, code_phase_rate, chips_to_correlate);
  }

  /* Spacing between VE and P correlators */
  u16 prompt_offset = s->spacing[0].samples + s->spacing[1].samples +
      round((s->spacing[0].chips + s->spacing[1].chips) *
      calc_samples_per_chip(code_phase_rate));

  /* Adjust first integration length due to correlator spacing
   * (+ first period is one sample short) */
  length += prompt_offset + 1;
  t->LENGTH = length;

  s->steps |= STEP1;

  /* Set to start on the timing strobe */
  NAP->TRK_CONTROL |= (1 << channel);

  s->steps |= STEP2;

  COMPILER_BARRIER();

  s->steps |= STEP3;

  /* Set up timing compare */
  u32 tc_req;
  while (1) {
    chSysLock();
    s->steps |= STEP4;
    tc_req = NAP->TIMING_COUNT + TIMING_COMPARE_DELTA_MIN;

    double cp = propagate_code_phase(code_phase, carrier_freq,
                                     tc_req - ref_timing_count, sid.code);

    /* Contrive for the timing strobe to occur at or close to a PRN edge
     * (code phase = 0) */
    tc_req += round((code_to_chip_count(sid.code) - cp) *
        calc_samples_per_chip(code_phase_rate));

    /* Correct timing count for correlator spacing */
    tc_req -= prompt_offset;

    s->steps |= STEP5;
    NAP->TRK_TIMING_COMPARE = tc_req;
    s->steps |= STEP6;
    chSysUnlock();

    s->steps |= STEP7;

    if (tc_req - NAP->TRK_COMPARE_SNAPSHOT <= (u32)TIMING_COMPARE_DELTA_MAX) {
      s->steps |= STEP8;
      break;
    } else {
      s->steps |= STEP9;
    }
  }

  s->steps |= STEP10;

  /* Sleep until compare match */
  s32 tc_delta;
  while ((tc_delta = tc_req - NAP->TIMING_COUNT) >= 0) {
    systime_t sleep_time = floor(CH_CFG_ST_FREQUENCY * tc_delta /
        NAP_TRACK_SAMPLE_RATE_Hz);

    s->steps |= STEP11;
    /* The next system tick will always occur less than the nominal tick period
     * in the future, so sleep for an extra tick. */
    chThdSleep(1 + sleep_time / 2);
    s->steps |= STEP12;
  }

  s->steps |= STEP13;

  /* Revert length adjustment for future integrations after channel started */
  length = t->LENGTH;
  t->LENGTH -= prompt_offset + 1;
  s->steps |= STEP14;
  assert(t->LENGTH < length);   /* check for overflow */
  s->init = false;
  s->steps |= STEP15;
}

void nap_track_update(u8 channel, double carrier_freq,
                      double code_phase_rate, u32 chips_to_correlate,
                      u8 corr_spacing)
{
  (void)corr_spacing; /* This is always written as 0, for now */

  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];

  s->code_phase_rate[1] = s->code_phase_rate[0];
  s->code_phase_rate[0] = code_phase_rate;

  u32 code_phase_frac = 0;
  if (!s->init) {
    code_phase_frac = t->CODE_PHASE_FRAC + t->LENGTH * t->CODE_PINC;
  }

  u32 cp_rate_units = round(code_phase_rate *
      NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ);

  t->CODE_PINC = cp_rate_units;
  u32 length = calc_length_samples(chips_to_correlate, code_phase_frac,
                                   cp_rate_units);
  t->LENGTH = length;
  if ((length < NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MIN_MS)) ||
      (length > NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MAX_MS))) {
    log_warn_sid(s->sid, "Wrong NAP correlation length: "
          "(%d %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %lf)",
          (int)s->init, chips_to_correlate, code_phase_frac, cp_rate_units,
          length, code_phase_rate);
  }

  t->CARR_PINC = round(-carrier_freq * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
}

void nap_track_read_results(u8 channel,
                            u32* count_snapshot, corr_t corrs[],
                            double *code_phase_prompt,
                            double *carrier_phase)
{
  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];

  u32 ovf = (t->STATUS & NAP_TRK_STATUS_OVF_Msk) >> NAP_TRK_STATUS_OVF_Pos;
  if (ovf) {
    log_warn_sid(s->sid,
                 "Track correlator overflow 0x%04X on channel %d",
                 ovf, channel);
  }

  /* map corr registers by following way:
  * VE: CORR[0] -> corrs[3], E: CORR[1] -> corrs[0], P: CORR[2] -> corrs[1],
  * L: CORR[3] -> corrs[2], VL: CORR[4] -> corrs[4]
  * This is needed to use track_gps_l1ca.c for both Piksi v2 and v3 */
  corrs[0].I = t->CORR[1].I >> 8; corrs[0].Q = t->CORR[1].Q >> 8;
  corrs[1].I = t->CORR[2].I >> 8; corrs[1].Q = t->CORR[2].Q >> 8;
  corrs[2].I = t->CORR[3].I >> 8; corrs[2].Q = t->CORR[3].Q >> 8;
  corrs[3].I = t->CORR[0].I >> 8; corrs[3].Q = t->CORR[0].Q >> 8;
  corrs[4].I = t->CORR[4].I >> 8; corrs[4].Q = t->CORR[4].Q >> 8;

  u64 nap_code_phase = ((u64)t->CODE_PHASE_INT << 32) | t->CODE_PHASE_FRAC;
  s64 nap_carr_phase = ((s64)t->CARR_PHASE_INT << 32) | t->CARR_PHASE_FRAC;

  *count_snapshot = t->TIMING_SNAPSHOT;

  *carrier_phase = (double)-nap_carr_phase /
      NAP_TRACK_CARRIER_PHASE_UNITS_PER_CYCLE;

  /* Spacing between VE and P correlators */
  double prompt_offset = s->spacing[0].chips + s->spacing[1].chips +
      (s->spacing[0].samples + s->spacing[1].samples) /
      calc_samples_per_chip(s->code_phase_rate[1]);

  /* Correct code phase with spacing between VE and P correlators */
  *code_phase_prompt = (double)nap_code_phase /
      NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP - prompt_offset;

  if (*code_phase_prompt < 0) {
    *code_phase_prompt += code_to_chip_count(s->sid.code);
  }

  if (0 == corrs[1].I && 0 == corrs[1].Q && s->first_interrupt) {
    log_info_sid(s->sid,
                 "ZeroIQ:%d %d %" PRIx16 " %" PRIx32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32,
                 (int)s->init, (int)s->first_interrupt, s->steps, NAP->TRK_CONTROL, t->LENGTH,
                 NAP->TRK_TIMING_COMPARE, NAP->TRK_COMPARE_SNAPSHOT, NAP->TIMING_COUNT, t->TIMING_SNAPSHOT);
    log_info_sid(s->sid,
                 "ZeroIQ2:%d %d %" PRIu32 " %f %f %" PRIu32,
                 (int)channel, (int)s->channel, s->ref_timing_count,
                 s->carrier_freq, s->code_phase, s->chips_to_correlate);
  }
  s->first_interrupt = false;
}

void nap_track_disable(u8 channel)
{
  NAP->TRK_CONTROL &= ~(1 << channel);
}
