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

#include "nap_constants.h"
#include "nap_hw.h"
#include "nap/nap_common.h"
#include "nap/track_channel.h"
#include "track.h"
#include "timing.h"
#include "main.h"
#include "timing.h"

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

/** Internal tracking channel state */
static struct nap_ch_state {
  bool init;                   /**< Initializing channel. */
  me_gnss_signal_t mesid;      /**< Channel ME sid */
  nap_spacing_t spacing[4];    /**< Correlator spacing. */
  double code_phase_rate[2];   /**< Code phase rates. */
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

/** Looks-up RF frontend channel for the given ME signal ID.
 * \param mesid ME signal ID.
 * \return RF front-end channel number.
 */
static u8 mesid_to_rf_frontend_channel(me_gnss_signal_t mesid)
{
  u8 ret = ~0;
  switch (mesid.code) {
  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    ret = NAP_RF_FRONTEND_CHANNEL_1;
    break;
  case CODE_GLO_L1CA:
    ret = NAP_RF_FRONTEND_CHANNEL_2;
    break;
  case CODE_GLO_L2CA:
    ret = NAP_RF_FRONTEND_CHANNEL_3;
    break;
  case CODE_GPS_L2CM:
  case CODE_GPS_L2CL:
    ret = NAP_RF_FRONTEND_CHANNEL_4;
    break;
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

/** Looks-up NAP constellation and band code for the given ME signal ID.
 * \param mesid ME signal ID.
 * \return NAP constellation and band code.
 */
static u8 mesid_to_nap_code(const me_gnss_signal_t mesid)
{
  u8 ret = ~0;
  switch (mesid.code) {
  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    ret = NAP_CODE_GPS_L1CA_SBAS_L1CA;
    break;
  case CODE_GPS_L2CM:
    ret = NAP_CODE_GPS_L2CM;
    break;
  case CODE_GPS_L2CL:
    ret = NAP_CODE_GPS_L2CL;
    break;
  case CODE_GLO_L1CA:
  case CODE_GLO_L2CA:
    ret = NAP_CODE_GLO_L1CA_GLO_L2CA;
    break;
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

void nap_track_init(u8 channel,
                    const me_gnss_signal_t mesid,
                    u32 ref_timing_count,
                    float doppler_freq_hz,
                    double code_phase,
                    u32 chips_to_correlate)
{
  assert((mesid.code == CODE_GPS_L1CA) ||
         (mesid.code == CODE_GPS_L2CM) ||
         (mesid.code == CODE_GPS_L2CL) ||
         (mesid.code == CODE_GLO_L1CA) ||
         (mesid.code == CODE_GLO_L2CA));

  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];

  s->mesid = mesid;
  /* Delay L2CL code phase by 1 chip to accommodate zero in the L2CM slot */
  /* Initial correlation length for L2CL is thus 1 chip shorter,
   * since first L2CM chip is skipped */
  if (mesid.code == CODE_GPS_L2CL) {
    code_phase -= 1.0f;
    if (code_phase < 0.0f) {
      code_phase += GPS_L2CL_CHIPS_NUM;
    }
    chips_to_correlate -= 1;
  }

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

  if (CODE_GLO_L1CA == mesid.code) {
    /* NAP_TRK_CONTROL_SAT field is not used for GLONASS, so be it 0. */
    control = 0;
  } else {
    u8 prn = mesid.sat - GPS_FIRST_PRN;
    control = (prn << NAP_TRK_CONTROL_SAT_Pos) & NAP_TRK_CONTROL_SAT_Msk;
  }
  /* RF frontend channel */
  control |= (mesid_to_rf_frontend_channel(mesid) << NAP_TRK_CONTROL_FRONTEND_Pos) &
             NAP_TRK_CONTROL_FRONTEND_Msk;
  /* Constellation and band for tracking */
  control |= (mesid_to_nap_code(mesid) << NAP_TRK_CONTROL_CODE_Pos) &
             NAP_TRK_CONTROL_CODE_Msk;

  t->CONTROL = control;

  /* Set correlator spacing */
  t->SPACING = (spacing_to_nap_offset(s->spacing[0]) <<
      NAP_TRK_SPACING_OFFSET0_Pos) |
      (spacing_to_nap_offset(s->spacing[1]) <<
      NAP_TRK_SPACING_OFFSET1_Pos) |
      (spacing_to_nap_offset(s->spacing[2]) <<
      NAP_TRK_SPACING_OFFSET2_Pos) |
      (spacing_to_nap_offset(s->spacing[3]) <<
      NAP_TRK_SPACING_OFFSET3_Pos);

  double carrier_freq_hz = code_to_carr_freq(mesid.code);
  double code_phase_rate = (1.0 + doppler_freq_hz / carrier_freq_hz) *
      code_to_chip_rate(mesid.code);

  s->init = true;
  s->code_phase_rate[0] = code_phase_rate;
  s->code_phase_rate[1] = code_phase_rate;

  nap_track_update(channel, mesid, doppler_freq_hz, code_phase_rate,
      chips_to_correlate, 0);

  u32 length = t->LENGTH;

  if ((length < NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MIN_MS)) ||
      (length > NAP_MS_2_SAMPLES(NAP_CORR_LENGTH_MAX_MS))) {
    log_warn_mesid(s->mesid,
                   "Wrong NAP init correlation length: "
                   "(%" PRIu32 ", %f, %lf %" PRIu32 ")",
                   length, doppler_freq_hz,
                   code_phase_rate, chips_to_correlate);
  }

  /* Spacing between VE and P correlators */
  u16 prompt_offset = s->spacing[0].samples + s->spacing[1].samples +
      round((s->spacing[0].chips + s->spacing[1].chips) *
      calc_samples_per_chip(code_phase_rate));

  /* Adjust first integration length due to correlator spacing
   * (+ first period is one sample short) */
  length += prompt_offset + 1;
  t->LENGTH = length;

  u64 profiling_begin = nap_timing_count();

  /* Set to start on the timing strobe */
  NAP->TRK_CONTROL |= (1 << channel);

  COMPILER_BARRIER();

  /* Set up timing compare */
  u32 tc_req;
  while (1) {
    u32 diff;
    chSysLock();
    u32 timing_count = NAP->TIMING_COUNT;
    tc_req = timing_count + TIMING_COMPARE_DELTA_MIN;

    diff = tc_req - ref_timing_count;
    double cp = propagate_code_phase(mesid,
                                     code_phase,
                                     doppler_freq_hz,
                                     diff);
    u8 index = 0;
    /* Contrive for the timing strobe to occur at
     * or close to next PRN start point */
    if (mesid.code == CODE_GPS_L2CL) {
      u32 code_length = code_to_chip_count(mesid.code);
      u32 chips = code_length * GPS_L2CL_PRN_START_INTERVAL_MS
                              / GPS_L2CL_PRN_PERIOD_MS;
      u8 cp_start = 0;
      double tmp = ceil(cp / chips);
      if (tmp >= 0 && tmp < GPS_L2CL_PRN_START_POINTS) {
        cp_start = tmp;
      }
      index = (cp_start == GPS_L2CL_PRN_START_POINTS) ? 0 : cp_start;
      tc_req += round((cp_start * chips - cp)
              * calc_samples_per_chip(code_phase_rate));
      t->CODE_INIT_INT = index * chips;
    } else {
      tc_req += round((code_to_chip_count(mesid.code) - cp)
              * calc_samples_per_chip(code_phase_rate));
      t->CODE_INIT_INT = 0;
    }
    t->CODE_INIT_FRAC = 0;
    t->CODE_INIT_G1 = mesid_to_init_g1(mesid, index);
    t->CODE_INIT_G2 = mesid_to_init_g2(mesid);

    /* Correct timing count for correlator spacing */
    tc_req -= prompt_offset;

    NAP->TRK_TIMING_COMPARE = tc_req;
    chSysUnlock();

    if (tc_req - NAP->TRK_COMPARE_SNAPSHOT <= (u32)TIMING_COMPARE_DELTA_MAX) {
      break;
    }
  }

  /* Sleep until compare match */
  s32 tc_delta;
  while ((tc_delta = tc_req - NAP->TIMING_COUNT) >= 0) {
    systime_t sleep_time = floor(CH_CFG_ST_FREQUENCY * tc_delta /
        NAP_TRACK_SAMPLE_RATE_Hz);

    /* The next system tick will always occur less than the nominal tick period
     * in the future, so sleep for an extra tick. */
    chThdSleep(1 + sleep_time / 2);
  }

  /* Revert length adjustment for future integrations after channel started */
  length = t->LENGTH;
  t->LENGTH -= prompt_offset + 1;
  u32 length_reg_val = t->LENGTH;
  /* check for underflow */
  if (length <= length_reg_val) {
    u64 profiling_end = nap_timing_count();
    log_error_mesid(mesid,
                    "LENGTH: %" PRIu32 " length: %" PRIu32
                    " prompt_offset: %" PRIu16
                    " delay_us: %.3lf",
                    length_reg_val, length, prompt_offset,
                    (profiling_end - profiling_begin) * RX_DT_NOMINAL * 1e6);
  }
  /* Future integrations for L2CL are 1 chip longer */
  if (mesid.code == CODE_GPS_L2CL) {
    t->LENGTH += calc_samples_per_chip(code_phase_rate);
  }
  s->init = false;
}

void nap_track_update(u8 channel,
                      const me_gnss_signal_t mesid,
                      double doppler_freq_hz,
                      double code_phase_rate,
                      u32 chips_to_correlate,
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
    log_error_mesid(s->mesid, "Wrong NAP correlation length: "
                    "(%d %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %lf)",
                    (int)s->init, chips_to_correlate, code_phase_frac,
                    cp_rate_units, length, code_phase_rate);
  }

  double carrier_freq_hz = 0;

  if (CODE_GLO_L1CA == mesid.code) {
    carrier_freq_hz = -(mesid.sat - 8) * GLO_L1_DELTA_HZ;
  } else if (CODE_GLO_L2CA == mesid.code) {
    carrier_freq_hz = -(mesid.sat - 8) * GLO_L2_DELTA_HZ;
  }

  carrier_freq_hz = carrier_freq_hz - doppler_freq_hz;

  t->CARR_PINC = round(carrier_freq_hz * NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
}

void nap_track_read_results(u8 channel,
                            u32* count_snapshot,
                            corr_t corrs[],
                            double *code_phase_prompt,
                            double *carrier_phase)
{
  nap_trk_regs_t *t = &NAP->TRK_CH[channel];
  struct nap_ch_state *s = &nap_ch_state[channel];

  u32 ovf = (t->STATUS & NAP_TRK_STATUS_OVF_Msk) >> NAP_TRK_STATUS_OVF_Pos;
  if (ovf) {
    log_warn_mesid(s->mesid,
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
    *code_phase_prompt += code_to_chip_count(s->mesid.code);
  }
}

void nap_track_disable(u8 channel)
{
  NAP->TRK_CONTROL &= ~(1 << channel);
}
