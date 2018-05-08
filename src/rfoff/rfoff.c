/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <math.h>
#include "rfoff.h"
#include "main.h"
#include <libswiftnav/logging.h>

/* RF off detection is run with this rate [ms] */
#define RFOFF_INT_MS 20

#define RFOFF_COUNTDOWN_MS 100

static u8 snr2tau3(float snr, u8 tau3_prev_ms) {
  assert(snr >= 0);
  static struct {
    float snr_min;
    float snr_max;
    u8 tau3_ms;
  } lookup[] = {
    {25., INFINITY, 20},
    /* {10., 20., 40}, */
    /* {10, 15., 60}, */
    {0, 7., 200}
  };

  u8 tau3_ms = tau3_prev_ms;
  for (size_t i = 0; i < ARRAY_SIZE(lookup); i++) {
    if (snr < lookup[i].snr_min) {
      continue;
    }
    if (snr > lookup[i].snr_max) {
      continue;
    }
    tau3_ms = lookup[i].tau3_ms;
    break;
  }
  assert(tau3_ms > 0);

  return tau3_ms;
}

void rfoff_init(rfoff_t *self) {
  /* alpha is going to affect noise filtering
     Noise level is not expected to change rapidly.
     Project 1000ms time constant for the noise IIR filter */
  double alpha = 1 - exp(-1. / (1000. / RFOFF_INT_MS));
  running_stats_init(&self->noise, alpha);

  self->signal_tau3_ms = RFOFF_INT_MS;
  alpha = 1 - exp(-1. / ((self->signal_tau3_ms / 3.) / RFOFF_INT_MS));
  running_stats_init(&self->signal, alpha);
}

bool rfoff_detected(rfoff_t *self,
                   me_gnss_signal_t mesid,
                   double cn0,
                   bool locked,
                   const u8 int_ms,
                   const corr_t *ve,
                   const corr_t *e,
                   const corr_t *p,
                   const corr_t *l) {
  (void)mesid;
  (void)cn0;
  (void)locked;
  double noise = ve->I * ve->I + ve->Q * ve->Q;
  self->noise_now += noise;

  double signal_e = e->I * e->I + e->Q * e->Q;
  double signal_p = p->I * p->I + p->Q * p->Q;
  double signal_l = l->I * l->I + l->Q * l->Q;

  double signal_max = MAX(signal_e, signal_p);
  self->signal_now += MAX(signal_max, signal_l);

  self->int_ms += int_ms;

  if (self->rfoff_countdown > int_ms) {
    self->rfoff_countdown -= int_ms;
  } else {
    self->rfoff_countdown = 0;
  }

  if (self->int_ms < RFOFF_INT_MS) {
    return self->rfoff;
  }

  self->noise_now /= self->int_ms;
  self->signal_now /= self->int_ms;
  self->int_ms = 0;

  running_stats_update(&self->noise, self->noise_now);
  self->noise_now = 0;
  running_stats_update(&self->signal, self->signal_now);
  self->signal_now = 0;

  double noise_std = running_stats_get_std(&self->noise);
  double noise_mean = running_stats_get_mean(&self->noise);
  double signal_mean = running_stats_get_mean(&self->signal);

  double snr = signal_mean / noise_mean;
  u8 signal_tau3_ms = snr2tau3(snr, self->signal_tau3_ms);
  if (signal_tau3_ms != self->signal_tau3_ms) {
    double alpha = 1 - exp(-1. / ((signal_tau3_ms / 3.) / RFOFF_INT_MS));
    /* log_info_mesid(mesid, "rfoff retune: snr=%.0f tau3: %d->%d alpha=%f", */
    /*                snr, */
    /*                (int)self->signal_tau3_ms, (int)signal_tau3_ms, alpha); */
    self->signal_tau3_ms = signal_tau3_ms;
    running_stats_init(&self->signal, alpha);
  }

  bool rfoff = signal_mean < (noise_mean + 2 * noise_std);

  /* if ((piksi_systime_elapsed_since_s(&self->last_systime) > 3.)/\*  || *\/ */
  /*     /\* self->rfoff != rfoff *\/) { */
  /*   log_info_mesid(mesid, "rfoff: cn0=%.0f,noise=%.1f(%.1f) " */
  /*                         "signal=%.1f snr=%.1f ld=%d off=%d", */
  /*            cn0, noise_std, noise_mean, */
  /*            signal_mean, snr, (int)locked, (int)rfoff); */
  /*   piksi_systime_get(&self->last_systime); */
  /* } */

  if (rfoff) {
    self->rfoff = true;
    self->rfoff_countdown = RFOFF_COUNTDOWN_MS;
  } else if (0 == self->rfoff_countdown) {
    self->rfoff = false;
  }

  return self->rfoff;
}
