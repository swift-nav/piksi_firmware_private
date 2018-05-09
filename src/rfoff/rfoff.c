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

#include "rfoff.h"
#include <assert.h>
#include <libswiftnav/logging.h>
#include <math.h>
#include "main.h"

/* RF off detection is run with this rate [ms] */
#define RFOFF_INT_MS 10

#define RFOFF_COUNTDOWN_MS 200

static u16 snr2tau3(float snr, u16 tau3_prev_ms) {
  assert(snr >= 0);
  /* clang-format off */
  static struct {
    float snr_min;
    float snr_max;
    u16 tau3_ms;
  } lookup[] = {
    {25., INFINITY, RFOFF_INT_MS},
    {0, 7., 200}
  };
  /* clang-format on */

  u16 tau3_ms = tau3_prev_ms;
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
     Project 1000ms three time constants for the noise IIR filter */
  double alpha = 1 - exp(-1. / ((3000. / 3.) / RFOFF_INT_MS));
  running_stats_init(&self->noise, alpha);

  self->signal_tau3_ms = RFOFF_INT_MS;
  alpha = 1 - exp(-1. / ((self->signal_tau3_ms / 3.) / RFOFF_INT_MS));
  running_stats_init(&self->signal, alpha);
}

bool rfoff_detected(rfoff_t *self,
                    const u8 int_ms,
                    const corr_t *ve,
                    const corr_t *e,
                    const corr_t *p,
                    const corr_t *l) {
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
  u16 signal_tau3_ms = snr2tau3(snr, self->signal_tau3_ms);
  if (signal_tau3_ms != self->signal_tau3_ms) {
    double alpha = 1 - exp(-1. / ((signal_tau3_ms / 3.) / RFOFF_INT_MS));
    self->signal_tau3_ms = signal_tau3_ms;
    running_stats_init(&self->signal, alpha);
  }

  bool rfoff = signal_mean < (noise_mean + 0.25 * noise_std);

  if (rfoff) {
    self->rfoff = true;
    self->rfoff_countdown = RFOFF_COUNTDOWN_MS;
  } else if (0 == self->rfoff_countdown) {
    self->rfoff = false;
  }

  return self->rfoff;
}
