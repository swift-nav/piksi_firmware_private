/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#define DEBUG 0
#include "cn0_est.h"
#include <libswiftnav/logging.h>
#include <math.h>

float cn0_estimate(track_cn0_state_t *data,
                   const corr_t* cs,
                   u8 int_ms)
{
//  (void)int_ms;
  float c = c_pwr(data,
                 (float) cs[1].I,
                 (float) cs[1].Q);

  float n0 = n0_pwr(data,
                 (float) cs[3].I,
                 (float) cs[3].Q);
  log_debug("signal: c=%f",c);
  log_debug("noise: n0=%f",n0);
  float loop_freq = 1000.f / int_ms;
  float cn0 = c * loop_freq / n0;
  cn0 = 10.f * log10f(cn0) + 20.f;
  log_debug("C/No=%f",cn0);
  return cn0;
}

void cn0_init(track_cn0_state_t *s, u8 int_ms, float cn0_0)
{
  (void)int_ms;
  (void)cn0_0;

  s->basic.I_prev = 0.f;
  s->basic.Q_prev = 0.f;
  s->basic.c = 0.f;

  s->noise.Q_abs = 0.f;
  s->noise.n0 = 0.f;
  s->noise.cnt = 200;
}

float c_pwr(track_cn0_state_t *s, float I, float Q)
{
  s->basic.c = I * I + Q * Q;
  return s->basic.c;
}

float n0_pwr(track_cn0_state_t *s, float I, float Q)
{
  (void)I;

  if (s->noise.cnt >= 1) {
    s->noise.Q_abs += fabsf(Q);
    s->noise.cnt--;
    if (s->noise.cnt == 0) {
      s->noise.Q_abs /= 200;
    }
  } else {
    /* (.0167f) -- cn0 est lpf alfa */
    float Q_abs = s->noise.Q_abs * (1 - (.0167f)) + (.0167f) * fabsf(Q);

    s->noise.Q_abs = Q_abs;

    s->noise.n0 = Q_abs * Q_abs;
  }

  return s->noise.n0;
}

void cn0_est_precompute()
{

}


