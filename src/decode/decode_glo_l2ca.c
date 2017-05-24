/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tpaakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_glo_l2ca.h"
#include "decode.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg_glo.h> /* For BIT_POLARITY_... constants */
#include <assert.h>
#include <string.h>

#include "ephemeris.h"
#include "track.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"
#include "shm.h"

/** GLO L2CA decoder data */
typedef struct {
  nav_msg_glo_t nav_msg;
} glo_l2ca_decoder_data_t;

static decoder_t glo_l2ca_decoders[NUM_GLO_L2CA_DECODERS];
static glo_l2ca_decoder_data_t glo_l2ca_decoder_data[ARRAY_SIZE(glo_l2ca_decoders)];

static void decoder_glo_l2ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_glo_l2ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_glo_l2ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_glo_l2ca = {
  .code =         CODE_GLO_L2CA,
  .init =         decoder_glo_l2ca_init,
  .disable =      decoder_glo_l2ca_disable,
  .process =      decoder_glo_l2ca_process,
  .decoders =     glo_l2ca_decoders,
  .num_decoders = ARRAY_SIZE(glo_l2ca_decoders)
};

static decoder_interface_list_element_t list_element_glo_l2ca = {
  .interface = &decoder_interface_glo_l2ca,
  .next = 0
};

void decode_glo_l2ca_register(void)
{
  for (u32 i = 0; i < ARRAY_SIZE(glo_l2ca_decoders); i++) {
    glo_l2ca_decoders[i].active = false;
    glo_l2ca_decoders[i].data = &glo_l2ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_glo_l2ca);
}

static void decoder_glo_l2ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data)
{
  (void)channel_info;
  glo_l2ca_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init_glo(&data->nav_msg);
}

static void decoder_glo_l2ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_glo_l2ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  glo_l2ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  s8 soft_bit;
  bool sensitivity_mode;
  while (tracking_channel_nav_bit_get(channel_info->tracking_channel,
                                      &soft_bit, &sensitivity_mode)) {
    (void)data;
  }
  return;
}
