/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tommi.paakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_gps_l2cl.h"
#include "decode.h"

#include <libswiftnav/logging.h>
#include <assert.h>
#include <string.h>

#include "track.h"

/** GPS L2 CL decoder data */
typedef struct {
} gps_l2cl_decoder_data_t;

static decoder_t gps_l2cl_decoders[NUM_GPS_L2CL_DECODERS];
static gps_l2cl_decoder_data_t gps_l2cl_decoder_data[ARRAY_SIZE(gps_l2cl_decoders)];

static void decoder_gps_l2cl_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_gps_l2cl_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_gps_l2cl_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gps_l2cl = {
  .code =         CODE_GPS_L2CL,
  .init =         decoder_gps_l2cl_init,
  .disable =      decoder_gps_l2cl_disable,
  .process =      decoder_gps_l2cl_process,
  .decoders =     gps_l2cl_decoders,
  .num_decoders = ARRAY_SIZE(gps_l2cl_decoders)
};

static decoder_interface_list_element_t list_element_gps_l2cl = {
  .interface = &decoder_interface_gps_l2cl,
  .next = 0
};

void decode_gps_l2cl_register(void)
{
  for (u32 i = 0; i < ARRAY_SIZE(gps_l2cl_decoders); i++) {
    gps_l2cl_decoders[i].active = false;
    gps_l2cl_decoders[i].data = &gps_l2cl_decoder_data[i];
  }

  decoder_interface_register(&list_element_gps_l2cl);
}

static void decoder_gps_l2cl_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_gps_l2cl_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_gps_l2cl_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  (void)decoder_data;

  /* Process incoming nav bits */
  s8 soft_bit;
  bool sensitivity_mode = true;
  while (tracking_channel_nav_bit_get(channel_info->tracking_channel,
                                      &soft_bit, &sensitivity_mode)) {
  }
}
