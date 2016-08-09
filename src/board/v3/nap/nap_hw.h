/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_REGS_H
#define SWIFTNAV_NAP_REGS_H

#include <libswiftnav/config.h>
#include <stdint.h>

/** Max number of tracking channels NAP configuration will be built with. */
#define NAP_MAX_N_TRACK_CHANNELS     (MAX_CHANNELS + 1)

typedef struct {
  volatile uint32_t STATUS;
  volatile uint32_t CONTROL;
  const volatile uint32_t START_SNAPSHOT;
  volatile uint32_t LENGTH;
  volatile uint32_t SPACING;
  volatile uint32_t CODE_INIT_INT;
  volatile uint32_t CODE_INIT_FRAC;
  volatile uint32_t CODE_INIT_G1;
  volatile uint32_t CODE_INIT_G2;
  volatile int32_t  CARR_PINC;
  volatile uint32_t CODE_PINC;
  const volatile int32_t  CARR_PHASE_INT;
  const volatile uint32_t CARR_PHASE_FRAC;
  const volatile uint32_t CODE_PHASE_INT;
  const volatile uint32_t CODE_PHASE_FRAC;
  struct {
    const volatile int32_t I;
    const volatile int32_t Q;
  } CORR[5];
} nap_trk_regs_t;

/* Registers */
typedef struct {
  volatile uint32_t STATUS;
  volatile uint32_t CONTROL;
  volatile uint32_t VERSION;
  volatile uint32_t IRQ;
  volatile uint32_t IRQ_ERROR;
  volatile uint32_t TIMING_COUNT;
  volatile uint32_t ACQ_STATUS;
  volatile uint32_t ACQ_CONTROL;
  volatile uint32_t ACQ_AXI_ATTRIBUTES;
  volatile uint32_t ACQ_TIMING_COMPARE;
  volatile uint32_t ACQ_TIMING_SNAPSHOT;
  volatile uint32_t ACQ_START_SNAPSHOT;
  volatile uint32_t ACQ_FFT_CONFIG;
  volatile int32_t  ACQ_PINC;
  volatile uint32_t ACQ_PEAK_MAGSQ;
  volatile uint32_t ACQ_SUM_MAGSQ;
  volatile uint32_t TRK_CONTROL;
  volatile uint32_t TRK_IRQ;
  volatile uint32_t TRK_IRQ_ERROR;
  volatile uint32_t TRK_TIMING_COMPARE;
  volatile uint32_t TRK_TIMING_SNAPSHOT;
  volatile uint32_t PPS_CONTROL;
  volatile uint32_t PPS_TIMING_COMPARE;
  volatile uint32_t EVENT_TIMING_SNAPSHOT;
  nap_trk_regs_t TRK_CH[NAP_MAX_N_TRACK_CHANNELS];
} nap_t;

/* Bitfields */
#define NAP_STATUS_AUTH_LOCKED_Pos (0U)
#define NAP_STATUS_AUTH_LOCKED_Msk (0x1U << NAP_STATUS_AUTH_LOCKED_Pos)

#define NAP_STATUS_TRACKING_CH_Pos (1U)
#define NAP_STATUS_TRACKING_CH_Msk (0x3FU << NAP_STATUS_TRACKING_CH_Pos)

#define NAP_CONTROL_VERSION_ADDR_Pos (0U)
#define NAP_CONTROL_VERSION_ADDR_Msk (0xFU << NAP_CONTROL_VERSION_ADDR_Pos)

#define NAP_CONTROL_KEY_ADDR_Pos (20U)
#define NAP_CONTROL_KEY_ADDR_Msk (0xFU << NAP_CONTROL_KEY_ADDR_Pos)

#define NAP_CONTROL_KEY_BYTE_Pos (24U)
#define NAP_CONTROL_KEY_BYTE_Msk (0xFFU << NAP_CONTROL_KEY_BYTE_Pos)

#define NAP_ACQ_STATUS_PEAK_INDEX_Pos (4U)
#define NAP_ACQ_STATUS_PEAK_INDEX_Msk (0x7FFFU << NAP_ACQ_STATUS_PEAK_INDEX_Pos)

#define NAP_ACQ_STATUS_PEAK_MAGSQ_OVF_Pos (2U)
#define NAP_ACQ_STATUS_PEAK_MAGSQ_OVF_Msk (0x1U << NAP_ACQ_STATUS_PEAK_MAGSQ_OVF_Pos)

#define NAP_ACQ_STATUS_SUM_MAGSQ_OVF_Pos (3U)
#define NAP_ACQ_STATUS_SUM_MAGSQ_OVF_Msk (0x1U << NAP_ACQ_STATUS_SUM_MAGSQ_OVF_Pos)

#define NAP_ACQ_CONTROL_DMA_INPUT_Pos (3U)
#define NAP_ACQ_CONTROL_DMA_INPUT_Msk (0x1U << NAP_ACQ_CONTROL_DMA_INPUT_Pos)
#define NAP_ACQ_CONTROL_DMA_INPUT_FFT (0U)
#define NAP_ACQ_CONTROL_DMA_INPUT_SAMPLE_GRABBER (1U)

#define NAP_ACQ_CONTROL_FFT_INPUT_Pos (4U)
#define NAP_ACQ_CONTROL_FFT_INPUT_Msk (0x1U << NAP_ACQ_CONTROL_FFT_INPUT_Pos)
#define NAP_ACQ_CONTROL_FFT_INPUT_DMA (0U)
#define NAP_ACQ_CONTROL_FFT_INPUT_FRONTEND (1U)

#define NAP_ACQ_CONTROL_FRONTEND_Pos (0U)
#define NAP_ACQ_CONTROL_FRONTEND_Msk (0x7U << NAP_ACQ_CONTROL_FRONTEND_Pos)

#define NAP_ACQ_CONTROL_LENGTH_Pos (7U)
#define NAP_ACQ_CONTROL_LENGTH_Msk (0xFFFFFU << NAP_ACQ_CONTROL_LENGTH_Pos)

#define NAP_ACQ_CONTROL_PEAK_SEARCH_Pos (5U)
#define NAP_ACQ_CONTROL_PEAK_SEARCH_Msk (0x1U << NAP_ACQ_CONTROL_PEAK_SEARCH_Pos)
#define NAP_ACQ_CONTROL_PEAK_SEARCH (1U)

#define NAP_ACQ_AXI_ATTRIBUTES_ARCACHE_Pos (0U)
#define NAP_ACQ_AXI_ATTRIBUTES_ARCACHE_Msk (0xFU << NAP_ACQ_AXI_ATTRIBUTES_ARCACHE_Pos)

#define NAP_ACQ_AXI_ATTRIBUTES_AWCACHE_Pos (4U)
#define NAP_ACQ_AXI_ATTRIBUTES_AWCACHE_Msk (0xFU << NAP_ACQ_AXI_ATTRIBUTES_AWCACHE_Pos)

#define NAP_ACQ_AXI_ATTRIBUTES_ARUSER_Pos (8U)
#define NAP_ACQ_AXI_ATTRIBUTES_ARUSER_Msk (0x1FU << NAP_ACQ_AXI_ATTRIBUTES_ARUSER_Pos)

#define NAP_ACQ_AXI_ATTRIBUTES_AWUSER_Pos (13U)
#define NAP_ACQ_AXI_ATTRIBUTES_AWUSER_Msk (0x1FU << NAP_ACQ_AXI_ATTRIBUTES_AWUSER_Pos)

#define NAP_ACQ_FFT_CONFIG_DIR_Pos (0U)
#define NAP_ACQ_FFT_CONFIG_DIR_Msk (0x1U << NAP_ACQ_FFT_CONFIG_DIR_Pos)

#define NAP_ACQ_FFT_CONFIG_SCALE_Pos (1U)
#define NAP_ACQ_FFT_CONFIG_SCALE_Msk (0x3FFFFFFFU << NAP_ACQ_FFT_CONFIG_SCALE_Pos)

#define NAP_TRK_STATUS_RUNNING_Pos (0U)
#define NAP_TRK_STATUS_RUNNING_Msk (0x1U << NAP_TRK_STATUS_RUNNING_Pos)

#define NAP_TRK_STATUS_OVF_Pos (1U)
#define NAP_TRK_STATUS_OVF_Msk (0x3FFU << NAP_TRK_STATUS_OVF_Pos)

#define NAP_TRK_SPACING_OUTER_Pos (0U)
#define NAP_TRK_SPACING_OUTER_Msk (0xFFFFU << NAP_TRK_SPACING_OUTER_Pos)

#define NAP_TRK_SPACING_INNER_Pos (16U)
#define NAP_TRK_SPACING_INNER_Msk (0xFFFFU << NAP_TRK_SPACING_INNER_Pos)

#define NAP_TRK_CONTROL_FRONTEND_Pos (0U)
#define NAP_TRK_CONTROL_FRONTEND_Msk (0x7U << NAP_TRK_CONTROL_FRONTEND_Pos)

#define NAP_TRK_CONTROL_SAT_Pos (3U)
#define NAP_TRK_CONTROL_SAT_Msk (0x1FU << NAP_TRK_CONTROL_SAT_Pos)

#define NAP_TRK_CONTROL_CODE_Pos (9U)
#define NAP_TRK_CONTROL_CODE_Msk (0x3U << NAP_TRK_CONTROL_CODE_Pos)

#define NAP_TRK_STATUS_RUNNING (1 << 31)

/* NAP RF fronend channel ID */
enum {
  NAP_RF_FRONTEND_CHANNEL_1 = 0, /* GPS L1 */
  NAP_RF_FRONTEND_CHANNEL_2,     /* Glonass L1 */
  NAP_RF_FRONTEND_CHANNEL_3,     /* Glonass L2 */
  NAP_RF_FRONTEND_CHANNEL_4      /* GPS L2 */
};

/* The values of NAP_TRKx_CONTROL::CODE field */
enum {
  NAP_CODE_GPS_L1CA_SBAS_L1CA = 0, /* GPS L1C/A, SBAS L1CA */
  NAP_CODE_GPS_L2CM                /* GPS L2CM */
};

/* Instances */
#define NAP ((nap_t *)0x43C00000)

#endif /* SWIFTNAV_NAP_REGS_H */
