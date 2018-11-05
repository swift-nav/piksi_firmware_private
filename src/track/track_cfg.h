/*
 * Copyright (C) 2011-2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Bit synchronization and data decoding */
#define TPF_BIT_PILOT ((u32)1 << 1) /* data comes in the fifth correlator */
#define TPF_BSYNC_SET ((u32)1 << 2)
#define TPF_BSYNC_ADD ((u32)1 << 3)
#define TPF_BSYNC_UPD ((u32)1 << 4)
#define TPF_BSYNC_INV ((u32)1 << 5) /* invert symbol data */

/* C/N0 estimator control */
#define TPF_CN0_SET ((u32)1 << 6)
#define TPF_CN0_ADD ((u32)1 << 7)
#define TPF_CN0_USE ((u32)1 << 8)

/* FLL control */
#define TPF_FLL_SET ((u32)1 << 9)
#define TPF_FLL_ADD ((u32)1 << 10)
#define TPF_FLL_USE ((u32)1 << 11)
#define TPF_FLL_HALFQ ((u32)1 << 12)

/* correlators control */
#define TPF_EPL_INV ((u32)1 << 13) /* invert correlators */
#define TPF_EPL_SET ((u32)1 << 14)
#define TPF_EPL_ADD ((u32)1 << 15)
#define TPF_EPL_USE ((u32)1 << 16)

/* Phase lock detector control */
#define TPF_PLD_SET ((u32)1 << 22)
#define TPF_PLD_ADD ((u32)1 << 23)
#define TPF_PLD_USE ((u32)1 << 24)

#define TPF_FPLL_RUN ((u32)1 << 25)

/** GPS L1 C/A cross-correlation frequency step [hz] */
#define L1CA_XCORR_FREQ_STEP 1000.f
/** GPS L1 C/A CN0 threshold for whitelisting [dB-Hz] */
#define L1CA_XCORR_WHITELIST_THRESHOLD 40.f
/** GPS L2 CM CN0 threshold for whitelisting [dB-Hz] */
#define L2CM_XCORR_WHITELIST_THRESHOLD 27.f
/** GPS L1 C/A CN0 threshold for suspected xcorr [dB-Hz] */
#define XCORR_SUSPECT_THRESHOLD -15.f
/** GPS L1 C/A CN0 threshold for confirmed xcorr [dB-Hz] */
#define XCORR_CONFIRM_THRESHOLD -20.f
/** cross-correlation update rate [Hz] */
#define XCORR_UPDATE_RATE (SECS_MS / GPS_L1CA_BIT_LENGTH_MS)
/** Carrier phases within tolerance are declared equal. [cycles]
 *  Stable PLL remains within +-15 degree from correct phase.
 *  360 * 0.08 ~= 30 degrees
 */
#define CARRIER_PHASE_TOLERANCE 0.08f
/** counter for half-cycle ambiguity resolution */
#define CARRIER_PHASE_AMBIGUITY_COUNTER 50
/** handover should occur when code phase is near zero [chips] */
#define HANDOVER_CODE_PHASE_THRESHOLD 0.5

/** Initial C/N0 for confirmation [dB/Hz] */
#define TP_TRACKER_CN0_CONFIRM_DELTA (2.f)

/** C/N0 threshold long interval [ms] */
#define TRACK_CN0_THRES_COUNT_LONG 2000

/** C/N0 threshold short interval [ms] */
#define TRACK_CN0_THRES_COUNT_SHORT 100

/** C/N0 hysteresis threshold */
#define TRACK_CN0_HYSTERESIS_THRES_DBHZ (3.f)

/* The outliers are likely due to genuine acceleration if CN0 > 35.f */
#define TP_OUTLIERS_CN0_THRES_DBHZ 35.f

/** C/N0 threshold when we can't say if we are still tracking */
#define TP_HARD_CN0_DROP_THRESHOLD_DBHZ (18.f)

/** Default C/N0 threshold in dB/Hz for bit polarity ambiguity */
#define TP_DEFAULT_CN0_AMBIGUITY_THRESHOLD_DBHZ (30.f)
/** Default C/N0 threshold in dB/Hz for dropping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_DROP_THRESHOLD_DBHZ (32.2f)
/** C/N0 threshold for measurements use */
#define TP_DEFAULT_CN0_USE_THRESHOLD_DBHZ (27.f)

#define TL_BWT_MAX (18.f * 0.020f)

#define ADJ_CN0_MIN (20.0f)
#define ADJ_CN0_MAX (60.0f)

#define PLL_BW_MIN (10.0f)
#define PLL_BW_MAX (20.0f)

#define FLL_BW_MIN (1.5f)
#define FLL_BW_MAX (3.0f)
