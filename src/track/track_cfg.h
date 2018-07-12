/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * Main tracking: PLL loop selection
 */

/* FLL-assisted PLL. FLL is first order and PLL is second order */
#define tl_pll2_state_t aided_tl_state_fll1_pll2_t
#define tl_pll2_init aided_tl_fll1_pll2_init
#define tl_pll2_retune aided_tl_fll1_pll2_retune
#define tl_pll2_update_fll aided_tl_fll1_pll2_update_fll
#define tl_pll2_update_dll aided_tl_fll1_pll2_update_dll
#define tl_pll2_adjust aided_tl_fll1_pll2_adjust
#define tl_pll2_get_dll_error aided_tl_fll1_pll2_get_dll_error
#define tl_pll2_discr_update aided_tl_fll1_pll2_discr_update
#define tl_pll2_get_rates aided_tl_fll1_pll2_get_rates

/*
 * 3rd order PLL loop selection.
 * The third order PLL loop selection is mutually exclusive from the three
 * available implementations:
 *
 * TRACK_PLL_MODE3_BL
 * PLL-assisted DLL. FLL and DLL are second order, PLL is third order
 * Note: Bilinear transform integrator implementation
 *
 * TRACK_PLL_MODE3_BC
 * PLL-assisted DLL. FLL and DLL are second order, PLL is third order
 * Note: Boxcar integrator implementation
 *
 * TRACK_PLL_MODE3_FLL
 * FLL-assisted PLL. FLL is second order and PLL is third order
 * Note: Bilinear transform integrator implementation
 *
 */
#define TRACK_PLL_MODE3_BL 1
#define TRACK_PLL_MODE3_BC 2
#define TRACK_PLL_MODE3_FLL 3

#ifndef TRACK_PLL_MODE3
#define TRACK_PLL_MODE3 TRACK_PLL_MODE3_FLL
#endif

#if TRACK_PLL_MODE3 == TRACK_PLL_MODE3_BL
#define tl_pll3_state_t aided_tl_state3_t
#define tl_pll3_init aided_tl_init3
#define tl_pll3_retune aided_tl_retune3
#define tl_pll3_update_fll aided_tl_update_fll3
#define tl_pll3_update_dll aided_tl_update_dll3
#define tl_pll3_adjust aided_tl_adjust3
#define tl_pll3_get_dll_error aided_tl_get_dll_error3
#define tl_pll3_discr_update aided_tl_discr_update3
#define tl_pll3_get_rates aided_tl_get_rates3
#elif TRACK_PLL_MODE3 == TRACK_PLL_MODE3_BC
#define tl_pll3_state_t aided_tl_state3b_t
#define tl_pll3_init aided_tl_init3b
#define tl_pll3_retune aided_tl_retune3b
#define tl_pll3_update_fll aided_tl_update_fll3b
#define tl_pll3_update_dll aided_tl_update_dll3b
#define tl_pll3_adjust aided_tl_adjust3b
#define tl_pll3_get_dll_error aided_tl_get_dll_error3b
#define tl_pll3_discr_update aided_tl_discr_update3b
#define tl_pll3_get_rates aided_tl_get_rates3b
#elif TRACK_PLL_MODE3 == TRACK_PLL_MODE3_FLL
#define tl_pll3_state_t aided_tl_state_fll2_pll3_t
#define tl_pll3_init aided_tl_fll2_pll3_init
#define tl_pll3_retune aided_tl_fll2_pll3_retune
#define tl_pll3_update_fll aided_tl_fll2_pll3_update_fll
#define tl_pll3_update_dll aided_tl_fll2_pll3_update_dll
#define tl_pll3_adjust aided_tl_fll2_pll3_adjust
#define tl_pll3_get_dll_error aided_tl_fll2_pll3_get_dll_error
#define tl_pll3_discr_update aided_tl_fll2_pll3_discr_update
#define tl_pll3_get_rates aided_tl_fll2_pll3_get_rates
#else
#error Unsupported 3rd order PLL Mode
#endif

/*
 * Main tracking: FLL loop selection
 */

/* FLL-assisted DLL. FLL is first order and DLL is second order */
#define tl_fll1_state_t aided_tl_state_fll1_t
#define tl_fll1_init aided_tl_fll1_init
#define tl_fll1_retune aided_tl_fll1_retune
#define tl_fll1_update_fll aided_tl_fll1_update_fll
#define tl_fll1_update_dll aided_tl_fll1_update_dll
#define tl_fll1_adjust aided_tl_fll1_adjust
#define tl_fll1_get_dll_error aided_tl_fll1_get_dll_error
#define tl_fll1_discr_update aided_tl_fll1_discr_update
#define tl_fll1_get_rates aided_tl_fll1_get_rates

/* FLL-assisted DLL. FLL and DLL are both second order */
#define tl_fll2_state_t aided_tl_state_fll2_t
#define tl_fll2_init aided_tl_fll2_init
#define tl_fll2_retune aided_tl_fll2_retune
#define tl_fll2_update_fll aided_tl_fll2_update_fll
#define tl_fll2_update_dll aided_tl_fll2_update_dll
#define tl_fll2_adjust aided_tl_fll2_adjust
#define tl_fll2_get_dll_error aided_tl_fll2_get_dll_error
#define tl_fll2_discr_update aided_tl_fll2_discr_update
#define tl_fll2_get_rates aided_tl_fll2_get_rates

/* Bit synchronization and data decoding */
#define TPF_BIT_PILOT ((u32)1 << 1) /* data comes in the fifth correlator */
#define TPF_BSYNC_SET ((u32)1 << 2)
#define TPF_BSYNC_ADD ((u32)1 << 3)
#define TPF_BSYNC_UPD ((u32)1 << 4)

/* C/N0 estimator control */
#define TPF_CN0_SET ((u32)1 << 5)
#define TPF_CN0_ADD ((u32)1 << 6)
#define TPF_CN0_USE ((u32)1 << 7)

/* FLL control */
#define TPF_FLL_SET ((u32)1 << 8)
#define TPF_FLL_ADD ((u32)1 << 9)
#define TPF_FLL_USE ((u32)1 << 11)
#define TPF_FLL_HALFQ ((u32)1 << 12)

/* correlators control */
#define TPF_EPL_INV ((u32)1 << 13) /* invert correlators */
#define TPF_EPL_SET ((u32)1 << 14)
#define TPF_EPL_ADD ((u32)1 << 15)
#define TPF_EPL_USE ((u32)1 << 16)

/* False lock detector control */
#define TPF_ALIAS_SET ((u32)1 << 18)
#define TPF_ALIAS_ADD ((u32)1 << 19)
#define TPF_ALIAS_1ST ((u32)1 << 20)
#define TPF_ALIAS_2ND ((u32)1 << 21)

/* Phase lock detector control */
#define TPF_PLD_SET ((u32)1 << 22)
#define TPF_PLD_ADD ((u32)1 << 23)
#define TPF_PLD_USE ((u32)1 << 24)

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

/** False lock detector filter interval in ms. */
#define TP_TRACKER_ALIAS_DURATION_MS (3000)
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

/** Correct the false lock if the absolute detected freq error is more than
    this value in [Hz]. In practice, observed alias frequencies were
    above 12Hz, so 10Hz is deemed a reasonable threshold. */
#define TRACK_ALIAS_THRESHOLD_HZ 10.f

/** C/N0 threshold when we can't say if we are still tracking */
#define TP_HARD_CN0_DROP_THRESHOLD_DBHZ (18.f)

/** Default C/N0 threshold in dB/Hz for bit polarity ambiguity */
#define TP_DEFAULT_CN0_AMBIGUITY_THRESHOLD_DBHZ (30.f)
/** Default C/N0 threshold in dB/Hz for dropping track (for 1 ms integration) */
#define TP_DEFAULT_CN0_DROP_THRESHOLD_DBHZ (31.f)
/** C/N0 threshold for measurements use */
#define TP_DEFAULT_CN0_USE_THRESHOLD_DBHZ (27.f)

#define TL_BWT_MAX (18.f * 0.020f)

#define ADJ_CN0_MIN (20.0f)
#define ADJ_CN0_MAX (60.0f)

#define PLL_BW_MIN (10.0f)
#define PLL_BW_MAX (20.0f)

#define FLL_BW_MIN (1.5f)
#define FLL_BW_MAX (3.0f)
