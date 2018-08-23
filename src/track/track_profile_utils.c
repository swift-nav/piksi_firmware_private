/*
 * Copyright (C) 2016,2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>

#include "track/track_flags.h"
#include "track_common.h"

/**
 * State entry.
 */
typedef struct {
  u8 state_ms; /**< State integration time */
  u32 flags;   /**< State operation flags */
} state_entry_t;

/**
 * State table.
 */
typedef struct {
  u8 cn0_ms;                     /**< C/N0 estimator integration time */
  u8 lockdet_ms;                 /**< Lock detector integration time */
  float alias_ms;                /**< Alias detector integration time */
  float flld_ms;                 /**< FLL discriminator integration time */
  float fpll_ms; /**< FLL & PLL discriminator integration time [ms] */
  u8 fpll_decim; /**< TPF_FPLL_RUN decimation factor
                      (0,1 - no decimation,
                      2 - every second etc) */
  u8 dlld_ms;    /**< DLL discriminator integration time */
  u16 dll_ms;    /**< DLL filter integration time [ms] */
  u8 dll_decim;  /**< TPF_DLL_RUN decimation factor
                      (0,1 - no decimation,
                      2 - every second etc) */
  u8 bit_ms;                     /**< Data update period */
  u8 ent_cnt;                    /**< State entries count */
  const state_entry_t entries[]; /**< State entries */
} state_table_t;

/* clang-format off */
/**
 * Initial tracking mode (no bit sync, FLL-assisted PLL, 1 ms)
 */
static const state_table_t mode_1msINI = {
  .cn0_ms = 1,
  .lockdet_ms = 1,
  .alias_ms = 0, /* not used for 1ms profile as equal to flld_ms */
  .flld_ms = 1,
  .dlld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dll_ms = 1,
  .dll_decim = 1,
  .bit_ms = 1,
  .ent_cnt = 1,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE   |
                                                                  TPF_FLL_HALFQ | TPF_LOOPS_RUN}
  }
};

#define TP_FLAGS_1MS                          \
  (TPF_EPL_SET | TPF_PLD_SET | TPF_FLL_SET |  \
   TPF_EPL_USE | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN)

#define TPF_DATAPILOT_SET (TPF_BIT_PILOT | TPF_BSYNC_SET)
#define TPF_DATAPILOT_ADD (TPF_BIT_PILOT | TPF_BSYNC_ADD)
#define TPF_DATAPILOT_UPD (TPF_BIT_PILOT | TPF_BSYNC_UPD)

/**
 * 1 ms tracking mode for most GPS and QZSS: exactly as above but with bitsync
 */
static const state_table_t mode_1ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 1,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dlld_ms = 1,
  .dll_ms = 1,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 20,
  .entries = {
    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_HALFQ},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE},

    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE | TPF_BSYNC_UPD }
  }
};

/**
 * 1 ms tracking mode for GLO
 */
static const state_table_t mode_1ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 1,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dlld_ms = 1,
  .dll_ms = 1,
  .dll_decim = 1,
  .bit_ms = 10,
  .ent_cnt = 10,
  .entries = {
    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_HALFQ},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE | TPF_BSYNC_UPD}
  }
};


/**
 * 1 ms tracking mode for SBAS and Beidou with D2 nav
 */
static const state_table_t mode_1ms_2ms = {
  .cn0_ms = 2,
  .lockdet_ms = 1,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dlld_ms = 1,
  .dll_ms = 1,
  .dll_decim = 1,
  .bit_ms = 2,
  .ent_cnt = 2,
  .entries = {
    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_HALFQ},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE | TPF_BSYNC_UPD}
  }
};


/**
 * 1 ms tracking mode for Galileo I/NAV
 */
static const state_table_t mode_1ms_sc4 = {
  .cn0_ms = 4,
  .lockdet_ms = 1,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dlld_ms = 1,
  .dll_ms = 1,
  .dll_decim = 1,
  .bit_ms = 4,
  .ent_cnt = 4,
  .entries = {
    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_DATAPILOT_SET},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_DATAPILOT_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_DATAPILOT_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_DATAPILOT_ADD
                     | TPF_CN0_USE | TPF_DATAPILOT_UPD }
  }
};

/**
 * 1 ms tracking mode for Beidou with D1 nav and GPS L5
 */
static const state_table_t mode_1ms_nh20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 1,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dlld_ms = 1,
  .dll_ms = 1,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 20,
  .entries = {
    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_HALFQ},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},

    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE},

    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_BSYNC_ADD},
    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},

    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TPF_EPL_INV  |
        TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE | TPF_BSYNC_UPD }
  }
};

#define TP_FLAGS_2MS                        \
  (TPF_EPL_SET | TPF_PLD_SET | TPF_FLL_SET | \
   TPF_EPL_USE | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN)

/**
 * 2 ms integration profile for most GPS and QZSS
 */
static const state_table_t mode_2ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 2,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 2,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dlld_ms = 2,
  .dll_ms = 2,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 10,
  .entries = {
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
        TPF_EPL_USE  |                               TPF_PLD_USE | TPF_FLL_USE |
                                                                   TPF_FLL_HALFQ | TPF_LOOPS_RUN},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE},

    {2, TP_FLAGS_2MS | TPF_CN0_SET | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE | TPF_BSYNC_UPD}
  }
};

/**
 * 2 ms integration profile for GLO
 */
static const state_table_t mode_2ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 2,
  .alias_ms = 0, /* not used for as equal to flld_ms */
  .flld_ms = 2,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dlld_ms = 2,
  .dll_ms = 2,
  .dll_decim = 1,
  .bit_ms = 10,
  .ent_cnt = 5,
  .entries = {
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE  |                               TPF_PLD_USE | TPF_FLL_USE   |
                                                                   TPF_FLL_HALFQ | TPF_LOOPS_RUN},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE | TPF_BSYNC_UPD}
  }
};

/**
 * 2 ms tracking mode for SBAS and Beidou with D2 nav
 */
static const state_table_t mode_2ms_2ms = {
  .cn0_ms = 2,
  .lockdet_ms = 2,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 1,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dlld_ms = 2,
  .dll_ms = 2,
  .dll_decim = 1,
  .bit_ms = 2,
  .ent_cnt = 2,
  .entries = {
    {1, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                   TPF_FLL_USE |
                                                                   TPF_FLL_HALFQ},
    {1, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},
  }
};

/**
 * 200 ms tracking mode for SBAS and Beidou with D2 nav
 */
static const state_table_t mode_200ms_2ms = {
  .cn0_ms = 2,
  .lockdet_ms = 2,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 1,
  .fpll_ms = 2,
  .fpll_decim = 100,
  .dlld_ms = 2,
  .dll_ms = 2,
  .dll_decim = 1,
  .bit_ms = 2,
  .ent_cnt = 2,
  .entries = {
    {1, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                   TPF_FLL_USE |
                                                                   TPF_FLL_HALFQ},
    {1, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN}
  }
};

/**
 * 2 ms tracking mode for Galileo I/NAV
 */
static const state_table_t mode_2ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 2,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 2,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dlld_ms = 2,
  .dll_ms = 2,
  .dll_decim = 1,
  .bit_ms = 4,
  .ent_cnt = 10,
  .entries = {
    {2, TP_FLAGS_2MS | TPF_CN0_SET | TPF_DATAPILOT_SET},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_ADD |
                                     TPF_DATAPILOT_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_SET },
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_ADD |
                                     TPF_DATAPILOT_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_SET |
                       TPF_CN0_USE },
    {2, TP_FLAGS_2MS | TPF_CN0_SET | TPF_DATAPILOT_ADD |
                                     TPF_DATAPILOT_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_SET },
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_ADD |
                                     TPF_DATAPILOT_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_SET },
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_DATAPILOT_ADD |
                       TPF_CN0_USE | TPF_DATAPILOT_UPD},
  }
};



/**
 * 2 ms integration profile for Beidou with D1 nav and GPS L5
 */
static const state_table_t mode_2ms_nh20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 2,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 2,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dlld_ms = 2,
  .dll_ms = 2,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 14,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   |
                                                                  TPF_FLL_HALFQ | TPF_LOOPS_RUN},

    {2, TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {1, TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {2, TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {2, TPF_EPL_INV |
        TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE | TPF_CN0_USE |                 TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {1, TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {2, TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {2, TPF_EPL_INV |
        TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {1, TPF_EPL_INV |
        TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD   |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},
  }
};



/**
 * 4 ms tracking mode for Galileo I/NAV
 */
static const state_table_t mode_4ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 4,
  .alias_ms = 0, /* not used as equal to flld_ms */
  .flld_ms = 4,
  .fpll_ms = 4,
  .fpll_decim = 1,
  .dlld_ms = 4,
  .dll_ms = 4,
  .dll_decim = 1,
  .bit_ms = 4,
  .ent_cnt = 11,
  .entries = {
    {1, TPF_EPL_SET  | TPF_CN0_SET | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  |               TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  |               TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET |
                       TPF_CN0_USE },
    {2, TPF_EPL_ADD  | TPF_CN0_SET | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  |               TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET },
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  |               TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET },
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},
  }
};

/**
 * 5 ms integrations for most GPS and QZSS
 */
static const state_table_t mode_5ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .alias_ms = 0, /* not used in this profile as replaced by FLL */
  .flld_ms = 2.5,
  .fpll_ms = 5,
  .fpll_decim = 1,
  .dlld_ms = 5,
  .dll_ms = 5,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 9,
  .entries = {
    {1, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                                  TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
                      TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_CN0_ADD | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
                      TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_CN0_ADD | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},
  }
};

/**
 * 5 ms integrations for GLO
 */
static const state_table_t mode_5ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .alias_ms = 0, /* not used in this profile as replaced by FLL  */
  .flld_ms = 2.5,
  .fpll_ms = 5,
  .fpll_decim = 1,
  .dlld_ms = 5,
  .dll_ms = 5,
  .dll_decim = 1,
  .bit_ms = 10,
  .ent_cnt = 5,
  .entries = {
    {1, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                                  TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
                      TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_CN0_ADD | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN}
  }
};

/**
 * 5 ms integrations for Beidou with D1 nav and GPS L5
 */
static const state_table_t mode_5ms_nh20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .alias_ms = 0, /* not used in this profile as equal to flld_ms */
  .flld_ms = 5,
  .fpll_ms = 5,
  .fpll_decim = 1,
  .dlld_ms = 5,
  .dll_ms = 5,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 14,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ | TPF_LOOPS_RUN},
    {1, TPF_EPL_INV |
        TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {2, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD   |
        TPF_EPL_USE | TPF_CN0_USE |                 TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD   |
        TPF_EPL_USE |                               TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN},

    {1, TPF_EPL_SET | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {3, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD   |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE   | TPF_LOOPS_RUN}
  }
};


/**
 * 10 ms tracking mode for Galileo I/NAV
 */
static const state_table_t mode_10ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 10,
  .alias_ms = 0, /* not used in this profile as replaced by FLL */
  .flld_ms = 5,
  .fpll_ms = 10,
  .fpll_decim = 1,
  .dlld_ms = 10,
  .dll_ms = 10,
  .dll_decim = 1,
  .bit_ms = 4,
  .ent_cnt = 11,
  .entries = {
    {1, TPF_EPL_SET  | TPF_CN0_SET | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                     TPF_DATAPILOT_UPD               | TPF_FLL_USE},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_SET },
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                     TPF_DATAPILOT_UPD },

    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  | TPF_CN0_USE |                     TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_DATAPILOT_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                     TPF_DATAPILOT_UPD},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD },
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_ADD  |               TPF_DATAPILOT_UPD |               TPF_FLL_USE},

    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_SET },
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},
  }
};


/**
 * 10 ms integrations for most GPS and QZSS
 */
static const state_table_t mode_10ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .alias_ms = 5, /* not used in this profile as replaced by FLL */
  .flld_ms = 5,
  .fpll_ms = 10,
  .fpll_decim = 1,
  .dlld_ms = 10,
  .dll_ms = 10,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 9,
  .entries = {
    {1, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                    TPF_PLD_USE | TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},

    {2, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_CN0_USE | TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN},

    {2, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                    TPF_PLD_USE | TPF_FLL_USE},

    {2, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_CN0_USE | TPF_EPL_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN}
  }
};

/**
 * 10 ms integrations for GLO
 */
static const state_table_t mode_10ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .alias_ms = 2.5,
  .flld_ms = 5,
  .fpll_ms = 10,
  .fpll_decim = 1,
  .dlld_ms = 10,
  .dll_ms = 10,
  .dll_decim = 1,
  .bit_ms = 10,
  .ent_cnt = 5,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                                                TPF_ALIAS_1ST},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                                                    TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND |
                                                                  TPF_FLL_HALFQ},

    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET |
                                                                                TPF_ALIAS_2ND},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND | TPF_LOOPS_RUN},
  }
};

/**
 * 200 ms integrations for GLO
 */
static const state_table_t mode_200ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .alias_ms = 0, /* not used */
  .flld_ms = 5,
  .fpll_ms = 10,
  .fpll_decim = 20,
  .dlld_ms = 10,
  .dll_ms = 10,
  .dll_decim = 1,
  .bit_ms = 10,
  .ent_cnt = 5,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                    TPF_PLD_USE | TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},

    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_LOOPS_RUN}
  }
};

/**
 * 10 ms integrations for Beidou with D1 nav and GPS L5
 */
static const state_table_t mode_10ms_nh20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .alias_ms = 5,
  .flld_ms = 10,
  .fpll_ms = 10,
  .fpll_decim = 1,
  .dlld_ms = 10,
  .dll_ms = 10,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 14,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                    TPF_PLD_USE |               TPF_ALIAS_1ST},

    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
        TPF_EPL_USE | TPF_CN0_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND |
                                                                  TPF_FLL_HALFQ               | TPF_LOOPS_RUN},

    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                    TPF_PLD_USE |               TPF_ALIAS_1ST},

    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {3, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND | TPF_LOOPS_RUN}
  }
};


/**
 * 20 ms integrations for Galileo I/NAV
 */
static const state_table_t mode_20ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .alias_ms = 4,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dlld_ms = 20,
  .dll_ms = 20,
  .dll_decim = 1,
  .bit_ms = 4,
  .ent_cnt = 11,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_1ST},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_2ND},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                      TPF_CN0_USE |                                                 TPF_FLL_USE },
    {2, TPF_EPL_ADD | TPF_CN0_SET | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_SET | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_2ND},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_2ND},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND | TPF_LOOPS_RUN}
  }
};

/**
 * 200 ms integrations for Galileo I/NAV
 */
static const state_table_t mode_200ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .alias_ms = 4,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 10,
  .dlld_ms = 20,
  .dll_ms = 20,
  .dll_decim = 1,
  .bit_ms = 4,
  .ent_cnt = 11,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_DATAPILOT_SET | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_1ST},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_2ND},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                      TPF_CN0_USE |                                   TPF_FLL_USE },
    {2, TPF_EPL_ADD | TPF_CN0_SET | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_SET | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_2ND},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                    TPF_DATAPILOT_UPD |                             TPF_ALIAS_2ND},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_SET | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_DATAPILOT_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_DATAPILOT_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND | TPF_LOOPS_RUN}
  }
};


/**
 * 20 ms integrations for most GPS and QZSS
 */
static const state_table_t mode_20ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .alias_ms = 2.5,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dlld_ms = 20,
  .dll_ms = 20,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 9,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                                                TPF_ALIAS_1ST},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                                                                                TPF_ALIAS_2ND},

    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                                                                                TPF_ALIAS_2ND},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                      TPF_CN0_USE |                               TPF_FLL_USE | TPF_ALIAS_2ND |
                                                                  TPF_FLL_HALFQ},

    {2, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET | TPF_ALIAS_SET |
                                                                                TPF_ALIAS_2ND},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                                                                                TPF_ALIAS_2ND},

    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
                                                                                TPF_ALIAS_2ND},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_SET |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND | TPF_LOOPS_RUN}
  }
};

static const state_table_t mode_200ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .alias_ms = 0, /* not used */
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 10,
  .dlld_ms = 20,
  .dll_ms = 20,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 9,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_USE | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                                  TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},

    {1, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD |               TPF_FLL_USE | TPF_LOOPS_RUN}
  }
};

/**
 * 20 ms integrations for Beidou with D1 nav and GPS L5
 */
static const state_table_t mode_20ms_nh20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .alias_ms = 5,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dlld_ms = 20,
  .dll_ms = 20,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 14,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                                                TPF_ALIAS_1ST},

    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                      TPF_CN0_USE |                               TPF_FLL_USE | TPF_ALIAS_2ND |
                                                                  TPF_FLL_HALFQ},

    {1, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                                                TPF_ALIAS_1ST},

    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {3, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND | TPF_LOOPS_RUN}
  }
};

/**
 * 200 ms integrations for Beidou with D1 nav and GPS L5
 */
static const state_table_t mode_200ms_nh20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .alias_ms = 5,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 10,
  .dlld_ms = 20,
  .dll_ms = 20,
  .dll_decim = 1,
  .bit_ms = 20,
  .ent_cnt = 14,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {3, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                                                TPF_ALIAS_1ST},

    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {2, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                      TPF_CN0_USE |                               TPF_FLL_USE | TPF_ALIAS_2ND |
                                                                  TPF_FLL_HALFQ},

    {1, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET | TPF_ALIAS_SET},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
                                                                                TPF_ALIAS_1ST},

    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {3, TPF_EPL_INV |
        TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | TPF_ALIAS_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_ALIAS_2ND | TPF_LOOPS_RUN}
  }
};
/* clang-format on */

/**
 * Helper for locating tracker state table.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return Table pointer or NULL on error.
 */
static const state_table_t *select_table(tp_tm_e tracking_mode) {
  switch (tracking_mode) {
    case TP_TM_INITIAL:
      return &mode_1msINI;

    case TP_TM_1MS_20MS:
      return &mode_1ms_20ms;

    case TP_TM_1MS_10MS:
      return &mode_1ms_10ms;

    case TP_TM_1MS_2MS:
      return &mode_1ms_2ms;

    case TP_TM_1MS_NH20MS:
      return &mode_1ms_nh20ms;

    case TP_TM_2MS_20MS:
      return &mode_2ms_20ms;

    case TP_TM_2MS_10MS:
      return &mode_2ms_10ms;

    case TP_TM_2MS_2MS:
      return &mode_2ms_2ms;

    case TP_TM_2MS_NH20MS:
      return &mode_2ms_nh20ms;

    case TP_TM_5MS_20MS:
      return &mode_5ms_20ms;

    case TP_TM_5MS_10MS:
      return &mode_5ms_10ms;

    case TP_TM_5MS_NH20MS:
      return &mode_5ms_nh20ms;

    case TP_TM_10MS_20MS:
      return &mode_10ms_20ms;

    case TP_TM_10MS_10MS:
      return &mode_10ms_10ms;

    case TP_TM_10MS_NH20MS:
      return &mode_10ms_nh20ms;

    case TP_TM_20MS_20MS:
      return &mode_20ms_20ms;

    case TP_TM_20MS_NH20MS:
      return &mode_20ms_nh20ms;

    case TP_TM_1MS_SC4:
      return &mode_1ms_sc4;

    case TP_TM_2MS_SC4:
      return &mode_2ms_sc4;

    case TP_TM_4MS_SC4:
      return &mode_4ms_sc4;

    case TP_TM_10MS_SC4:
      return &mode_10ms_sc4;

    case TP_TM_20MS_SC4:
      return &mode_20ms_sc4;

    case TP_TM_200MS_20MS:
      return &mode_200ms_20ms;

    case TP_TM_200MS_10MS:
      return &mode_200ms_10ms;

    case TP_TM_200MS_2MS:
      return &mode_200ms_2ms;

    case TP_TM_200MS_NH20MS:
      return &mode_200ms_nh20ms;

    case TP_TM_200MS_SC4:
      return &mode_200ms_sc4;

    default:
      assert(!"Invalid mode");
      break;
  }

  assert(false);
  return NULL;
}

/**
 * Selects entry for the given cycle number.
 *
 * \param[in] table    Tracker state table.
 * \param[in] cycle_no Cycle number.
 *
 * \return Entry for the given cycle number or NULL on error.
 */
static const state_entry_t *select_entry(const state_table_t *table,
                                         u8 cycle_no) {
  if (NULL != table && cycle_no < table->ent_cnt) {
    return &table->entries[cycle_no];
  }
  return NULL;
}

/**
 * Returns next cycle number
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] cycle_no      Cycle number.
 *
 * \return Number of cycles in the given mode
 */
u8 tp_next_cycle_counter(tp_tm_e tracking_mode, u8 cycle_no) {
  u8 cycle_cnt; /**< Number of cycles in the current tracking mode */

  cycle_cnt = tp_get_cycle_count(tracking_mode);

  if (cycle_cnt > 0) {
    if (++cycle_no >= cycle_cnt) cycle_no = 0;
  } else {
    cycle_no = 0;
  }

  return cycle_no;
}

/**
 * Computes tracker flags.
 *
 * \param tracker  Tracking channel data.
 * \param cycle_no
 */
u32 tp_get_cycle_flags(tracker_t *tracker, u8 cycle_no) {
  const state_table_t *tbl = select_table(tracker->tracking_mode);
  const state_entry_t *ent = select_entry(tbl, cycle_no);

  assert(NULL != ent);

  return ent->flags;
}

/**
 * Returns cycle count for the current integration mode.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return Number of cycles in the given mode
 */
u8 tp_get_cycle_count(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->ent_cnt;
}

/**
 * Returns current cycle duration.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] cycle_no      Current cycle number.
 *
 * \return Current cycle duration in ms.
 */
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no) {
  const state_table_t *tbl = select_table(tracking_mode);
  const state_entry_t *ent = select_entry(tbl, cycle_no);

  assert(NULL != ent);

  return ent->state_ms;
}

/**
 * Returns rollover cycle duration.
 *
 * Rollover cycle number corresponds to current cycle number plus two.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] cycle_no      Current cycle number.
 *
 * \return Rollover cycle duration in ms.
 */
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(tbl != NULL);

  u8 cycle_cnt = tbl->ent_cnt;
  cycle_no += 2;
  cycle_no %= cycle_cnt;

  const state_entry_t *ent = select_entry(tbl, cycle_no);

  assert(ent != NULL);

  return ent->state_ms;
}

/**
 * Get C/N0 estimator update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return C/N0 estimator update period in ms.
 */
u8 tp_get_cn0_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->cn0_ms;
}

/**
 * Get lock detector update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return lock detector update period in ms.
 */
u8 tp_get_ld_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->lockdet_ms;
}

/**
 * Get false lock (alias) detector update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return false lock (alias) detector update period in ms.
 */
float tp_get_alias_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->alias_ms;
}

/**
 * Get FLL discriminator update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return FLL discriminator update period in ms.
 */
float tp_get_flld_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->flld_ms;
}

/**
 * Get FLL & PLL filter update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return FLL & PLL filter update period in ms.
 */
u8 tp_get_fpll_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->fpll_ms;
}

/**
 * Get bit sync update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return bit sync update period in ms.
 */
u8 tp_get_bit_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->bit_ms;
}

/**
 * Get DLL discriminator update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return DLL discriminator update period in ms.
 */
u8 tp_get_dlld_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->dlld_ms;
}

/**
 * Get DLL run decimation factor.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return DLL decimation factor
 */
u8 tp_get_dll_decim(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->dll_decim;
}

/**
 * Get FLL & PLL run decimation factor.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return DLL decimation factor
 */
u8 tp_get_fpll_decim(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->fpll_decim;
}

/**
 * Get DLL integration period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return DLL integration period in ms.
 */
u8 tp_get_dll_ms(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->dll_ms;
}

/**
 * Returns a literal for the given mode enumeration.
 *
 * @param[in] v Tracking mode
 *
 * @return Mode literal
 */
const char *tp_get_mode_str(tp_tm_e v) {
  const char *str = "?";
  switch (v) {
    case TP_TM_INITIAL:
      str = "TM INI";
      break;
    case TP_TM_1MS_20MS:
      str = "TM 1/20 MS";
      break;
    case TP_TM_1MS_10MS:
      str = "TM 1/10 MS";
      break;
    case TP_TM_1MS_2MS:
      str = "TM 1/2 MS";
      break;
    case TP_TM_1MS_NH20MS:
      str = "TM 1/NH20 MS";
      break;
    case TP_TM_2MS_20MS:
      str = "TM 2/20 MS";
      break;
    case TP_TM_2MS_10MS:
      str = "TM 2/10 MS";
      break;
    case TP_TM_2MS_2MS:
      str = "TM 2/2 MS";
      break;
    case TP_TM_2MS_NH20MS:
      str = "TM 2/20 MS";
      break;
    case TP_TM_5MS_20MS:
      str = "TM 5/20 MS";
      break;
    case TP_TM_5MS_10MS:
      str = "TM 5/10 MS";
      break;
    case TP_TM_5MS_NH20MS:
      str = "TM 5/NH20 MS";
      break;
    case TP_TM_10MS_20MS:
      str = "TM 10/20 MS";
      break;
    case TP_TM_10MS_10MS:
      str = "TM 10/10 MS";
      break;
    case TP_TM_10MS_NH20MS:
      str = "TM 10/NH20 MS";
      break;
    case TP_TM_20MS_20MS:
      str = "TM 20/20 MS";
      break;
    case TP_TM_20MS_NH20MS:
      str = "TM 20/NH20 MS";
      break;
    case TP_TM_1MS_SC4:
      str = "TM 1/SC4 MS";
      break;
    case TP_TM_2MS_SC4:
      str = "TM 2/SC4 MS";
      break;
    case TP_TM_4MS_SC4:
      str = "TM 4/SC4 MS";
      break;
    case TP_TM_10MS_SC4:
      str = "TM 10/SC4 MS";
      break;
    case TP_TM_20MS_SC4:
      str = "TM 20/SC4 MS";
      break;
    case TP_TM_200MS_20MS:
      str = "TM 200/20 MS";
      break;
    case TP_TM_200MS_10MS:
      str = "TM 200/10 MS";
      break;
    case TP_TM_200MS_2MS:
      str = "TM 200/2 MS";
      break;
    case TP_TM_200MS_NH20MS:
      str = "TM 200/NH20 MS";
      break;
    case TP_TM_200MS_SC4:
      str = "TM 200/SC4 MS";
      break;

    default:
      assert(false);
  }
  return str;
}
