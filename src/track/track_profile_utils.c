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
  u8 cn0_ms;     /**< C/N0 estimator integration time */
  u8 lockdet_ms; /**< Lock detector integration time */
  float flld_ms; /**< FLL discriminator integration time */
  float fpll_ms; /**< FLL & PLL discriminator integration time [ms] */
  u8 fpll_decim; /**< TPF_FPLL_RUN decimation factor
                      (0,1 - no decimation,
                      2 - every second etc) */
  u16 dll_ms;    /**< DLL filter integration time [ms] */
  u8 bit_ms;     /**< Data update period */
  u16 ent_cnt;   /**< State entries count */
  const state_entry_t entries[]; /**< State entries */
} state_table_t;

/* clang-format off */
/**
 * Initial tracking mode (no bit sync, FLL-assisted PLL, 1 ms)
 */
static const state_table_t mode_1msINI = {
  .cn0_ms = 1,
  .lockdet_ms = 1,
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dll_ms = 1,
  .bit_ms = 1,
  .ent_cnt = 1,
  .entries = {
    {1, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE   |
                                                                  TPF_FLL_HALFQ | TPF_FPLL_RUN}
  }
};

#define TP_FLAGS_1MS                          \
  (TPF_EPL_SET | TPF_PLD_SET | TPF_FLL_SET |  \
   TPF_EPL_USE | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN)

/**
 * 1 ms tracking mode for most GPS and QZSS: exactly as above but with bitsync
 */
static const state_table_t mode_1ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 1,
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dll_ms = 1,
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
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dll_ms = 1,
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
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dll_ms = 1,
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
  .flld_ms = 1,
  .fpll_ms = 1,
  .fpll_decim = 1,
  .dll_ms = 1,
  .bit_ms = 4,
  .ent_cnt = 4,
  .entries = {
    {1, TP_FLAGS_1MS | TPF_CN0_SET | TPF_BSYNC_SET},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD},
    {1, TP_FLAGS_1MS | TPF_CN0_ADD | TPF_BSYNC_ADD
                     | TPF_CN0_USE | TPF_BSYNC_UPD }
  }
};

#define TP_FLAGS_2MS                         \
  (TPF_EPL_SET | TPF_PLD_SET | TPF_FLL_SET | \
   TPF_EPL_USE | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN)

/**
 * 2 ms integration profile for most GPS and QZSS
 */
static const state_table_t mode_2ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 2,
  .flld_ms = 2,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dll_ms = 2,
  .bit_ms = 20,
  .ent_cnt = 10,
  .entries = {
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
        TPF_EPL_USE  |                               TPF_PLD_USE | TPF_FLL_USE |
                                                                   TPF_FLL_HALFQ | TPF_FPLL_RUN},
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
  .flld_ms = 2,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dll_ms = 2,
  .bit_ms = 10,
  .ent_cnt = 5,
  .entries = {
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET   |
        TPF_EPL_USE  |                               TPF_PLD_USE | TPF_FLL_USE   |
                                                                   TPF_FLL_HALFQ | TPF_FPLL_RUN},
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
  .flld_ms = 1,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dll_ms = 2,
  .bit_ms = 2,
  .ent_cnt = 2,
  .entries = {
    {1, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                   TPF_FLL_USE |
                                                                   TPF_FLL_HALFQ},
    {1, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},
  }
};

/**
 * 200 ms tracking mode for SBAS and Beidou with D2 nav
 */
static const state_table_t mode_200ms_2ms = {
  .cn0_ms = 2,
  .lockdet_ms = 0, /* not used in FLL only mode */
  .flld_ms = 1,
  .fpll_ms = 2,
  .fpll_decim = 100,
  .dll_ms = 2,
  .bit_ms = 2,
  .ent_cnt = 2,
  .entries = {
    {1, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_SET |
                                                     TPF_FLL_USE |
                                                     TPF_FLL_HALFQ},
    {1, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_SET |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

/**
 * 2 ms tracking mode for Galileo I/NAV
 */
static const state_table_t mode_2ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 2,
  .flld_ms = 2,
  .fpll_ms = 2,
  .fpll_decim = 1,
  .dll_ms = 2,
  .bit_ms = 4,
  .ent_cnt = 10,
  .entries = {
    {2, TP_FLAGS_2MS | TPF_CN0_SET | TPF_BSYNC_SET},
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                                     TPF_BSYNC_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_SET },
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                                     TPF_BSYNC_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_SET |
                       TPF_CN0_USE },
    {2, TP_FLAGS_2MS | TPF_CN0_SET | TPF_BSYNC_ADD |
                                     TPF_BSYNC_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_SET },
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                                     TPF_BSYNC_UPD},

    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_SET },
    {2, TP_FLAGS_2MS | TPF_CN0_ADD | TPF_BSYNC_ADD |
                       TPF_CN0_USE | TPF_BSYNC_UPD},
  }
};

/**
 * 4 ms tracking mode for Galileo I/NAV
 */
static const state_table_t mode_4ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 4,
  .flld_ms = 2,
  .fpll_ms = 4,
  .fpll_decim = 1,
  .dll_ms = 4,
  .bit_ms = 4,
  .ent_cnt = 10,
  .entries = {
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                   TPF_FLL_USE},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  |               TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                   TPF_FLL_USE},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  |               TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                       TPF_CN0_USE |                               TPF_FLL_USE},
    {2, TPF_EPL_ADD  | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  |               TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                   TPF_FLL_USE},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  |               TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_EPL_SET  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                   TPF_FLL_USE},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},
  }
};

/**
 * 5 ms integrations for most GPS and QZSS
 */
static const state_table_t mode_5ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .flld_ms = 2.5,
  .fpll_ms = 5,
  .fpll_decim = 1,
  .dll_ms = 5,
  .bit_ms = 20,
  .ent_cnt = 9,
  .entries = {
    {2, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},
    {2, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET},
    {1, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                      TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_CN0_ADD | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
                      TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_CN0_ADD | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},
  }
};

/**
 * 5 ms integrations for GLO
 */
static const state_table_t mode_5ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 5,
  .flld_ms = 2.5,
  .fpll_ms = 5,
  .fpll_decim = 1,
  .dll_ms = 5,
  .bit_ms = 10,
  .ent_cnt = 5,
  .entries = {
    {2, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},
    {2, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET},
    {1, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                      TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_CN0_ADD | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                                                  TPF_FLL_USE},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

/**
 * 10 ms tracking mode for Galileo I/NAV
 */
static const state_table_t mode_10ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 10,
  .flld_ms = 5,
  .fpll_ms = 10,
  .fpll_decim = 1,
  .dll_ms = 10,
  .bit_ms = 4,
  .ent_cnt = 7,
  .entries = {
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                     TPF_BSYNC_UPD               | TPF_FLL_USE},
    {4, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_SET |
                                     TPF_BSYNC_UPD },

    {2, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE  | TPF_CN0_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},
    {2, TPF_EPL_SET  | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET |
                                     TPF_BSYNC_UPD},
    {4, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
                                     TPF_BSYNC_UPD |               TPF_FLL_USE},

    {4, TPF_EPL_ADD  | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE  | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},
  }
};


/**
 * 10 ms integrations for most GPS and QZSS
 */
static const state_table_t mode_10ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 10,
  .flld_ms = 5,
  .fpll_ms = 10,
  .fpll_decim = 1,
  .dll_ms = 10,
  .bit_ms = 20,
  .ent_cnt = 7,
  .entries = {
    {2, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                                  TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},

    {5, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE |                 TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},

    {2, TPF_CN0_SET | TPF_EPL_SET | TPF_BSYNC_ADD | TPF_PLD_SET | TPF_FLL_SET},
    {3, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                                  TPF_FLL_USE},

    {5, TPF_CN0_ADD | TPF_EPL_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_CN0_USE | TPF_EPL_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

/**
 * 10 ms integrations for GLO
 */
static const state_table_t mode_10ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 10,
  .flld_ms = 5,
  .fpll_ms = 10,
  .fpll_decim = 1,
  .dll_ms = 10,
  .bit_ms = 10,
  .ent_cnt = 4,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                                                  TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},

    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN},
  }
};

#define GLO_FSM_20MS_NUM (GLO_STRING_LENGTH_MS / GLO_MEANDER_WIPEOFF_ALIGN_MS)

#define GLO_FLAGS(x) ((x) ? (TPF_EPL_INV | TPF_BSYNC_INV) : 0)
#define FSM_GLO_100_MS  FSM_GLO_20_MS(0, 1) /*20*/,  FSM_GLO_20_MS(0, 1) /*40*/,  \
                        FSM_GLO_20_MS(0, 1) /*60*/,  FSM_GLO_20_MS(0, 1) /*80*/,  \
                        FSM_GLO_20_MS(0, 1) /*100*/

#define FSM_GLO_400_MS FSM_GLO_100_MS /*100*/, FSM_GLO_100_MS /*200*/, \
                       FSM_GLO_100_MS /*300*/, FSM_GLO_100_MS /*400*/

/* GLO string integration state machine for >=20 integration times
   for 10ms GLO meander wipe-off */
#define FSM_GLO_2000_MS                                 \
    /* navigation part of GLO string (1.7s long) */     \
    FSM_GLO_400_MS /*400*/ , FSM_GLO_400_MS /*800*/,    \
    FSM_GLO_400_MS /*1200*/, FSM_GLO_400_MS /*1600*/,   \
    FSM_GLO_100_MS /*1700*/,                            \
                                                        \
    /* time mark of GLO string (0.3s long) */           \
                         FSM_GLO_20_MS(1, 1), /* 0x3 */ \
    FSM_GLO_20_MS(1, 1), FSM_GLO_20_MS(1, 0), /* 0xE */ \
    FSM_GLO_20_MS(0, 0), FSM_GLO_20_MS(1, 1), /* 0x3 */ \
    FSM_GLO_20_MS(0, 1), FSM_GLO_20_MS(1, 1), /* 0x7 */ \
    FSM_GLO_20_MS(0, 1), FSM_GLO_20_MS(0, 1), /* 0x5 */ \
    FSM_GLO_20_MS(0, 0), FSM_GLO_20_MS(0, 0), /* 0x0 */ \
    FSM_GLO_20_MS(1, 0), FSM_GLO_20_MS(0, 1), /* 0x9 */ \
    FSM_GLO_20_MS(0, 1), FSM_GLO_20_MS(1, 0)  /* 0x6 */

/**
 * 20 ms integrations for GLO
 */
static const state_table_t mode_20ms_10ms = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dll_ms = 20,
  .bit_ms = 10,
  .ent_cnt = GLO_FSM_20MS_NUM * 6, /* 600 */
  .entries = {
#define FSM_GLO_20_MS(odd, even)                                                                  \
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET | GLO_FLAGS(odd)},  \
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(odd)},  \
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(odd)},  \
                                                                                                  \
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(odd) |  \
                      TPF_CN0_USE | TPF_BSYNC_UPD |               TPF_FLL_USE |                   \
                                                                  TPF_FLL_HALFQ},                 \
                                                                                                  \
    {5, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_SET | GLO_FLAGS(even)}, \
                                                                                                  \
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(even) | \
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}

    FSM_GLO_2000_MS
#undef FSM_GLO_20_MS
  }
};

/**
 * 20 ms integrations for GLO
 */
static const state_table_t mode_20ms_10ms_base = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dll_ms = 20,
  .bit_ms = 10,
  .ent_cnt = GLO_FSM_20MS_NUM * 6, /* 600 */
  .entries = {
#define FSM_GLO_20_MS(odd, even)                                                                  \
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET | GLO_FLAGS(odd)},  \
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(odd)},  \
                                                                                                  \
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(odd)},  \
                                                                                                  \
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(odd) |  \
                      TPF_CN0_USE | TPF_BSYNC_UPD |               TPF_FLL_USE |                   \
                                                                  TPF_FLL_HALFQ},                 \
                                                                                                  \
    {5, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_SET | GLO_FLAGS(even)}, \
                                                                                                  \
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD | GLO_FLAGS(even) | \
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}

    FSM_GLO_2000_MS
#undef FSM_GLO_20_MS
  }
};

/**
 * 200 ms integrations for GLO
 */
static const state_table_t mode_200ms_10ms = {
  .cn0_ms = 20,
  .lockdet_ms = 0, /* not used in FLL only mode */
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 10,
  .dll_ms = 20,
  .bit_ms = 10,
  .ent_cnt = GLO_FSM_20MS_NUM * 6, /* 600 */
  .entries = {

#define FSM_GLO_20_MS(odd, even)                                                    \
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_SET | GLO_FLAGS(odd)},  \
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD | GLO_FLAGS(odd)},  \
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD | GLO_FLAGS(odd)},  \
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD | GLO_FLAGS(odd) |  \
                                    TPF_BSYNC_UPD | TPF_FLL_USE |                   \
                                                    TPF_FLL_HALFQ},                 \
                                                                                    \
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_FLL_SET | GLO_FLAGS(even)}, \
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD | GLO_FLAGS(even) | \
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_FLL_USE | TPF_FPLL_RUN}

    FSM_GLO_2000_MS
#undef FSM_GLO_20_MS
  }
};

/**
 * 200 ms integrations for GLO without meander sync
 */
static const state_table_t mode_200ms_10ms_nm = {
    .cn0_ms = 10,
    .lockdet_ms = 0, /* not used in FLL only mode */
    .flld_ms = 5,
    .fpll_ms = 10,
    .fpll_decim = 20,
    .dll_ms = 10,
    .bit_ms = 10,
    .ent_cnt = 4,
    .entries = {
      {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_SET},
      {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD},
      {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD |
                                                      TPF_FLL_USE |
                                                      TPF_FLL_HALFQ},

      {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_SET |
          TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

/**
 * 20 ms integrations for Galileo I/NAV
 */
static const state_table_t mode_20ms_sc4 = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dll_ms = 20,
  .bit_ms = 4,
  .ent_cnt = 7,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},
    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
                      TPF_CN0_USE |                               TPF_FLL_USE },
    {2, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
                                    TPF_BSYNC_UPD},
    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},
    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

/**
 * 20 ms integrations for Galileo I/NAV  (base station profile)
 */
static const state_table_t mode_20ms_sc4_base = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dll_ms = 20,
  .bit_ms = 4,
  .ent_cnt = 7,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},

    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},

    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
                      TPF_CN0_USE |                               TPF_FLL_USE },
    {2, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET |
                                    TPF_BSYNC_UPD },

    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD },

    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

/**
 * 200 ms integrations for Galileo I/NAV
 */
static const state_table_t mode_200ms_sc4 = {
  .cn0_ms = 20,
  .lockdet_ms = 0, /* not used in FLL only mode */
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 10,
  .dll_ms = 20,
  .bit_ms = 4,
  .ent_cnt = 7,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},
    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_FLL_ADD |
                                                    TPF_FLL_USE },
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_SET |
                                    TPF_BSYNC_UPD},
    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_FLL_ADD |
                                    TPF_BSYNC_UPD},
    {4, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_SET | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};


/**
 * 20 ms integrations for most GPS and QZSS
 */
static const state_table_t mode_20ms_20ms = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dll_ms = 20,
  .bit_ms = 20,
  .ent_cnt = 6,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},

    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                      TPF_CN0_USE |                               TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},

    {5, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET},

    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};


/**
 * 20 ms integrations for most GPS and QZSS (base station profile)
 */
static const state_table_t mode_20ms_20ms_base = {
  .cn0_ms = 10,
  .lockdet_ms = 20,
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 1,
  .dll_ms = 20,
  .bit_ms = 20,
  .ent_cnt = 6,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_PLD_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD},

    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
                      TPF_CN0_USE |                               TPF_FLL_USE |
                                                                  TPF_FLL_HALFQ},

    {5, TPF_EPL_ADD | TPF_CN0_SET | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_SET },

    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_PLD_ADD | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_PLD_USE | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

static const state_table_t mode_200ms_20ms = {
  .cn0_ms = 20,
  .lockdet_ms = 0, /* not used in FLL only mode */
  .flld_ms = 10,
  .fpll_ms = 20,
  .fpll_decim = 10,
  .dll_ms = 20,
  .bit_ms = 20,
  .ent_cnt = 6,
  .entries = {
    {2, TPF_EPL_SET | TPF_CN0_SET | TPF_BSYNC_SET | TPF_FLL_SET},
    {2, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD},
    {1, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD},
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD |
                                                    TPF_FLL_USE |
                                                    TPF_FLL_HALFQ},

    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_SET},
    {5, TPF_EPL_ADD | TPF_CN0_ADD | TPF_BSYNC_ADD | TPF_FLL_ADD |
        TPF_EPL_USE | TPF_CN0_USE | TPF_BSYNC_UPD | TPF_FLL_USE | TPF_FPLL_RUN}
  }
};

/* clang-format on */

static const state_table_t *tracking_table_lut[TP_TM_COUNT] =
    {[TP_TM_INITIAL] = &mode_1msINI,

     [TP_TM_1MS_2MS] = &mode_1ms_2ms,
     [TP_TM_2MS_2MS] = &mode_2ms_2ms,
     [TP_TM_200MS_2MS] = &mode_200ms_2ms,

     [TP_TM_1MS_10MS] = &mode_1ms_10ms,
     [TP_TM_2MS_10MS] = &mode_2ms_10ms,
     [TP_TM_5MS_10MS] = &mode_5ms_10ms,
     [TP_TM_10MS_10MS] = &mode_10ms_10ms,
     [TP_TM_20MS_10MS] = &mode_20ms_10ms,
     [TP_TM_20MS_10MS_BASE] = &mode_20ms_10ms_base,
     [TP_TM_200MS_10MS] = &mode_200ms_10ms,
     [TP_TM_200MS_10MS_NM] = &mode_200ms_10ms_nm,

     [TP_TM_1MS_20MS] = &mode_1ms_20ms,
     [TP_TM_2MS_20MS] = &mode_2ms_20ms,
     [TP_TM_5MS_20MS] = &mode_5ms_20ms,
     [TP_TM_10MS_20MS] = &mode_10ms_20ms,
     [TP_TM_20MS_20MS] = &mode_20ms_20ms,
     [TP_TM_20MS_20MS_BASE] = &mode_20ms_20ms_base,
     [TP_TM_200MS_20MS] = &mode_200ms_20ms,

     [TP_TM_1MS_SC4] = &mode_1ms_sc4,
     [TP_TM_2MS_SC4] = &mode_2ms_sc4,
     [TP_TM_4MS_SC4] = &mode_4ms_sc4,
     [TP_TM_10MS_SC4] = &mode_10ms_sc4,
     [TP_TM_20MS_SC4] = &mode_20ms_sc4,
     [TP_TM_20MS_SC4_BASE] = &mode_20ms_sc4_base,
     [TP_TM_200MS_SC4] = &mode_200ms_sc4};

/**
 * Helper for locating tracker state table.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return Table pointer or NULL on error.
 */
static const state_table_t *select_table(tp_tm_e tracking_mode) {
  if ((TP_TM_COUNT <= tracking_mode) ||
      (NULL == tracking_table_lut[tracking_mode])) {
    log_error("tracking mode %d is not initialized", tracking_mode);
    assert(0);
    return NULL;
  }
  return tracking_table_lut[tracking_mode];
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
                                         u16 cycle_no) {
  if (NULL != table && cycle_no < table->ent_cnt) {
    return &table->entries[cycle_no];
  }
  return NULL;
}

/**
 * Computes tracker flags.
 *
 * \param tracker  Tracking channel data.
 * \param cycle_no
 */
u32 tp_get_cycle_flags(tracker_t *tracker, u16 cycle_no) {
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
u16 tp_get_cycle_count(tp_tm_e tracking_mode) {
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);
  assert(tbl->ent_cnt);

  return tbl->ent_cnt;
}

/**
 * Calculates the cycle index to start integration with
 *
 * \param[in] tracker Tracker handler
 * \param[in] mode Tracking mode.
 * \param[in] switch_in_ms Profile switch is planned after this delay [ms]
 *
 * \return The init cycle
 */
u16 tp_calc_init_cycle_no(const tracker_t *tracker,
                          tp_tm_e mode,
                          u8 switch_in_ms) {
  const state_table_t *const tbl = select_table(mode);
  u16 cycle_no = 0;
  if ((tbl == &mode_20ms_10ms) || (tbl == &mode_200ms_10ms) ||
      (tbl == &mode_20ms_10ms_base)) {
    /* We are about to activate GLO >=20ms integration mode.
       Compute the index within 2000ms FSM for GLO meander wipe-off. */
    assert(0 != (tracker->flags & TRACKER_FLAG_GLO_STRING_SYNC));
    u16 ms = tracker->glo_into_string_ms + switch_in_ms;
    assert(0 == (ms % GLO_MEANDER_WIPEOFF_ALIGN_MS));
    u8 cycles_in_20ms = tbl->ent_cnt / GLO_FSM_20MS_NUM;
    assert(0 == (tbl->ent_cnt % GLO_FSM_20MS_NUM));
    cycle_no = (ms / GLO_MEANDER_WIPEOFF_ALIGN_MS) * cycles_in_20ms;
    cycle_no = tp_wrap_cycle(mode, cycle_no);
    assert(cycle_no < tbl->ent_cnt);
  } else if (IS_GLO(tracker->mesid) && (tbl->dll_ms > 10)) {
    /* GLO meander removal is attempted without proper init cycle calculation */
    assert(0);
  }
  return cycle_no;
}

/**
 * Returns cycle duration.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] cycle_no      Cycle number.
 *
 * \return Cycle duration in ms.
 */
u8 tp_get_cycle_duration(tp_tm_e tracking_mode, u16 cycle_no) {
  const state_table_t *tbl = select_table(tracking_mode);
  const state_entry_t *ent = select_entry(tbl, cycle_no);

  assert(NULL != ent);

  return ent->state_ms;
}

/**
 * Wraps cycle number as a modulo of the total number of cycles
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] cycle_no      Cycle number to wrap
 *
 * \return The wrapped cycle number
 */
u16 tp_wrap_cycle(tp_tm_e tracking_mode, u16 cycle_no) {
  const state_table_t *tbl = select_table(tracking_mode);
  assert(tbl != NULL);
  assert(tbl->ent_cnt);
  return cycle_no % tbl->ent_cnt;
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

static const char *tracking_table_str_lut[TP_TM_COUNT] =
    {[TP_TM_INITIAL] = "TM INI",

     [TP_TM_1MS_2MS] = "TM 1/2 MS",
     [TP_TM_2MS_2MS] = "TM 2/2 MS",
     [TP_TM_200MS_2MS] = "TM 200/2 MS",

     [TP_TM_1MS_10MS] = "TM 1/10 MS",
     [TP_TM_2MS_10MS] = "TM 2/10 MS",
     [TP_TM_5MS_10MS] = "TM 5/10 MS",
     [TP_TM_10MS_10MS] = "TM 10/10 MS",
     [TP_TM_20MS_10MS] = "TM 20/10 MS",
     [TP_TM_20MS_10MS_BASE] = "TM 20/10 MS BASE",
     [TP_TM_200MS_10MS] = "TM 200/10 MS",
     [TP_TM_200MS_10MS_NM] = "TM 200/10 MS NM",

     [TP_TM_1MS_20MS] = "TM 1/20 MS",
     [TP_TM_2MS_20MS] = "TM 2/20 MS",
     [TP_TM_5MS_20MS] = "TM 5/20 MS",
     [TP_TM_10MS_20MS] = "TM 10/20 MS",
     [TP_TM_20MS_20MS] = "TM 20/20 MS",
     [TP_TM_20MS_20MS_BASE] = "TM 20/20 MS BASE",
     [TP_TM_200MS_20MS] = "TM 200/20 MS",

     [TP_TM_1MS_SC4] = "TM 1/SC4",
     [TP_TM_2MS_SC4] = "TM 2/SC4",
     [TP_TM_4MS_SC4] = "TM 4/SC4",
     [TP_TM_10MS_SC4] = "TM 10/SC4",
     [TP_TM_20MS_SC4] = "TM 20/SC4",
     [TP_TM_20MS_SC4_BASE] = "TM 20/SC4 BASE",
     [TP_TM_200MS_SC4] = "TM 200/SC4"};

/**
 * Returns a literal for the given mode enumeration.
 *
 * @param[in] tracking_mode Tracking mode
 *
 * @return Mode literal
 */
const char *tp_get_mode_str(tp_tm_e tracking_mode) {
  static const char *str = "?";
  if ((TP_TM_COUNT <= tracking_mode) ||
      (NULL == tracking_table_str_lut[tracking_mode])) {
    log_error("tracking mode %d is not described", tracking_mode);
    assert(0);
    return str;
  }
  return tracking_table_str_lut[tracking_mode];
}
