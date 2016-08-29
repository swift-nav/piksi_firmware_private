/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_profile_utils.h"

#include <assert.h>

#define TP_FLAGS_SHORT_DEFAULT \
  (TP_CFLAG_SHORT_CYCLE | TP_CFLAG_ALIAS_FIRST | TP_CFLAG_CN0_SET | \
   TP_CFLAG_EPL_SET| TP_CFLAG_ALIAS_SET | TP_CFLAG_LD_SET | TP_CFLAG_FLL_SET)

#define TP_FLAGS_LONG_DEFAULT_1PN \
  (TP_CFLAG_LONG_CYCLE | TP_CFLAG_ALIAS_SECOND | TP_CFLAG_CN0_ADD | \
   TP_CFLAG_CN0_USE | TP_CFLAG_EPL_ADD | TP_CFLAG_EPL_USE | \
   TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE | \
   TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE)

#define TP_FLAGS_SPLIT_DEF \
    (TP_CFLAG_CN0_ADD | TP_CFLAG_EPL_ADD | TP_CFLAG_LD_ADD | \
     TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_SECOND)

#define TP_FLAGS_SPLIT_LAST \
    (TP_FLAGS_SPLIT_DEF | TP_CFLAG_LD_USE | TP_CFLAG_CN0_USE | \
         TP_CFLAG_EPL_USE | TP_CFLAG_BIT_SYNC_UPDATE )

#define TP_FLAGS_PIPELINING_DEFAULT \
  (TP_CFLAG_ALIAS_FIRST | TP_CFLAG_CN0_SET | \
   TP_CFLAG_EPL_SET | TP_CFLAG_ALIAS_SET | TP_CFLAG_LD_SET | \
   TP_CFLAG_CN0_USE | TP_CFLAG_EPL_USE | TP_CFLAG_BIT_SYNC_UPDATE )

/**
 * State entry.
 */
typedef struct {
  u8  int_ms;  /**< State integration time */
  u32 flags;   /**< State operation flags */
} state_entry_t;

/**
 * State table.
 */
typedef struct {
  u8 int_ms;  /**< General integration time */
  u8 cn0_ms;  /**< C/N0 estimator integration time */
  u8 ld_ms;   /**< Lock detector integration time */
  u8 fl_ms;   /**< Alias detector integration time */
  u8 fll_ms;  /**< FLL discriminator integration time */
  u8 bit_ms;  /**< Data update period */
  u8 ent_cnt; /**< State entries count */
  const state_entry_t entries[]; /**< State entries */
} state_table_t;

/**
 * Initial tracking mode (no bit sync, FLL-assisted PLL, 1 ms)
 */
static const state_table_t mode_1msINI = {
  .int_ms  = 1,
  .cn0_ms  = 1,
  .ld_ms   = 1,
  .fl_ms   = 1,
  .fll_ms  = 1,
  .bit_ms  = 1,
  .ent_cnt = 1,
  .entries = {
    {1, (TP_CFLAG_USE_CONTROLLER |
         TP_CFLAG_CN0_SET | TP_CFLAG_CN0_USE |
         TP_CFLAG_EPL_SET | TP_CFLAG_EPL_USE |
         TP_CFLAG_BIT_SYNC_UPDATE | TP_CFLAG_LD_SET | TP_CFLAG_LD_USE)}
  }
};

/**
 * 5 ms integrations; split mode.
 */
static const state_table_t mode_5msSPLT = {
  .int_ms  = 5,
  .cn0_ms  = 5,
  .ld_ms   = 1,
  .fl_ms   = 1,
  .fll_ms  = 1,
  .bit_ms  = 5,
  .ent_cnt = 5,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 1, TP_FLAGS_SPLIT_DEF | TP_CFLAG_LONG_CYCLE },
    { 1, TP_FLAGS_SPLIT_DEF },
    { 1, TP_FLAGS_SPLIT_DEF },
    { 1, TP_FLAGS_SPLIT_LAST },
  }
};

/**
 * 10 ms integrations; split mode.
 */
static const state_table_t mode_10msSPLT = {
  .int_ms  = 10,
  .cn0_ms  = 5,
  .ld_ms   = 1,
  .fl_ms   = 1,
  .fll_ms  = 1,
  .bit_ms  = 10,
  .ent_cnt = 10,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 1, TP_FLAGS_SPLIT_DEF | TP_CFLAG_LONG_CYCLE },
    { 1, TP_FLAGS_SPLIT_DEF },
    { 1, TP_FLAGS_SPLIT_DEF },
    { 1, TP_FLAGS_SPLIT_DEF | TP_CFLAG_CN0_USE },
    { 1, TP_FLAGS_SPLIT_DEF | TP_CFLAG_CN0_SET },
    { 1, TP_FLAGS_SPLIT_DEF },
    { 1, TP_FLAGS_SPLIT_DEF },
    { 1, TP_FLAGS_SPLIT_DEF },
    { 1, TP_FLAGS_SPLIT_LAST },
  }
};

/**
 * 5 ms integrations; pipelining mode.
 */
static const state_table_t mode_5msPIP = {
  .int_ms  = 5,
  .cn0_ms  = 5,
  .ld_ms   = 5,
  .fl_ms   = 5,
  .fll_ms  = 1,
  .bit_ms  = 5,
  .ent_cnt = 1,
  .entries = {
    { 5, TP_FLAGS_PIPELINING_DEFAULT },
  }
};

/**
 * 10 ms integrations; pipelining mode.
 */
static const state_table_t mode_10msPIP = {
  .int_ms  = 10,
  .cn0_ms  = 10,
  .ld_ms   = 10,
  .fl_ms   = 10,
  .fll_ms  = 1,
  .bit_ms  = 10,
  .ent_cnt = 1,
  .entries = {
    { 10, TP_FLAGS_PIPELINING_DEFAULT },
  }
};

/**
 * 20 ms integrations; pipelining mode.
 */
static const state_table_t mode_20msPIP = {
  .int_ms  = 20,
  .cn0_ms  = 20,
  .ld_ms   = 20,
  .fl_ms   = 20,
  .fll_ms  = 1,
  .bit_ms  = 20,
  .ent_cnt = 1,
  .entries = {
    { 20, TP_FLAGS_PIPELINING_DEFAULT },
  }
};

/**
 * 5 ms integrations; 1+N mode.
 */
static const state_table_t mode_5ms1PN = {
  .int_ms  = 5,
  .cn0_ms  = 5,
  .ld_ms   = 5,
  .fl_ms   = 4,
  .fll_ms  = 1,
  .bit_ms  = 5,
  .ent_cnt = 2,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 4, TP_FLAGS_LONG_DEFAULT_1PN },
  }
};

/**
 * 10 ms integrations; 1+N mode.
 */
static const state_table_t mode_10ms1PN = {
  .int_ms  = 10,
  .cn0_ms  = 10,
  .ld_ms   = 10,
  .fl_ms   = 9,
  .fll_ms  = 1,
  .bit_ms  = 10,
  .ent_cnt = 2,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 9, TP_FLAGS_LONG_DEFAULT_1PN },
  }
};

/**
 * 20 ms integrations; 1+N mode.
 */
static const state_table_t mode_20ms1PN = {
  .int_ms  = 20,
  .cn0_ms  = 20,
  .ld_ms   = 20,
  .fl_ms   = 19,
  .fll_ms  = 1,
  .bit_ms  = 20,
  .ent_cnt = 2,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 19, TP_FLAGS_LONG_DEFAULT_1PN }
  }
};

/**
 * 10 ms integrations; 1+N5 mode.
 */
static const state_table_t mode_10ms1PN5 = {
  .int_ms  = 10,
  .cn0_ms  = 10,
  .ld_ms   = 5,
  .fl_ms   = 5,
  .fll_ms  = 1,
  .bit_ms  = 5,
  .ent_cnt = 3,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 4,
      (TP_CFLAG_ALIAS_FIRST | TP_CFLAG_LONG_CYCLE | TP_CFLAG_CN0_ADD |
       TP_CFLAG_EPL_ADD | TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE)
    },
    { 5,
      (TP_CFLAG_ALIAS_SECOND | TP_CFLAG_CN0_ADD | TP_CFLAG_EPL_ADD |
       TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE) | TP_CFLAG_CN0_USE | TP_CFLAG_EPL_USE
    },
  }
};

/**
 * 20 ms integrations; 1+N5 mode.
 */
static const state_table_t mode_20ms1PN5 = {
  .int_ms  = 20,
  .cn0_ms  = 20,
  .ld_ms   = 5,
  .fl_ms   = 5,
  .fll_ms  = 15,
  .bit_ms  = 5,
  .ent_cnt = 5,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 4,
      (TP_CFLAG_ALIAS_FIRST | TP_CFLAG_LONG_CYCLE | TP_CFLAG_CN0_ADD |
       TP_CFLAG_EPL_ADD | TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE | TP_CFLAG_FLL_ADD | TP_CFLAG_FLL_FIRST)
    },
    { 5,
      (TP_CFLAG_ALIAS_SECOND | TP_CFLAG_CN0_ADD | TP_CFLAG_EPL_ADD |
       TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE | TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND)
    },
    { 5,
      (TP_CFLAG_ALIAS_SECOND | TP_CFLAG_CN0_ADD | TP_CFLAG_EPL_ADD |
       TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE | TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND)
    },
    { 5,
      (TP_CFLAG_ALIAS_SECOND | TP_CFLAG_CN0_ADD | TP_CFLAG_EPL_ADD |
       TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE | TP_CFLAG_CN0_USE | TP_CFLAG_EPL_USE |
       TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND | TP_CFLAG_FLL_USE)
    },
  }
};

/**
 * 20 ms integrations; 1+N10 mode.
 */
static const state_table_t mode_20ms1PN10 = {
  .int_ms  = 20,
  .cn0_ms  = 20,
  .ld_ms   = 10,
  .fl_ms   = 10,
  .fll_ms  = 10,
  .bit_ms  = 20,
  .ent_cnt = 3,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT },
    { 9,
      (TP_CFLAG_ALIAS_FIRST | TP_CFLAG_LONG_CYCLE | TP_CFLAG_CN0_ADD |
       TP_CFLAG_EPL_ADD | TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE | TP_CFLAG_FLL_ADD | TP_CFLAG_FLL_FIRST)
    },
    { 10,
      (TP_CFLAG_ALIAS_SECOND | TP_CFLAG_CN0_ADD | TP_CFLAG_EPL_ADD |
       TP_CFLAG_ALIAS_ADD | TP_CFLAG_BIT_SYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE | TP_CFLAG_CN0_USE | TP_CFLAG_EPL_USE |
       TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND | TP_CFLAG_FLL_USE)
    },
  }
};

/**
 * Helper for locating tracker state table.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 *
 * \return Table pointer or NULL on error.
 */
static const state_table_t *select_table(tp_tm_e tracking_mode, u8 int_ms)
{
  switch (tracking_mode) {
  case TP_TM_INITIAL:
    if (int_ms == 1)
      return &mode_1msINI;
    return &mode_1msINI;
    break;

  case TP_TM_ONE_PLUS_N:
    switch (int_ms) {
    case 5:  return &mode_5ms1PN;
    case 10: return &mode_10ms1PN;
    case 20: return &mode_20ms1PN;
    default: break;
    }
    break;

  case TP_TM_ONE_PLUS_N5:
    switch (int_ms) {
    case 5:  return &mode_5ms1PN;
    case 10: return &mode_10ms1PN5;
    case 20: return &mode_20ms1PN5;
    default: break;
    }
    break;

  case TP_TM_ONE_PLUS_N10:
    switch (int_ms) {
    case 10: return &mode_10ms1PN5;
    case 20: return &mode_20ms1PN10;
    default: break;
    }
    break;

  case TP_TM_SPLIT:
    switch (int_ms) {
    case 5:  return &mode_5msSPLT;
    case 10: return &mode_10msSPLT;
//    case 20: return &mode_20ms1PN5;
    default: break;
    }
    break;

  case TP_TM_PIPELINING:
    switch (int_ms) {
    case 5:  return &mode_5msPIP;
    case 10: return &mode_10msPIP;
    case 20: return &mode_20msPIP;
    default: break;
    }
    break;

  default:
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
                                         u8 cycle_no)
{
  if (NULL != table && cycle_no < table->ent_cnt) {
    return &table->entries[cycle_no];
  }
  return NULL;
}

/**
 * Returns next cycle number
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 * \param[in] cycle_no      Cycle number.
 *
 * \return Number of cycles in the given mode
 */
u8 tp_next_cycle_counter(tp_tm_e tracking_mode,
                         u8 int_ms,
                         u8 cycle_no)
{
  u8 cycle_cnt; /**< Number of cycles in the current tracking mode */

  cycle_cnt = tp_get_cycle_count(tracking_mode, int_ms);

  if (cycle_cnt > 0) {
    if (++cycle_no >= cycle_cnt)
      cycle_no = 0;
  } else {
    cycle_no = 0;
  }

  return cycle_no;
}

/**
 * Computes tracker flags.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 * \param[in] cycle_no      Cycle number.
 */
u32 tp_get_cycle_flags(tp_tm_e tracking_mode,
                       u8 int_ms,
                       u8 cycle_no)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);
  const state_entry_t *ent = select_entry(tbl, cycle_no);

  assert(NULL != ent);

  return ent->flags;
}

/**
 * Returns cycle count for the current integration mode.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 *
 * \return Number of cycles in the given mode
 */
u8 tp_get_cycle_count(tp_tm_e tracking_mode, u8 int_ms)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);

  assert(NULL != tbl);

  return tbl->ent_cnt;
}

/**
 * Returns current cycle duration.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 * \param[in] cycle_no      Current cycle number.
 *
 * \return Current cycle duration in ms.
 */
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode,
                                 u8 int_ms,
                                 u8 cycle_no)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);
  const state_entry_t *ent = select_entry(tbl, cycle_no);

  assert(NULL != ent);

  return ent->int_ms;
}

/**
 * Returns rollover cycle duration.
 *
 * Rollover cycle number corresponds to current cycle number plus two.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 * \param[in] cycle_no      Current cycle number.
 *
 * \return Rollover cycle duration in ms.
 */
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode,
                                   u8 int_ms,
                                   u8 cycle_no)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);

  assert(tbl != NULL);

  u8 cycle_cnt = tbl->ent_cnt;
  cycle_no += 2;
  cycle_no %= cycle_cnt;

  const state_entry_t *ent = select_entry(tbl, cycle_no);

  assert(ent != NULL);

  return ent->int_ms;
}

/**
 * Get C/N0 estimator update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 *
 * \return C/N0 estimator update period in ms.
 */
u8 tp_get_cn0_ms(tp_tm_e tracking_mode, u8 int_ms)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);

  assert(NULL != tbl);

  return tbl->cn0_ms;
}

/**
 * Get lock detector update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 *
 * \return lock detector update period in ms.
 */
u8 tp_get_ld_ms(tp_tm_e tracking_mode, u8 int_ms)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);

  assert(NULL != tbl);

  return tbl->ld_ms;
}

/**
 * Get false lock (alias) detector update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 *
 * \return false lock (alias) detector update period in ms.
 */
u8 tp_get_alias_ms(tp_tm_e tracking_mode, u8 int_ms)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);

  assert(NULL != tbl);

  return tbl->fl_ms;
}

/**
 * Get FLL discriminator update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 *
 * \return FLL discriminator update period in ms.
 */
u8 tp_get_fll_ms(tp_tm_e tracking_mode, u8 int_ms)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);

  assert(NULL != tbl);

  return tbl->fll_ms;
}


/**
 * Get bit sync update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 * \param[in] int_ms        Tracking sub-mode (integration period).
 *
 * \return bit sync update period in ms.
 */
u8 tp_get_bit_ms(tp_tm_e tracking_mode, u8 int_ms)
{
  const state_table_t *tbl = select_table(tracking_mode, int_ms);

  assert(NULL != tbl);

  return tbl->bit_ms;
}
