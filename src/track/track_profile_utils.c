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

#define TP_FLAGS_INIT_DEFAULT \
  (TP_CFLAG_CN0_SET | TP_CFLAG_CN0_USE | \
   TP_CFLAG_EPL_SET | TP_CFLAG_EPL_USE | \
   TP_CFLAG_BSYNC_SET | TP_CFLAG_BSYNC_UPDATE | \
   TP_CFLAG_LD_SET | TP_CFLAG_LD_USE | \
   TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND)

#define TP_FLAGS_DYNAMICS_DEFAULT_FIRST \
  (TP_CFLAG_CN0_SET | \
   TP_CFLAG_EPL_SET | TP_CFLAG_EPL_USE | \
   TP_CFLAG_BSYNC_SET | TP_CFLAG_BSYNC_UPDATE | \
   TP_CFLAG_LD_SET | TP_CFLAG_LD_USE | \
   TP_CFLAG_FLL_SET | TP_CFLAG_FLL_FIRST)

#define TP_FLAGS_DYNAMICS_DEFAULT_SECOND \
  (TP_CFLAG_CN0_ADD | \
   TP_CFLAG_EPL_SET | TP_CFLAG_EPL_USE | \
   TP_CFLAG_BSYNC_SET | TP_CFLAG_BSYNC_UPDATE | \
   TP_CFLAG_LD_SET | TP_CFLAG_LD_USE | \
   TP_CFLAG_FLL_ADD | TP_CFLAG_FLL_SECOND)

#define TP_FLAGS_SHORT_DEFAULT \
  (TP_CFLAG_SHORT_CYCLE | \
   TP_CFLAG_CN0_SET | \
   TP_CFLAG_EPL_SET | \
   TP_CFLAG_BSYNC_SET | \
   TP_CFLAG_LD_SET | \
   TP_CFLAG_FLL_SET)

#define TP_FLAGS_5MS1PN_LONG_DEFAULT_FIRST \
  (TP_CFLAG_LONG_CYCLE | \
   TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_FIRST | \
   TP_CFLAG_CN0_ADD | TP_CFLAG_CN0_USE | \
   TP_CFLAG_EPL_ADD | TP_CFLAG_EPL_USE | \
   TP_CFLAG_BSYNC_ADD | TP_CFLAG_BSYNC_UPDATE | \
   TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE | \
   TP_CFLAG_FLL_ADD | TP_CFLAG_FLL_FIRST)

#define TP_FLAGS_5MS1PN_LONG_DEFAULT_SECOND \
  (TP_CFLAG_LONG_CYCLE | \
   TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_SECOND | \
   TP_CFLAG_CN0_ADD | TP_CFLAG_CN0_USE | \
   TP_CFLAG_EPL_ADD | TP_CFLAG_EPL_USE | \
   TP_CFLAG_BSYNC_ADD | TP_CFLAG_BSYNC_UPDATE | \
   TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE | \
   TP_CFLAG_FLL_ADD | TP_CFLAG_FLL_SECOND | TP_CFLAG_FLL_USE)

#define TP_FLAGS_10MS1PN_LONG_DEFAULT_FIRST \
  (TP_CFLAG_LONG_CYCLE | \
   TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_FIRST | \
   TP_CFLAG_CN0_ADD | \
   TP_CFLAG_EPL_ADD | \
   TP_CFLAG_BSYNC_ADD | TP_CFLAG_BSYNC_UPDATE | \
   TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE | \
   TP_CFLAG_FLL_ADD | TP_CFLAG_FLL_FIRST)

#define TP_FLAGS_10MS1PN_LONG_DEFAULT_SECOND \
  (TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_SECOND | \
   TP_CFLAG_CN0_ADD | TP_CFLAG_CN0_USE | \
   TP_CFLAG_EPL_ADD | TP_CFLAG_EPL_USE | \
   TP_CFLAG_BSYNC_ADD | TP_CFLAG_BSYNC_UPDATE | \
   TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE | \
   TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND | TP_CFLAG_FLL_USE)

/**
 * State entry.
 */
typedef struct {
  u8  state_ms;  /**< State integration time */
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
  u8 flld_ms; /**< FLL discriminator integration time */
  u8 flll_ms; /**< FLL loop integration time */
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
  .flld_ms = 10,
  .flll_ms = 10,
  .bit_ms  = 1,
  .ent_cnt = 20,
  .entries = {
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT | TP_CFLAG_FLL_USE},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT},
    {1, TP_FLAGS_INIT_DEFAULT | TP_CFLAG_FLL_USE},
  }
};

/**
 * Dynamics tracking mode (bit sync, FLL-assisted PLL, 1 ms)
 */
static const state_table_t mode_1msDYN = {
  .int_ms  = 1,
  .cn0_ms  = 10,
  .ld_ms   = 1,
  .fl_ms   = 1,
  .flld_ms = 9,
  .flll_ms = 10,
  .bit_ms  = 1,
  .ent_cnt = 20,
  .entries = {
    {1, TP_FLAGS_DYNAMICS_DEFAULT_FIRST},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND | TP_CFLAG_CN0_USE | TP_CFLAG_FLL_USE},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_FIRST},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND},
    {1, TP_FLAGS_DYNAMICS_DEFAULT_SECOND | TP_CFLAG_CN0_USE | TP_CFLAG_FLL_USE},
  }
};

/**
 * 5 ms integrations; 1+N mode.
 */
static const state_table_t mode_5ms1PN = {
  .int_ms  = 5,
  .cn0_ms  = 5,
  .ld_ms   = 5,
  .fl_ms   = 5,
  .flld_ms = 5,
  .flll_ms = 10,
  .bit_ms  = 5,
  .ent_cnt = 8,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT | TP_CFLAG_ALIAS_SET },
    { 4, TP_FLAGS_5MS1PN_LONG_DEFAULT_FIRST },
    { 1, TP_FLAGS_SHORT_DEFAULT | TP_CFLAG_ALIAS_ADD },
    { 4, TP_FLAGS_5MS1PN_LONG_DEFAULT_SECOND },
    { 1, TP_FLAGS_SHORT_DEFAULT | TP_CFLAG_ALIAS_SET },
    { 4, TP_FLAGS_5MS1PN_LONG_DEFAULT_FIRST },
    { 1, TP_FLAGS_SHORT_DEFAULT | TP_CFLAG_ALIAS_ADD },
    { 4, TP_FLAGS_5MS1PN_LONG_DEFAULT_SECOND },
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
  .flld_ms = 5,
  .flll_ms = 10,
  .bit_ms  = 5,
  .ent_cnt = 6,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT | TP_CFLAG_ALIAS_SET },
    { 4, TP_FLAGS_10MS1PN_LONG_DEFAULT_FIRST },
    { 5, TP_FLAGS_10MS1PN_LONG_DEFAULT_SECOND },
    { 1, TP_FLAGS_SHORT_DEFAULT | TP_CFLAG_ALIAS_SET },
    { 4, TP_FLAGS_10MS1PN_LONG_DEFAULT_FIRST },
    { 5, TP_FLAGS_10MS1PN_LONG_DEFAULT_SECOND },
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
  .flld_ms = 15,
  .flll_ms = 20,
  .bit_ms  = 5,
  .ent_cnt = 5,
  .entries = {
    { 1, TP_FLAGS_SHORT_DEFAULT | TP_CFLAG_ALIAS_SET },
    { 4,
      (TP_CFLAG_LONG_CYCLE |
       TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_FIRST |
       TP_CFLAG_CN0_ADD |
       TP_CFLAG_EPL_ADD |
       TP_CFLAG_BSYNC_ADD | TP_CFLAG_BSYNC_UPDATE |
       TP_CFLAG_LD_ADD | TP_CFLAG_LD_USE |
       TP_CFLAG_FLL_ADD | TP_CFLAG_FLL_FIRST)
    },
    { 5,
      (TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_SECOND |
       TP_CFLAG_CN0_ADD |
       TP_CFLAG_EPL_ADD |
       TP_CFLAG_BSYNC_SET | TP_CFLAG_BSYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE |
       TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND)
    },
    { 5,
      (TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_SECOND |
       TP_CFLAG_CN0_ADD |
       TP_CFLAG_EPL_ADD |
       TP_CFLAG_BSYNC_SET | TP_CFLAG_BSYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE |
       TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND)
    },
    { 5,
      (TP_CFLAG_ALIAS_ADD | TP_CFLAG_ALIAS_SECOND |
       TP_CFLAG_CN0_ADD | TP_CFLAG_CN0_USE |
       TP_CFLAG_EPL_ADD |TP_CFLAG_EPL_USE |
       TP_CFLAG_BSYNC_SET | TP_CFLAG_BSYNC_UPDATE |
       TP_CFLAG_LD_SET | TP_CFLAG_LD_USE  |
       TP_CFLAG_FLL_SET | TP_CFLAG_FLL_SECOND | TP_CFLAG_FLL_USE)
    },
  }
};

/**
 * Helper for locating tracker state table.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return Table pointer or NULL on error.
 */
static const state_table_t *select_table(tp_tm_e tracking_mode)
{
  switch (tracking_mode) {
  case TP_TM_INITIAL:
    return &mode_1msINI;

  case TP_TM_5MS:
    return &mode_5ms1PN;

  case TP_TM_DYN:
    return &mode_1msDYN;

  case TP_TM_10MS:
    return &mode_10ms1PN5;

  case TP_TM_20MS:
    return &mode_20ms1PN5;

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
 * \param[in] cycle_no      Cycle number.
 *
 * \return Number of cycles in the given mode
 */
u8 tp_next_cycle_counter(tp_tm_e tracking_mode,
                         u8 cycle_no)
{
  u8 cycle_cnt; /**< Number of cycles in the current tracking mode */

  cycle_cnt = tp_get_cycle_count(tracking_mode);

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
 * \param[in] cycle_no      Cycle number.
 */
u32 tp_get_cycle_flags(tp_tm_e tracking_mode,
                       u8 cycle_no)
{
  const state_table_t *tbl = select_table(tracking_mode);
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
u8 tp_get_cycle_count(tp_tm_e tracking_mode)
{
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
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode,
                                 u8 cycle_no)
{
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
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode,
                                   u8 cycle_no)
{
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
u8 tp_get_cn0_ms(tp_tm_e tracking_mode)
{
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
u8 tp_get_ld_ms(tp_tm_e tracking_mode)
{
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->ld_ms;
}

/**
 * Get false lock (alias) detector update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return false lock (alias) detector update period in ms.
 */
u8 tp_get_alias_ms(tp_tm_e tracking_mode)
{
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->fl_ms;
}

/**
 * Get FLL discriminator update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return FLL discriminator update period in ms.
 */
u8 tp_get_flld_ms(tp_tm_e tracking_mode)
{
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->flld_ms;
}

/**
 * Get FLL loop update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return FLL loop update period in ms.
 */
u8 tp_get_flll_ms(tp_tm_e tracking_mode)
{
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->flll_ms;
}


/**
 * Get bit sync update period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return bit sync update period in ms.
 */
u8 tp_get_bit_ms(tp_tm_e tracking_mode)
{
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->bit_ms;
}

/**
 * Get PLL integration period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return PLL integration period in ms.
 */
u8 tp_get_pll_ms(tp_tm_e tracking_mode)
{
  const state_table_t *tbl = select_table(tracking_mode);

  assert(NULL != tbl);

  return tbl->int_ms;
}

/**
 * Get DLL integration period in ms.
 *
 * \param[in] tracking_mode Tracking mode.
 *
 * \return DLL integration period in ms.
 */
u8 tp_get_dll_ms(tp_tm_e tracking_mode)
{
  return tp_get_pll_ms(tracking_mode);
}

/**
 * Returns a literal for the given mode enumeration.
 *
 * @param[in] v Tracking mode
 *
 * @return Mode literal
 */
const char *tp_get_mode_str(tp_tm_e v)
{
  const char *str = "?";
  switch (v) {
  case TP_TM_INITIAL: str = "TM_INI"; break;
  case TP_TM_DYN:     str = "TM_DYN"; break;
  case TP_TM_5MS:     str = "TM_5MS"; break;
  case TP_TM_10MS:    str = "TM_10MS"; break;
  case TP_TM_20MS:    str = "TM_20MS"; break;
  default: assert(false);
  }
  return str;
}

/**
 * Test if the tracker controller is PLL or FLL assisted PLL
 *
 * \param[in] ctrl Tracker controller type.
 *
 * \retval true  controller is PLL or FLL assisted FLL
 * \retval false controller is not PLL
 */
bool tp_is_pll_ctrl(tp_ctrl_e ctrl)
{
  switch (ctrl) {
  case TP_CTRL_PLL2:
  case TP_CTRL_PLL3:
    return true;

  case TP_CTRL_FLL1:
  case TP_CTRL_FLL2:
    return false;

  default:
    assert(!"Unsupported controller type");
    break;
  }
  return false;
}

/**
 * Test if the tracker controller is FLL.
 *
 * \param[in] ctrl Tracker controller type.
 *
 * \retval true  controller is FLL
 * \retval false controller is not FLL
 */
bool tp_is_fll_ctrl(tp_ctrl_e ctrl)
{
  switch (ctrl) {
  case TP_CTRL_PLL2:
  case TP_CTRL_PLL3:
    return false;

  case TP_CTRL_FLL1:
  case TP_CTRL_FLL2:
    return true;

  default:
    assert(!"Unsupported controller type");
    break;
  }
  return false;
}

/**
 * Sets and clears the L1 & L2 xcorr_suspect flag.
 *
 * This function checks if the xcorr_suspect status has changed for the signal,
 * and sets / clears the flag respectively.
 *
 * \param[in]     channel_info      Channel information.
 * \param[in,out] common_data       Channel data.
 * \param[in,out] input             Common L1 or L2 tracker data.
 * \param[in]     xcorr_suspect     Flag indicating if signal is xcorr suspect.
 * \param[in]     sensitivity_mode  Flag indicating sensitivity mode.
 *
 * \return None
 */
void set_xcorr_suspect_flag(const tracker_channel_info_t *channel_info,
                            tracker_common_data_t *common_data,
                            void *input,
                            bool xcorr_suspect,
                            bool sensitivity_mode)
{
  if (CODE_GPS_L1CA == channel_info->mesid.code) {
    gps_l1ca_tracker_data_t *data;
    data = (gps_l1ca_tracker_data_t*) input;
    if ((data->xcorr_flag) == xcorr_suspect) {
      return;
    }
    data->xcorr_flag = xcorr_suspect;
  } else {
    gps_l2cm_tracker_data_t *data;
    data = (gps_l2cm_tracker_data_t*) input;
    if ((data->xcorr_flag) == xcorr_suspect) {
      return;
    }
    data->xcorr_flag = xcorr_suspect;
  }

  if (xcorr_suspect) {
    common_data->flags |= TRACK_CMN_FLAG_XCORR_SUSPECT;
    if (!sensitivity_mode) {
      log_debug_mesid(channel_info->mesid,
                      "setting cross-correlation suspect flag");
    }
  } else {
    common_data->flags &= ~TRACK_CMN_FLAG_XCORR_SUSPECT;
    if (!sensitivity_mode) {
      log_debug_mesid(channel_info->mesid,
                      "clearing cross-correlation suspect flag");
    }
  }
  common_data->xcorr_change_count = common_data->update_count;
}
