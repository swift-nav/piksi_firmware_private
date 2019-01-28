#ifndef SOFT_MACQ_MAIN_H_
#define SOFT_MACQ_MAIN_H_

#include <swiftnav/common.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

#include "signal_db/signal_db.h"

/** Acquisition CN0 threshold to determine if
 * handover to tracking should be initiated. */
#define ACQ_THRESHOLD 38.0 /* dBHz */

/** Acquisition CN0 threshold to determine if a strong peak has been found.
 * High CN0 triggers early exit from acquisition frequency sweep */
#define ACQ_EARLY_THRESHOLD 39.0 /* dBHz */

/** If handover to tracking fails,
 *  satellite with a high CN0 should be prioritized */
#define ACQ_RETRY_THRESHOLD 39.0 /* dBHz */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  u64 sample_count;
  float cp;
  float df_hz;
  float cn0;
} acq_result_t;

typedef struct {
  float cn0;
  float doppler_hz;
  u32 sample_offset;
} acq_peak_search_t;

bool soft_multi_acq_search(const me_gnss_signal_t _sMeSid,
                           float doppler_min_hz,
                           float doppler_max_hz,
                           acq_result_t *_psAcqResult);

float soft_multi_acq_bin_width(void);

#ifdef __cplusplus
}
#endif

#endif /*  SOFT_MACQ_MAIN_H_ */
