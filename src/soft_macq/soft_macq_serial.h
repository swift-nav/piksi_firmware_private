#ifndef SOFT_MACQ_SERIAL_H_
#define SOFT_MACQ_SERIAL_H_

#include "soft_macq_defines.h"
#include "soft_macq_main.h"

#ifdef __cplusplus
extern "C" {
#endif

float soft_acq_bin_width(void);

bool soft_acq_search(const sc16_t *_cSignal,
                     const me_gnss_signal_t mesid,
                     float cf_min,
                     float cf_max,
                     float cf_bin_width,
                     acq_result_t *acq_result);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_MACQ_SERIAL_H_ */
