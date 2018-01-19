#ifndef SOFT_MACQ_MDBZP_H_
#define SOFT_MACQ_MDBZP_H_

#include "soft_macq_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

bool mdbzp_static( sc16_t *_cSignal, int8_t *_piCode, sFauParams_t *_pPar, acqResults_t *_pRes );

#ifdef __cplusplus
}
#endif

#endif /* SOFT_MACQ_MDBZP_H_ */
