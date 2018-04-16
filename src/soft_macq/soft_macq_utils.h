#ifndef SOFT_MACQ_UTILS_H_
#define SOFT_MACQ_UTILS_H_

#include "soft_macq_defines.h"

#define BBNCO_CARRPH_BITS (4)
#define BBNCO_CARRPH_SIZE (1 << BBNCO_CARRPH_BITS)
#define BBNCO_CARRPH_MASK (BBNCO_CARRPH_SIZE - 1)
#define BBLUT_SIZE     \
  (1 << (SAMPLE_BITS + \
         BBNCO_CARRPH_BITS)) /** 256 bytes with 2 bpsamp and 4 bpcarr  */

#define SAMPLE_BITS (2)
#define SAMPLE_MASK ((1 << SAMPLE_BITS) - 1)

#ifdef __cplusplus
extern "C" {
#endif

extern sc16_t bbConvTable[BBLUT_SIZE];

int InitBBConvLut(void);

uint32_t CirclesToUint32(double dCircles);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_MACQ_UTILS_H_ */
