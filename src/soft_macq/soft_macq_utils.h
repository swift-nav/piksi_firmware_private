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

typedef enum _modulation_e { BPSK = 0, BOC_N1 = 1 } modulation_t;

extern sc16_t bbConvTable[BBLUT_SIZE];

int InitBBConvLut(void);
uint32_t CirclesToUint32(double dCircles);
void Sc16ArrayMulX(sc16_t *_pr, sc16_t *_pa, sc16_t *_pb, u32 _iSize);
void Sc16ArrayAddAbsTo(float *_fOut, sc16_t *_fIn, u32 _iSize);
bool IsAcquired3D(const float *vec,
                  const u32 _iCodeSh,
                  const u32 _iFreqSh,
                  const u32 _iNonCoh,
                  float *_fMval,
                  float *_pfCodeMaxI,
                  float *_pfFreqMaxI);
void code_resample(const u8 *const code,
                   const u32 code_len,
                   const u32 code_rate_hz,
                   s8 *upsamp,
                   const u32 upsamp_length,
                   const u32 fs_hz,
                   const modulation_t m);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_MACQ_UTILS_H_ */
