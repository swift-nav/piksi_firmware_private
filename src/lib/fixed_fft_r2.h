#ifndef FIXED_FFT_R2_H_
#define FIXED_FFT_R2_H_

#include <stdint.h>

#define MAX_FFTR2_RANKS (14)
#define INTFFT_MAXSIZE  (1<<MAX_FFTR2_RANKS)

typedef struct _sc16_t { int16_t r;   int16_t i;   } sc16_t;

/*! \struct _intFFT_t
 *  \brief
 */
typedef struct _intFFTr2_t {
  sc16_t W[INTFFT_MAXSIZE];         /* Twiddle factors table */

  uint16_t BR[INTFFT_MAXSIZE];      /* Bit-reversal */
  uint32_t tmpBRX[INTFFT_MAXSIZE];  /* Shuffle temp array */

  uint32_t N;                       /* FFT length */
  uint32_t M;                       /* FFT order */
} intFFTr2_t;


#ifdef __cplusplus
extern "C" {
#endif


int InitIntFFTr2(intFFTr2_t *_pIntFFT, int _N);

void FreeIntFFTr2(intFFTr2_t *_pIntFFT);

void DoFwdIntFFTr2(intFFTr2_t *_pIntFFT, sc16_t *_x, uint32_t _uScale, int _iShuf);

void DoBwdIntFFTr2(intFFTr2_t *_pIntFFT, sc16_t *_x, uint32_t _uScale, int _iShuf);


#ifdef __cplusplus
}
#endif


#endif /* FIXED_FFT_R2_H_ */
