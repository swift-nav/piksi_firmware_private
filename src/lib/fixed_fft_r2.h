#ifndef FIXED_FFT_R2_H_
#define FIXED_FFT_R2_H_

#include <stdint.h>
#include <swiftnav/common.h>

#define MAX_FFTR2_RANKS (14)
#define INTFFT_MAXSIZE (1 << MAX_FFTR2_RANKS)

/** Default complex FFT sample */
typedef struct _sc16 {
  s16 r;
  s16 i;
} sc16_t;

/** Default complex FFT sample */
typedef struct _sc32 {
  s32 r;
  s32 i;
} sc32_t;

/** MACRO called to declare and initialize a FFT */
#define FFT_DECL(SIZE, NAME)                                           \
  struct {                                                             \
    u32 M;                                                             \
    u32 N;                                                             \
    sc16_t *W;                                                         \
    u32 *tmpBRX;                                                       \
    u16 *BR;                                                           \
    u8 uFftMem[(SIZE) * (sizeof(sc16_t) + sizeof(u32) + sizeof(u16))]; \
  } NAME;

#ifdef __cplusplus
extern "C" {
#endif

void InitIntFFTr2(void *_pIntFFT, u32 _N);

void DoFwdIntFFTr2(void *_pIntFFT, sc16_t *_x, u32 _uScale, int _iShuf);

void DoBwdIntFFTr2(void *_pIntFFT, sc16_t *_x, u32 _uScale, int _iShuf);

#ifdef __cplusplus
}
#endif

#endif /* FIXED_FFT_R2_H_ */
