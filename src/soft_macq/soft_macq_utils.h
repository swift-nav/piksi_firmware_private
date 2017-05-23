#ifndef SOFT_MACQ_UTILS_H_
#define SOFT_MACQ_UTILS_H_

#include "soft_macq_defines.h"

#define BBNCO_CARRPH_BITS (4)
#define BBNCO_CARRPH_SIZE (1<<BBNCO_CARRPH_BITS)
#define BBNCO_CARRPH_MASK (BBNCO_CARRPH_SIZE-1)
#define BBLUT_SIZE (1<<(SAMPLE_BITS+BBNCO_CARRPH_BITS)) /** 256 bytes with 2 bpsamp and 4 bpcarr  */

#define SAMPLE_BITS (2)
#define SAMPLE_MASK ((1<<SAMPLE_BITS)-1)

#ifdef __cplusplus
extern "C" {
#endif

extern sc16_t bbConvTable[BBLUT_SIZE];
//~ extern sFauParams_t sPar;

int InitBBConvLut(void);

uint8_t HexToDec(const char cHex);
uint32_t CirclesToUint32(double dCircles);

void Sc16ArrayMulX(sc16_t *_pr, sc16_t *_pa, sc16_t *_pb, int _iSize);
void Fc32ArrayMulX(fc32_t *_pr, fc32_t *_pa, fc32_t *_pb, int _iSize) ;
void Fc32ArrayAddAbsTo(float *_fOut, fc32_t *_fIn, int _iSize);
void Sc16ArrayAddAbsTo(float *_fOut, sc16_t *_fIn, int _iSize);

float FloatMax(float *_pVec, int _iSize);

void Satid2Usi(int *_puUsi, char _sSatId[4]);
void Usi2Satid(char _sSatId[4], int _uUsi);

int IsAcquired3D(float *vec, int _iRows, int _iCols, float *_fMval, int *_iMRval, int *_iMCval);

int CodeGenGps(uint32_t svId, int8_t *code, int iCodeLen);
int CodeGenSbas(uint32_t svId, int8_t *code, int iCodeLen);
int CodeGenGalE1B(int _iSvId, int8_t *piCode, int iCodeLen);
int CodeGenGalE1C(int _iSvId, int8_t *piCode, int iCodeLen);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_MACQ_UTILS_H_ */
