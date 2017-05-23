/*
 * defines.h
 *
 *  Created on: Feb 21, 2017
 *      Author: mic
 */

#ifndef SPECAN_MAIN_H_
#define SPECAN_MAIN_H_

#include <stdint.h>
#include <libswiftnav/time.h>

#define TRACE_SBP_POINTS  (224)

typedef struct _sSpecPayload {
  uint16_t uMsgTag;
  gps_time_t sTime;
  float fStartFreq;
  float fFreqStep;
  float fMinAmpl;
  float fAmplStep;
  uint8_t puValues[TRACE_SBP_POINTS];
} sSpecPayload_t;

int SpecanStart(void);


#endif /* SPECAN_MAIN_H_ */
