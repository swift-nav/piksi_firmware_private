/*
 * main.c
 *
 *  Created on: Feb 15, 2017
 *      Author: mic
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h>
#include <swiftnav/common.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>

#include <libsbp/piksi.h>
#include <sbp.h>
#include "board/v3/nap/grabber.h"
#include "board/v3/nap/nap_constants.h"
#include "settings/settings_api.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

#include "lib/fixed_fft_r2.h"
#include "specan_main.h"

#define SPECAN_THREAD_STACK (4 * 1024)
#define SPECAN_THREAD_PRIORITY (LOWPRIO + 2)

#define SPECAN_SAMPLING_FREQ (NAP_FRONTEND_RAW_SAMPLE_RATE_Hz)
#define SPECAN_FFT_SIZE (1024)
#define SPECAN_AVGCOUNT (100)
#define SPECAN_NUMLINES (SPECAN_FFT_SIZE / 2)
#define TRACE_ARESOLUTION (255)

#define SPECAN_GRABBER_LENGTH (FIXED_GRABBER_LENGTH)
#define SPECAN_BBSAMPLES (SPECAN_GRABBER_LENGTH)

#if SPECAN_FFT_SIZE > SPECAN_GRABBER_LENGTH
#error "SPECAN_FFT_SIZE shouldn't be greater than SPECAN_GRABBER_LENGTH"
#endif

static uint8_t *pSampleBuf;
static sc16_t pBaseBand[SPECAN_BBSAMPLES];
static sc16_t pTmpTrace[SPECAN_FFT_SIZE];
static uint8_t uCoeff[SPECAN_FFT_SIZE];
static bool run_spectrum;

static struct {
  msg_specan_t header;
  u8 payload[TRACE_SBP_POINTS];
} curr_trace;

static FFT_DECL(SPECAN_FFT_SIZE, sFFT);

static float pSpecTrace[SPECAN_NUMLINES];
static uint32_t uTraceStep;

static void SpecanCore(uint8_t _uWhichBand);

THD_WORKING_AREA(wa_manage_specan_thread, SPECAN_THREAD_STACK);
void ThreadManageSpecan(void *arg) {
  (void)arg;
  uint8_t uBand;
  uint32_t um, k, uMsgLength, uNumPoints;
  float fMinAmpl, fMaxAmpl;
  uint32_t uBuffLen;

  /* fecth utility pointer */
  msg_specan_t *p_head = &(curr_trace.header);

  chRegSetThreadName("spectrum analyzer");
  /* At least at startup in clear sky we want the acquisition not to
   * be interfered with, later on they will have to contend the CPU as
   * they both are low priority tasks
   * */
  while (TRUE) {
    chThdSleepMilliseconds(500);
    if (!run_spectrum) continue;

    pSampleBuf = GrabberGetBufferPt(&uBuffLen);
    if (NULL == pSampleBuf) {
      log_error("GrabberGetBufferPt() failed in spectrum analyzer");
      continue;
    }

    /* TODO *
     * replace this with the current snapshot timer */
    gps_time_t curr_time = get_current_time();

    for (uBand = 1; uBand <= 4; uBand++) {
      p_head->channel_tag = uBand;
      p_head->t.wn = curr_time.wn;
      p_head->t.tow = curr_time.tow;
      p_head->t.ns_residual = 0;
      /** The call below sets start frequency and frequency step */
      SpecanCore(uBand);

      /** split trace in SBP_MSG_SPECAN and send out */
      for (um = 0; um < SPECAN_NUMLINES; um += TRACE_SBP_POINTS) {
        /** cap the number of points to TRACE_SBP_POINTS */
        uNumPoints = ((um + TRACE_SBP_POINTS) <= SPECAN_NUMLINES)
                         ? (TRACE_SBP_POINTS)
                         : (SPECAN_NUMLINES - um);
        uMsgLength = sizeof(msg_specan_t) + uNumPoints;

        /** find min and max amplitude */
        fMinAmpl = fMaxAmpl = pSpecTrace[um];
        for (k = 1; k < uNumPoints; k++) {
          if (fMaxAmpl < pSpecTrace[um + k]) fMaxAmpl = pSpecTrace[um + k];
          if (fMinAmpl > pSpecTrace[um + k]) fMinAmpl = pSpecTrace[um + k];
        }
        /** resulting minimum amplitude and amplitude step */
        p_head->amplitude_ref = fMinAmpl;
        p_head->amplitude_unit = (fMaxAmpl - fMinAmpl) / (TRACE_ARESOLUTION);

        /** scale amplitude points to uint8_t */
        for (k = 0; k < uNumPoints; k++) {
          p_head->amplitude_value[k] = (uint8_t)floorf(
              (pSpecTrace[um + k] - fMinAmpl) / (p_head->amplitude_unit));
        }

        /** send this SBP message */
        sbp_send_msg(SBP_MSG_SPECAN, uMsgLength, (uint8_t *)&curr_trace);

        /** update starting frequency before sending next message... */
        p_head->freq_ref = p_head->freq_ref + uNumPoints * p_head->freq_step;

        /** */
        chThdSleepMilliseconds(1);
      } /* for user message */
      chThdSleepMilliseconds(200);
    } /* for band=1:4 */
  }
}

int SpecanStart(void) {
  uint32_t k;
  const float fTwoPI = 2.0 * M_PI;

  SETTING(
      "system_monitor", "spectrum_analyzer", run_spectrum, SETTINGS_TYPE_BOOL);

  for (k = 0; k < SPECAN_FFT_SIZE; k++) {
    /* uCoeff[k] = MIN(MIN(k+1, SPECAN_FFT_SIZE-k), 32); */
    uCoeff[k] = rintf(32.0 * (1.0 - cosf(k * fTwoPI / SPECAN_FFT_SIZE)));
  }

  InitIntFFTr2(&sFFT, SPECAN_FFT_SIZE);

  chThdCreateStatic(wa_manage_specan_thread,
                    sizeof(wa_manage_specan_thread),
                    SPECAN_THREAD_PRIORITY,
                    ThreadManageSpecan,
                    NULL);
  return 0;
}

static void Sca16AddAbssqTo(float *_fOut, sc16_t *_fIn, int _iSize);

static void SpecanCore(uint8_t _uWhichBand) {
  uint32_t k, h;
  int16_t iSignMagLut[4] = {+1, +3, -1, -3};
  uint32_t uFftScale = 0x0;
  uint32_t uFftStartPt, uTraceStart = 0;
  float fStartFreq;
  const float fFrontEndSpms = NAP_FRONTEND_RAW_SAMPLE_RATE_Hz * 1e-6;
  const float fFreqNco1 = 1590.000;
  const float fFreqNco2 = 1235.000;

  switch (_uWhichBand) {
    case 1:
      for (k = 0; k < SPECAN_GRABBER_LENGTH; k++) {
        pBaseBand[k].r = iSignMagLut[((pSampleBuf[k] >> 0) & 0x3)];
        pBaseBand[k].i = 0;
      }
      uTraceStart = 0;
      fStartFreq = fFreqNco1;
      break;
    case 2:
      for (k = 0; k < SPECAN_GRABBER_LENGTH; k++) {
        pBaseBand[k].r = iSignMagLut[((pSampleBuf[k] >> 2) & 0x3)];
        pBaseBand[k].i = 0;
      }
      uTraceStart = SPECAN_FFT_SIZE / 2;
      fStartFreq = fFreqNco1 - fFrontEndSpms / 2;
      break;
    case 3:
      for (k = 0; k < SPECAN_GRABBER_LENGTH; k++) {
        pBaseBand[k].r = iSignMagLut[((pSampleBuf[k] >> 4) & 0x3)];
        pBaseBand[k].i = 0;
      }
      uTraceStart = 0;
      fStartFreq = fFreqNco2;
      break;
    case 4:
      for (k = 0; k < SPECAN_GRABBER_LENGTH; k++) {
        pBaseBand[k].r = iSignMagLut[((pSampleBuf[k] >> 6) & 0x3)];
        pBaseBand[k].i = 0;
      }
      uTraceStart = SPECAN_FFT_SIZE / 2;
      fStartFreq = fFreqNco2 - fFrontEndSpms / 2;
      break;
    default:
      break;
  }

  curr_trace.header.freq_ref = fStartFreq;
  curr_trace.header.freq_step = fFrontEndSpms / SPECAN_FFT_SIZE;

  /* add abs(FFT) */
  memset(pSpecTrace, 0, sizeof(float) * SPECAN_NUMLINES);
  for (k = 0; k < SPECAN_AVGCOUNT; k++) {
    /* chose a co-prime factor so I am sure to scan all possible start points
     * eventually */
    uFftStartPt =
        ((uTraceStep++) * 1331) % (SPECAN_BBSAMPLES - SPECAN_FFT_SIZE);
    MEMCPY_S(pTmpTrace,
             sizeof(pTmpTrace),
             pBaseBand + uFftStartPt,
             sizeof(sc16_t) * SPECAN_FFT_SIZE);
    /* perform the FFT of the input data, no scaling */
    for (h = 0; h < SPECAN_FFT_SIZE; h++) {
      pTmpTrace[h].r = pTmpTrace[h].r * uCoeff[h];
    }
    DoFwdIntFFTr2(&sFFT, pTmpTrace, uFftScale, 1);
    Sca16AddAbssqTo(pSpecTrace, pTmpTrace + uTraceStart, SPECAN_NUMLINES);
  }

  /* Logarithmic scale */
  for (k = 0; k < SPECAN_NUMLINES; k++) {
    pSpecTrace[k] = -80 + 10 * log10f(pSpecTrace[k]);
    /* yep, that's a magic number there.. a waveform generator anybody? */
  }
}

static void Sca16AddAbssqTo(float *_fOut, sc16_t *_fIn, int _iSize) {
  int i;
  if (NULL == _fOut) return;
  if (NULL == _fIn) return;
  if (_iSize <= 0) return;

  for (i = 0; i < _iSize; i++) {
    _fOut[i] +=
        (((float)_fIn[i].r * _fIn[i].r) + ((float)_fIn[i].i * _fIn[i].i));
  }
}
