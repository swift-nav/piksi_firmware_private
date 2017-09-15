/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michael Wurm <mwurm@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_dma.h"
#include "registers/swiftnap.h"
#include "zynq7000.h"
#include <hal.h>
#include <stdint.h>
#include <libswiftnav/logging.h>

#define TRACK_DMA_THREAD_PRIO (NORMALPRIO)
#define TRACK_DMA_THREAD_STACK 2000

/** Working area for the track_dma thread. */
static THD_WORKING_AREA(wa_track_dma_thread, TRACK_DMA_THREAD_STACK);


static void interrupts_init(void);
static void pl330_dma_irq_handler(void *context);


static void track_dma_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("track DMA");

  u32 count = 0;
  while(TRUE) {
  	log_info("track DMA thread test output %0d --------------", count++);
  	chThdSleepMilliseconds(2000);
  }
}

void track_dma_init(void) {
  log_info("track DMA thread starting ----------------");

	chThdCreateStatic(wa_track_dma_thread,
					  sizeof(wa_track_dma_thread),
					  TRACK_DMA_THREAD_PRIO,
					  track_dma_thread,
					  NULL);

  log_info("track DMA thread started ------------------");

  // enable clock
  *PL330_APER_CLK_CTRL |= (1 << PL330_DMA_CPU_2XCLKACT_Pos);
  interrupts_init();


  // reset controller (make TrustZone register writeable)
//  *PL330_DMAC_RST_CTRL |= (1 << PL330_DMAC_RST_Pos);

  // enable interrupt 0
  *PL330_INTEN |= (1 << PL330_INTEN_0);

  // enable address increment on both src and dest
//	*PL330_CCR0 |= (1 << PL330_CCR0_SRC_INC_Pos);
//	*PL330_CCR0 |= (1 << PL330_CCR0_DST_INC_Pos);

// 	*PL330_SAR_CH0 = s_addr;
//  *PL330_DAR_CH0 = d_addr;

	/* CAREFUL WITH WRITING THESE REGISTERS - MUST BE WRITTEN IN EXACT SEQUENCE
	 * --> SEE ug585 TRM pg. 1156 */
// 	*PL330_DMA_SRC_ADDR = s_addr;
//	*PL330_DMA_DST_ADDR = d_addr;
// 	*PL330_DMA_SRC_LEN = SWIFTNAP_TRACKING_NUM_READABLE;
// 	*PL330_DMA_DST_LEN = SWIFTNAP_TRACKING_NUM_READABLE;

  *PL330_GIC_DIST_EN = 0;
//	*PL330_ICDICFR2 = 0;
//
//  // disable all spi
//	volatile u32 *gic_disable = PL330_GIC_DISABLE;
//	for(u8 i = 0; i < 3; i++) {
//		*gic_disable = 0xFFFFFFFF;
//		gic_disable += 4;
//	}
//

  u32 regval1 = *PL330_GIC_DIST_EN;

  *PL330_GIC_DIST_EN = (1 << PL330_GIC_EN_INT_Pos);

  u32 regval2 = *PL330_GIC_DIST_EN;
  log_warn("##### Set GIC DIST Enable val1: %x, val2: %x", regval1, regval2);


//
//	*PL330_GIC_CPU_PRIOR = 0xF0U;
  *PL330_GIC_CONTROL = 0x07U;

}

/** PL330 DMA IRQ handler.
 *
 * \param context   Interrupt context (pointer to pl330_dma_dir_driver_t).
 */
static void pl330_dma_irq_handler(void *context) {
  (void)context;

  /* Read interrupt status flags */
  u32 status = *PL330_INT_STATUS;

  log_info("YEEEAAAHH ----------------------------- DMA INTERRUPT CAPTURED !!!! %x", status);

  /* Clear interrupt flags */
  *PL330_INTCLR &= ~(1 << PL330_INTCLR_0);
  gic_irq_pending_clear(IRQ_ID_DMAC_0);
}

/** Initialize interrupts for an PL330 DMA direction driver.
 */
static void interrupts_init(void) {
  gic_handler_register(IRQ_ID_DMAC_0, pl330_dma_irq_handler, NULL);
  gic_irq_sensitivity_set(IRQ_ID_DMAC_0, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(IRQ_ID_DMAC_0, 4);
  gic_irq_enable(IRQ_ID_DMAC_0);
}

void track_dma_start(u32* const s_addr, u32* const d_addr) {
//  (void)s_addr;
//  (void)d_addr;
  char DmaProg[32];
  DmaProg[0]  = 0xBC; // DMAMOV CCR, SS32 DS32
  DmaProg[1]  = 0x01;
  DmaProg[2]  = 0x00;
  DmaProg[3]  = 0x01;
  DmaProg[4]  = 0x40;
  DmaProg[5]  = 0x05;
  DmaProg[6]  = 0xBC; // DMAMOV SAR, <src addr>
  DmaProg[7]  = 0x00;
  DmaProg[8]  = (((u32)s_addr & 0x000000FF) >>  0);
  DmaProg[9]  = (((u32)s_addr & 0x0000FF00) >>  8);
  DmaProg[10] = (((u32)s_addr & 0x00FF0000) >> 16);
  DmaProg[11] = (((u32)s_addr & 0xFF000000) >> 24);
  DmaProg[12] = 0xBC; // DMAMOV DAR, <dst addr>
  DmaProg[13] = 0x02;
  DmaProg[14] = (((u32)d_addr & 0x000000FF) >>  0);
  DmaProg[15] = (((u32)d_addr & 0x0000FF00) >>  8);
  DmaProg[16] = (((u32)d_addr & 0x00FF0000) >> 16);
  DmaProg[17] = (((u32)d_addr & 0xFF000000) >> 24);
  DmaProg[18] = 0x20; // DMALP 12
  DmaProg[19] = 0x0C;
  DmaProg[20] = 0x04; // DMALD
  DmaProg[21] = 0x08; // DMAST
  DmaProg[22] = 0x38; // DMALPEND
  DmaProg[23] = 0x02;
  DmaProg[24] = 0x32; // DMASEV
  DmaProg[25] = 0x00;
  DmaProg[26] = 0x00; // DMAEND

  char DmaGoProg[8];
  DmaGoProg[0] = 0x0A;
  DmaGoProg[1] = 0x00;
  DmaGoProg[2] = (((u32)DmaProg & 0x000000FF) >>  0);
  DmaGoProg[3] = (((u32)DmaProg & 0x0000FF00) >>  8);
  DmaGoProg[4] = (((u32)DmaProg & 0x00FF0000) >> 16);
  DmaGoProg[5] = (((u32)DmaProg & 0xFF000000) >> 24);


  /* CAREFUL WITH WRITING THESE REGISTERS - MUST BE WRITTEN IN EXACT SEQUENCE
   * --> SEE ug585 TRM pg. 1156 */
//  *PL330_DMA_SRC_ADDR = (u32)s_addr;
//  *PL330_DMA_DST_ADDR = (u32)d_addr;
//  *PL330_SAR_CH0 = (u32)s_addr;
//  *PL330_DAR_CH0 = (u32)d_addr;

  u32 DbgInst0 = XDmaPs_DBGINST0(DmaGoProg[1], DmaGoProg[0], 0, 0);
  u32 DbgInst1 = (u32)DmaProg;
  //log_info("### local DBGINST0: 0x%0x DBGINST1: 0x%0x", DbgInst0, DbgInst1);

  *PL330_DBGINST0 = DbgInst0;
  *PL330_DBGINST1 = DbgInst1;
  //log_info("### regs  DBGINST0: 0x%0x DBGINST1: 0x%0x", *PL330_DBGINST0, *PL330_DBGINST1);
  log_warn("___status___ 0x%0x", *PL330_DBGSTATUS);

  volatile u32 counter = 10000;
  while((*PL330_DBGSTATUS & 0x01 /*(1 << PL330_DBGSTATUS_BUSY_Pos)*/) && (counter > 0))
  {
    counter--;
    if(counter == 0)
      log_info("############ DMA TIMEOUT ##############################################");
  }

  *PL330_DBGCMD = 0;

  if(*PL330_DBGSTATUS)
    log_info("status 0x%0x", *PL330_DBGSTATUS);

  static u32 cnt = 0;
  cnt++;
  if(cnt > 3000)
  {
    cnt = 0;
  }
}
