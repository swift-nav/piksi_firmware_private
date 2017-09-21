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
#include "pl330.h"
#include "registers/swiftnap.h"
#include "zynq7000.h"
#include <hal.h>
#include <stdint.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/common.h>
#include <string.h>

#define TRACK_DMA_THREAD_PRIO (NORMALPRIO)
#define TRACK_DMA_THREAD_STACK 2000

/** Working area for the track_dma thread. */
static THD_WORKING_AREA(wa_track_dma_thread, TRACK_DMA_THREAD_STACK);

static u32 src_buf[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
static u32 dst_buf[12] = {0};
static struct pl330_transfer_struct pl330;
static u8 progbuf[128] = {0};
static u8 count = 0;

static void interrupts_init(void);
static void pl330_dma_irq_handler(void *context);
//
//static u32 irq_nr = 4711;
//
//void track_dma_set_irq_number(u32 nr) {
//	irq_nr = nr;
//}
//
//static void track_dma_print_irq_number(void) {
//	log_warn("irq number captured: %d", irq_nr);
//}

static void track_dma_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("track DMA");

  while(TRUE) {
    track_dma_start(src_buf, dst_buf);

    if(memcmp(src_buf, dst_buf, sizeof(src_buf)) != 0) {
		log_info("- src \t %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d",
			src_buf[0],
			src_buf[1],
			src_buf[2],
			src_buf[3],
			src_buf[4],
			src_buf[5],
			src_buf[6],
			src_buf[7],
			src_buf[8],
			src_buf[9],
			src_buf[10],
			src_buf[11]);

		log_info("- dest\t %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d",
			dst_buf[0],
			dst_buf[1],
			dst_buf[2],
			dst_buf[3],
			dst_buf[4],
			dst_buf[5],
			dst_buf[6],
			dst_buf[7],
			dst_buf[8],
			dst_buf[9],
			dst_buf[10],
			dst_buf[11]);
	}
    else
    	log_warn("YEEEAAAHH ------- COPY WORKED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    for(u8 i = 0; i < 12; i++)
    	src_buf[i]++;


    if(*PL330_INT_STATUS)
    	log_warn("FIRST interrupt status reg: 0x%0x ----------", *PL330_INT_STATUS);
	log_warn("interrupt event ris reg: 0x%0x ----------", *PL330_INT_EVENT_RIS);
	log_warn("count: %d ----------", count);

//  *PL330_INTCLR = 0xFFFFFFFF;
    	log_warn("interrupt status reg: 0x%0x ----------", *PL330_INT_STATUS);


  	chThdSleepMilliseconds(3000);
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

  pl330.channel_num = PL330_CHANNEL_TO_USE;
  pl330.src_addr = (u32)src_buf;
  pl330.dst_addr = (u32)dst_buf;
  pl330.size_byte = sizeof(src_buf);
  pl330.brst_size = 2;              // NOT CHANGEABLE (AXI 32bit width)  word access (u32, 4 byte)
  pl330.single_brst_size = 2;       // NOT CHANGEABLE (AXI 32bit width) -"-
  pl330.brst_len = 16; 				// num transfers each burst (valid 1-16)
  pl330.peripheral_id = 1;          // #define XPAR_PS7_DMA_S_DEVICE_ID 1
  pl330.enable_cache1 = 0;          // no cache
  pl330.buf = progbuf;              // microcode program buffer
  pl330.buf_size = sizeof(progbuf); // microcode program buffer size

  /* activate DMA peripheral clock */
  *PL330_APER_CLK_CTRL |= (1 << PL330_DMA_CPU_2XCLKACT_Pos);

  pl330_transfer_init(&pl330);


  // clear both bits (set target 'no cpu')
  *PL330_TARGET_REG_TO_USE &= ~(PL330_GIC_TARGET_BOTH_CPU << PL330_TARGET_ID_TO_USE);
  // set respectively
  *PL330_TARGET_REG_TO_USE |= (PL330_TARGET_CPU_TO_USE << PL330_TARGET_ID_TO_USE);

//  *PL330_INTEN |= (1 << PL330_INTEN_7);
  *PL330_INTEN |= (1 << PL330_CHANNEL_TO_USE);
//  *PL330_INTEN = 0xFFFFFFFF;
  *PL330_GIC_DIST_EN |= (1 << PL330_GIC_EN_INT_Pos);
//  *PL330_GIC_ENABLE_SET |= (1 << PL330_GIC_EN_INT_Pos);
  *PL330_GIC_ENABLE_SET1 |= (1 << (PL330_IRQ_ID_TO_USE-32));
  *PL330_GIC_ENABLE_SET0 |= (1 << PL330_CHANNEL_TO_USE);

//  *PL330_GIC_ENABLE_SET0 = 0xFFFFFFFF;
//  *PL330_GIC_ENABLE_SET1 = 0xFFFFFFFF;
//  *PL330_GIC_ENABLE_SET2 = 0xFFFFFFFF;
//  *PL330_GIC_CONTROL |= 0x01;
//  *PL330_INTCLR &= ~(1 << PL330_INTCLR_0);
//  *PL330_INTCLR |= (1 << PL330_CHANNEL_TO_USE);
  *PL330_INTCLR = 0xFFFFFFFF;
  interrupts_init();
}

/** PL330 DMA IRQ handler.
 *
 * \param context   Interrupt context (pointer to pl330_dma_dir_driver_t).
 */
static void pl330_dma_irq_handler(void *context) {
  (void)context;

//  chSysLockFromISR();
//  log_warn("YEEEAAAHH ----------------------------- DMA INTERRUPT CAPTURED !!!!");// %x", status);

  *PL330_INTCLR |= (1 << PL330_CHANNEL_TO_USE);
  /* Clear interrupt flags */
//  gic_irq_pending_clear(IRQ_ID_DMAC_7);
//  gic_irq_pending_clear(PL330_IRQ_ID_TO_USE);
	count++;

//  chSysUnlockFromISR();
}

/** Initialize interrupts for an PL330 DMA direction driver.
 */
static void interrupts_init(void) {
//  gic_handler_register(IRQ_ID_DMAC_7, pl330_dma_irq_handler, NULL);
//  gic_irq_sensitivity_set(IRQ_ID_DMAC_7, IRQ_SENSITIVITY_EDGE);
//  gic_irq_priority_set(IRQ_ID_DMAC_7, 5);
//  gic_irq_enable(IRQ_ID_DMAC_7);//
  gic_handler_register(PL330_IRQ_ID_TO_USE, pl330_dma_irq_handler, NULL);
  gic_irq_sensitivity_set(PL330_IRQ_ID_TO_USE, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(PL330_IRQ_ID_TO_USE, 4);
  gic_irq_enable(PL330_IRQ_ID_TO_USE);
}

void track_dma_start(u32* const s_addr, u32* const d_addr) {
  pl330.src_addr = (u32)s_addr;
  pl330.dst_addr = (u32)d_addr;

  //pl330_transfer_setup_src_dst(&pl330);
  pl330_transfer_start(&pl330);
}
