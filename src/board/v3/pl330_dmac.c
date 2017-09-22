/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Michael Wurm <mwurm@swift-nav.com>
 *          Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "pl330_dmac.h"

#include <libswiftnav/logging.h>

#define PL330_BASE (0xF8003000U)

#define PL330_BURST_SIZE (4)
#define PL330_BURST_LENGTH (16)

#define PL330_MAX_CHANNELS (8)
#define PL330_TIMEOUT_LOOPS (10000)

#define PL330_STATE_STOPPED (0x0000)
#define PL330_STATE_EXECUTING (0x0001)
#define PL330_STATE_WFE (0x0004)
#define PL330_STATE_FAULTING (0x000f)
#define PL330_STATE_COMPLETING (0x0010)
#define PL330_STATE_WFP (0x0020)
#define PL330_STATE_KILLING (0x0040)
#define PL330_STATE_FAULT_COMPLETING (0x0080)
#define PL330_STATE_CACHEMISS (0x0100)
#define PL330_STATE_UPDTPC (0x0200)
#define PL330_STATE_ATBARRIER (0x400)
#define PL330_STATE_QUEUEBUSY (0x800)
#define PL330_STATE_INVALID (0x8000)

/* DMA Status */
#define DS 0x0
#define DS_ST_STOP 0x0
#define DS_ST_EXEC 0x1
#define DS_ST_CMISS 0x2
#define DS_ST_UPDTPC 0x3
#define DS_ST_WFE 0x4
#define DS_ST_ATBRR 0x5
#define DS_ST_QBUSY 0x6
#define DS_ST_WFP 0x7
#define DS_ST_KILL 0x8
#define DS_ST_CMPLT 0x9
#define DS_ST_FLTCMP 0xe
#define DS_ST_FAULT 0xf

/* DMA Program Count register */
#define DPC 0x04
/* Interrupt Enable register */
#define INTEN 0x20
/* Event-Interrupt Raw Status register */
#define ES 0x24
/* Interrupt Status register */
#define INTSTATUS 0x28
/* Interrupt Clear register */
#define INTCLR 0x2c
/* Fault Status DMA Manager register */
#define FSM 0x30
/* Fault Status DMA Channel register */
#define FSC 0x34
/* Fault Type DMA Manager register */
#define FTM 0x38

/* Fault Type DMA Channel register */
#define _FTC 0x040
#define FTC(n) (_FTC + (n)*0x4)

/* Channel Status register */
#define _CS 0x100
#define CS(n) (_CS + (n)*0x8)
#define CS_CNS (1 << 21)

/* Channel Program Counter register */
#define _CPC 0x104
#define CPC(n) (_CPC + (n)*0x8)

/* Source Address register */
#define _SA 0x400
#define SA(n) (_SA + (n)*0x20)

/* Destination Address register */
#define _DA 0x404
#define DA(n) (_DA + (n)*0x20)

/* Channel Control register */
#define _CC 0x408
#define CC(n) (_CC + (n)*0x20)

/* Channel Control register (CCR) Setting */
#define CC_SRCINC (1 << 0)
#define CC_DSTINC (1 << 14)
#define CC_SRCPRI (1 << 8)
#define CC_DSTPRI (1 << 22)
#define CC_SRCNS (1 << 9)
#define CC_DSTNS (1 << 23)
#define CC_SRCIA (1 << 10)
#define CC_DSTIA (1 << 24)
#define CC_SRCBRSTLEN_SHIFT 4
#define CC_DSTBRSTLEN_SHIFT 18
#define CC_SRCBRSTSIZE_SHIFT 1
#define CC_DSTBRSTSIZE_SHIFT 15
#define CC_SRCCCTRL_SHIFT 11
#define CC_SRCCCTRL_MASK 0x7
#define CC_DSTCCTRL_SHIFT 25
#define CC_DRCCCTRL_MASK 0x7
#define CC_SWAP_SHIFT 28

/* Loop Counter 0 register */
#define _LC0 0x40c
#define LC0(n) (_LC0 + (n)*0x20)

/* Loop Counter 1 register */
#define _LC1 0x410
#define LC1(n) (_LC1 + (n)*0x20)

/* Debug Status register */
#define DBGSTATUS 0xd00
#define DBG_BUSY (1 << 0)

/* Debug Command register */
#define DBGCMD 0xd04
/* Debug Instruction 0 register */
#define DBGINST0 0xd08
/* Debug Instruction 1 register */
#define DBGINST1 0xd0c

/* Configuration register */
#define CR0 0xe00
#define CR1 0xe04
#define CR2 0xe08
#define CR3 0xe0c
#define CR4 0xe10
#define CRD 0xe14

/* Peripheral Identification register */
#define PERIPH_ID 0xfe0
/* Component Identification register */
#define PCELL_ID 0xff0

/* Configuration register value */
#define CR0_PERIPH_REQ_SET (1 << 0)
#define CR0_BOOT_EN_SET (1 << 1)
#define CR0_BOOT_MAN_NS (1 << 2)
#define CR0_NUM_CHANS_SHIFT 4
#define CR0_NUM_CHANS_MASK 0x7
#define CR0_NUM_PERIPH_SHIFT 12
#define CR0_NUM_PERIPH_MASK 0x1f
#define CR0_NUM_EVENTS_SHIFT 17
#define CR0_NUM_EVENTS_MASK 0x1f

/* Configuration register value */
#define CR1_ICACHE_LEN_SHIFT 0
#define CR1_ICACHE_LEN_MASK 0x7
#define CR1_NUM_ICACHELINES_SHIFT 4
#define CR1_NUM_ICACHELINES_MASK 0xf

/* Configuration register value */
#define CRD_DATA_WIDTH_SHIFT 0
#define CRD_DATA_WIDTH_MASK 0x7
#define CRD_WR_CAP_SHIFT 4
#define CRD_WR_CAP_MASK 0x7
#define CRD_WR_Q_DEP_SHIFT 8
#define CRD_WR_Q_DEP_MASK 0xf
#define CRD_RD_CAP_SHIFT 12
#define CRD_RD_CAP_MASK 0x7
#define CRD_RD_Q_DEP_SHIFT 16
#define CRD_RD_Q_DEP_MASK 0xf
#define CRD_DATA_BUFF_SHIFT 20
#define CRD_DATA_BUFF_MASK 0x3ff

/* Opcode value */
#define CMD_DMAADDH 0x54
#define CMD_DMAEND 0x00
#define CMD_DMAFLUSHP 0x35
#define CMD_DMAGO 0xa0
#define CMD_DMALD 0x04
#define CMD_DMALDP 0x25
#define CMD_DMALP 0x20
#define CMD_DMALPEND 0x28
#define CMD_DMAKILL 0x01
#define CMD_DMAMOV 0xbc
#define CMD_DMANOP 0x18
#define CMD_DMARMB 0x12
#define CMD_DMASEV 0x34
#define CMD_DMAST 0x08
#define CMD_DMASTP 0x29
#define CMD_DMASTZ 0x0c
#define CMD_DMAWFE 0x36
#define CMD_DMAWFP 0x30
#define CMD_DMAWMB 0x13

/* Opcode size */
#define SZ_DMAADDH 3
#define SZ_DMAEND 1
#define SZ_DMAFLUSHP 2
#define SZ_DMALD 1
#define SZ_DMALDP 2
#define SZ_DMALP 2
#define SZ_DMALPEND 2
#define SZ_DMAKILL 1
#define SZ_DMAMOV 6
#define SZ_DMANOP 1
#define SZ_DMARMB 1
#define SZ_DMASEV 2
#define SZ_DMAST 1
#define SZ_DMASTP 2
#define SZ_DMASTZ 1
#define SZ_DMAWFE 2
#define SZ_DMAWFP 2
#define SZ_DMAWMB 1
#define SZ_DMAGO 6

#define UNTIL(t, s)                   \
  while (!(pl330_get_state(t) & (s))) \
    ;

typedef enum {
  SAR = 0,
  CCR,
  DAR,
} dmamov_dst_t;

typedef enum {
  SINGLE,
  BURST,
  ALWAYS,
} pl330_cond_t;

typedef struct {
  pl330_cond_t cond;
  u8 forever;
  u8 loop;
  u8 bjump;
} arg_LPEND_t;

typedef struct {
  u8 ns;
  u8 chan;
  u32 addr;
} arg_GO_t;

typedef enum {
  SCCTRL0 = 0, /* Noncacheable and nonbufferable */
  SCCTRL1,     /* Bufferable only */
  SCCTRL2,     /* Cacheable, but do not allocate */
  SCCTRL3,     /* Cacheable and bufferable, but do not allocate */
  SINVALID1,   /* Invalid */
  SINVALID2,   /* INvalid */
  SCCTRL6,     /* Cacheable write-through, allocate on reads only */
  SCCTRL7      /* Cacheable write-back, allocate on reads only */
} srccachectrl_t;

typedef enum {
  DCCTRL0 = 0,   /* Noncacheable and nonbufferable */
  DCCTRL1,       /* Bufferable only */
  DCCTRL2,       /* Cacheable, but do not allocate */
  DCCTRL3,       /* Cacheable and bufferable, but do not allocate */
  DINVALID1 = 8, /* Invalid */
  DINVALID2,     /* Invalid */
  DCCTRL6,       /* Cacheable write-through, allocate on writes only */
  DCCTRL7        /* Cacheable write-back, allocate on writes only */
} dstcachectrl_t;

typedef enum {
  SWAP_NO = 0,
  SWAP_2,
  SWAP_4,
  SWAP_8,
  SWAP_16,
} byteswap_t;

typedef struct {
  u8 dst_inc : 1;
  u8 src_inc : 1;
  u8 nonsecure;
  u8 privileged;
  u8 insaccess;
  u8 brst_len : 5;
  u8 brst_size : 3; /* power of 2 */
  dstcachectrl_t dcctl;
  srccachectrl_t scctl;
  byteswap_t swap;
} reqcfg_t;

static inline u32 emit_END(u8 buf[]) {
  buf[0] = CMD_DMAEND;
  return SZ_DMAEND;
}

static inline u32 emit_FLUSHP(u8 buf[], u8 peri) {
  buf[0] = CMD_DMAFLUSHP;
  peri &= 0x1f;
  peri <<= 3;
  buf[1] = peri;
  return SZ_DMAFLUSHP;
}

static inline u32 emit_LD(u8 buf[], pl330_cond_t cond) {
  buf[0] = CMD_DMALD;
  if (cond == SINGLE)
    buf[0] |= (0 << 1) | (1 << 0);
  else if (cond == BURST)
    buf[0] |= (1 << 1) | (1 << 0);
  return SZ_DMALD;
}

static inline u32 emit_LP(u8 buf[], u8 loop, u16 cnt) {
  buf[0] = CMD_DMALP;
  if (loop) buf[0] |= (1 << 1);
  cnt--;
  buf[1] = cnt;
  return SZ_DMALP;
}

static inline u32 emit_LPEND(u8 buf[], arg_LPEND_t* arg) {
  buf[0] = CMD_DMALPEND;
  if (arg->loop) buf[0] |= (1 << 2);
  if (!arg->forever) buf[0] |= (1 << 4);
  if (arg->cond == SINGLE)
    buf[0] |= (0 << 1) | (1 << 0);
  else if (arg->cond == BURST)
    buf[0] |= (1 << 1) | (1 << 0);
  buf[1] = arg->bjump;
  return SZ_DMALPEND;
}

static inline u32 emit_KILL(u8 buf[]) {
  buf[0] = CMD_DMAKILL;
  return SZ_DMAKILL;
}

static inline u32 emit_MOV(u8 buf[], dmamov_dst_t dst, u32 val) {
  buf[0] = CMD_DMAMOV;
  buf[1] = dst;
  buf[2] = val & 0xFF;
  buf[3] = (val >> 8) & 0xFF;
  buf[4] = (val >> 16) & 0xFF;
  buf[5] = (val >> 24) & 0xFF;
  return SZ_DMAMOV;
}

static inline u32 emit_NOP(u8 buf[]) {
  buf[0] = CMD_DMANOP;
  return SZ_DMANOP;
}

static inline u32 emit_RMB(u8 buf[]) {
  buf[0] = CMD_DMARMB;
  return SZ_DMARMB;
}

static inline u32 emit_SEV(u8 buf[], u8 ev) {
  buf[0] = CMD_DMASEV;
  ev &= 0x1f;
  ev <<= 3;
  buf[1] = ev;
  return SZ_DMASEV;
}

static inline u32 emit_ST(u8 buf[], pl330_cond_t cond) {
  buf[0] = CMD_DMAST;
  if (cond == SINGLE)
    buf[0] |= (0 << 1) | (1 << 0);
  else if (cond == BURST)
    buf[0] |= (1 << 1) | (1 << 0);
  return SZ_DMAST;
}

static inline u32 emit_STP(u8 buf[], pl330_cond_t cond, u8 peri) {
  buf[0] = CMD_DMASTP;
  if (cond == BURST) buf[0] |= (1 << 1);
  peri &= 0x1f;
  peri <<= 3;
  buf[1] = peri;
  return SZ_DMASTP;
}

static inline u32 emit_STZ(u8 buf[]) {
  buf[0] = CMD_DMASTZ;
  return SZ_DMASTZ;
}

static inline u32 emit_WFP(u8 buf[], pl330_cond_t cond, u8 peri) {
  buf[0] = CMD_DMAWFP;
  if (cond == SINGLE)
    buf[0] |= (0 << 1) | (0 << 0);
  else if (cond == BURST)
    buf[0] |= (1 << 1) | (0 << 0);
  else
    buf[0] |= (0 << 1) | (1 << 0);
  peri &= 0x1f;
  peri <<= 3;
  buf[1] = peri;
  return SZ_DMAWFP;
}

static inline u32 emit_WMB(u8 buf[]) {
  buf[0] = CMD_DMAWMB;
  return SZ_DMAWMB;
}

static inline u32 emit_GO(u8 buf[], arg_GO_t* arg) {
  buf[0] = CMD_DMAGO;
  buf[0] |= (arg->ns << 1);
  buf[1] = arg->chan & 0x7;
  buf[2] = arg->addr & 0xFF;
  buf[3] = (arg->addr >> 8) & 0xFF;
  buf[4] = (arg->addr >> 16) & 0xFF;
  buf[5] = (arg->addr >> 24) & 0xFF;
  return SZ_DMAGO;
}

static inline u32 prepare_ccr(reqcfg_t* rqc) {
  u32 ccr = 0;

  if (rqc->src_inc) ccr |= CC_SRCINC;

  if (rqc->dst_inc) ccr |= CC_DSTINC;

  if (rqc->privileged) ccr |= CC_SRCPRI | CC_DSTPRI;

  if (rqc->nonsecure) ccr |= CC_SRCNS | CC_DSTNS;

  if (rqc->insaccess) ccr |= CC_SRCIA | CC_DSTIA;

  ccr |= (((rqc->brst_len - 1) & 0xf) << CC_SRCBRSTLEN_SHIFT);
  ccr |= (((rqc->brst_len - 1) & 0xf) << CC_DSTBRSTLEN_SHIFT);

  ccr |= (rqc->brst_size << CC_SRCBRSTSIZE_SHIFT);
  ccr |= (rqc->brst_size << CC_DSTBRSTSIZE_SHIFT);

  ccr |= (rqc->scctl << CC_SRCCCTRL_SHIFT);
  ccr |= (rqc->dcctl << CC_DSTCCTRL_SHIFT);

  ccr |= (rqc->swap << CC_SWAP_SHIFT);

  return ccr;
}

static u8 pl330_until_dmac_idle(u32 loops) {
  do {
    if (!(*(u32*)(PL330_BASE + DBGSTATUS) & DBG_BUSY)) break;
  } while (--loops);
  if (!loops) return 1;
  return 0;
}

static void pl330_execute_DBGINSN(u8 ins[], u8 as_manager, u8 channel) {
  u32 val = (ins[0] << 16) | (ins[1] << 24);

  if (!as_manager) val |= (1 << 0);

  val |= (channel << 8);
  *(u32*)(PL330_BASE + DBGINST0) = val;

  val = ins[2];
  val = val | (ins[3] << 8);
  val = val | (ins[4] << 16);
  val = val | (ins[5] << 24);
  *(u32*)(PL330_BASE + DBGINST1) = val;

  if (pl330_until_dmac_idle(PL330_TIMEOUT_LOOPS)) {
    return;
  }

  *(u32*)(PL330_BASE + DBGCMD) = 0;
}

static u32 pl330_get_state(u8 channel) {
  u32 val = (*(u32*)(PL330_BASE + CS(channel)) & 0xf);
  switch (val) {
    case DS_ST_STOP:
      return PL330_STATE_STOPPED;
    case DS_ST_EXEC:
      return PL330_STATE_EXECUTING;
    case DS_ST_CMISS:
      return PL330_STATE_CACHEMISS;
    case DS_ST_UPDTPC:
      return PL330_STATE_UPDTPC;
    case DS_ST_WFE:
      return PL330_STATE_WFE;
    case DS_ST_FAULT:
      return PL330_STATE_FAULTING;
    case DS_ST_ATBRR:
      return PL330_STATE_ATBARRIER;
    case DS_ST_QBUSY:
      return PL330_STATE_QUEUEBUSY;
    case DS_ST_WFP:
      return PL330_STATE_WFP;
    case DS_ST_KILL:
      return PL330_STATE_KILLING;
    case DS_ST_CMPLT:
      return PL330_STATE_COMPLETING;
    case DS_ST_FLTCMP:
      return PL330_STATE_FAULT_COMPLETING;
    default:
      return PL330_STATE_INVALID;
  }
}

s32 pl330_trigger(pl330_transfer_t* xfer) {
  if (pl330_get_state(xfer->channel) != PL330_STATE_STOPPED) return 1;

  u8 ins[6] = {0, 0, 0, 0, 0, 0};
  arg_GO_t go;
  go.chan = xfer->channel;
  go.addr = (u32)xfer->prog;
  go.ns = 0;
  emit_GO(ins, &go);

  pl330_execute_DBGINSN(ins, 1, xfer->channel);
  return 0;
}

static void pl330_stop(u8 channel) {
  if (pl330_get_state(channel) == PL330_STATE_FAULT_COMPLETING)
    UNTIL(channel, PL330_STATE_FAULTING | PL330_STATE_KILLING);

  if (pl330_get_state(channel) == PL330_STATE_COMPLETING ||
      pl330_get_state(channel) == PL330_STATE_KILLING ||
      pl330_get_state(channel) == PL330_STATE_STOPPED)
    return;

  u8 ins[6] = {0, 0, 0, 0, 0, 0};
  emit_KILL(ins);

  pl330_execute_DBGINSN(ins, 0, channel);
}

s32 pl330_init(pl330_transfer_t* xfer) {
  if (xfer->src_addr & 0x3) {
    log_error("PL330 DMAC: Source address not aligned.");
    return 1;
  }

  if (xfer->dst_addr & 0x3) {
    log_error("PL330 DMAC: Destination address not aligned.");
    return 1;
  }

  if (xfer->bytes % PL330_BURST_SIZE) {
    log_error("PL330 DMAC: Number of bytes has to be a multiple of %u.",
              PL330_BURST_SIZE);
    return 1;
  }

  reqcfg_t reqcfg;
  reqcfg.dcctl = DCCTRL0;
  reqcfg.scctl = SCCTRL0;
  reqcfg.swap = SWAP_NO;
  reqcfg.dst_inc = 1;
  reqcfg.src_inc = 1;
  reqcfg.nonsecure = 0;
  reqcfg.privileged = 1;
  reqcfg.insaccess = 0;
  reqcfg.brst_len = PL330_BURST_LENGTH;
  reqcfg.brst_size = (PL330_BURST_SIZE >> 1);

  u32 offset = 0;

  /* Preparing the CCR value */
  u32 ccr = prepare_ccr(&reqcfg);
  /* DMAMOV CCR, ccr */
  offset += emit_MOV(&xfer->prog[offset], CCR, ccr);

  /* DMAMOV SAR, x->src_addr */
  offset += emit_MOV(&xfer->prog[offset], SAR, xfer->src_addr);
  /* DMAMOV DAR, x->dst_addr */
  offset += emit_MOV(&xfer->prog[offset], DAR, xfer->dst_addr);

  u32 length = xfer->bytes / PL330_BURST_SIZE;
  u32 bursts = length / PL330_BURST_LENGTH;
  u32 single = length % PL330_BURST_LENGTH;

  /* Bursts */
  while (bursts > 0) {
    u16 lcnt0 = 256;
    if (bursts < 256) {
      lcnt0 = bursts;
    }

    /* DMALP0 */
    offset += emit_LP(&xfer->prog[offset], 0, lcnt0);
    u32 loopjmp0 = offset;

    /* DMALD */
    offset += emit_LD(&xfer->prog[offset], ALWAYS);

    /* DMAST */
    offset += emit_ST(&xfer->prog[offset], ALWAYS);

    /* DMALPEND0 */
    arg_LPEND_t lpend0;
    lpend0.cond = ALWAYS;
    lpend0.forever = 0;
    lpend0.loop = 0;
    lpend0.bjump = offset - loopjmp0;
    offset += emit_LPEND(&xfer->prog[offset], &lpend0);

    bursts -= lcnt0;
  }

  /* Single */
  if (single > 0) {
    reqcfg.brst_len = 1;
    /* Preparing the CCR value */
    ccr = prepare_ccr(&reqcfg);
    /* DMAMOV CCR, ccr */
    offset += emit_MOV(&xfer->prog[offset], CCR, ccr);

    u16 lcnt0 = single;

    /* DMALP0 */
    offset += emit_LP(&xfer->prog[offset], 0, lcnt0);
    u32 loopjmp0 = offset;

    /* DMALD */
    offset += emit_LD(&xfer->prog[offset], ALWAYS);

    /* DMAST */
    offset += emit_ST(&xfer->prog[offset], ALWAYS);

    /* DMALPEND0 */
    arg_LPEND_t lpend0;
    lpend0.cond = ALWAYS;
    lpend0.forever = 0;
    lpend0.loop = 0;
    lpend0.bjump = offset - loopjmp0;
    offset += emit_LPEND(&xfer->prog[offset], &lpend0);
  }

  /* DMASEV - notify processor */
  offset += emit_SEV(&xfer->prog[offset], xfer->channel);

  /* DMAEND */
  offset += emit_END(&xfer->prog[offset]);

  if (offset > sizeof(xfer->prog)) {
    log_error("PL330 DMAC: Program buffer too small.");
    return 1;
  }
  return 0;
}

s32 pl330_start(pl330_transfer_t* xfer) {
  switch (pl330_get_state(xfer->channel)) {
    case PL330_STATE_FAULT_COMPLETING:
      UNTIL(xfer->channel, PL330_STATE_FAULTING | PL330_STATE_KILLING);
      if (pl330_get_state(xfer->channel) == PL330_STATE_KILLING)
        UNTIL(xfer->channel, PL330_STATE_STOPPED);

    case PL330_STATE_FAULTING:
      pl330_stop(xfer->channel);

    case PL330_STATE_KILLING:
    case PL330_STATE_COMPLETING:
      UNTIL(xfer->channel, PL330_STATE_STOPPED);

    case PL330_STATE_STOPPED:
      return pl330_trigger(xfer);

    case PL330_STATE_WFP:
    case PL330_STATE_QUEUEBUSY:
    case PL330_STATE_ATBARRIER:
    case PL330_STATE_UPDTPC:
    case PL330_STATE_CACHEMISS:
    case PL330_STATE_EXECUTING:
    case PL330_STATE_WFE:
    default:
      return 1;
  }
}

void pl330_irq_enable(u8 channel) {
  *(u32*)(PL330_BASE + INTEN) |= (1 << channel);
}

void pl330_irq_disable(u8 channel) {
  *(u32*)(PL330_BASE + INTEN) &= ~(1 << channel);
}

void pl330_irq_clear(u8 channel) {
  *(u32*)(PL330_BASE + INTCLR) |= (1 << channel);
}
