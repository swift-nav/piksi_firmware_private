/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

    .section .mmu_tbl,"a"
    /* Note: section must be aligned to a 16kB boundary */

    .global  MMUTable
MMUTable:
    /* Each table entry occupies one 32-bit word and there are
     * 4096 entries, so the entire table takes up 16kB.
     * Each entry covers a 1MB section.
     */
.set SECT, 0

                            /* 0x00000000 - 0x00100000 (Low OCM / DDR)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000

.rept   0x1BF               /* 0x00100000 - 0x1c000000 (Linux DDR)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x10                /* 0x1c000000 - 0x1d000000 (Shared DDR)
                             * Normal, shared, non-cacheable */
.word   SECT + 0x14de2      /* S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x30                /* 0x1d000000 - 0x1fffffff (Firmware DDR)
                             * Normal, shared, inner and outer cacheable */
.word   SECT + 0x15de6      /* S=b1 TEX=b101 AP=b11, Domain=b1111, C=b0, B=b1 */
.set    SECT, SECT+0x100000
.endr

.rept   0x200               /* 0x20000000 - 0x3fffffff (DDR reserved)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0400              /* 0x40000000 - 0x7fffffff (FPGA slave0)
                             * Strongly ordered */
.word   SECT + 0xc02        /* S=b0 TEX=b000 AP=b11, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0400              /* 0x80000000 - 0xbfffffff (FPGA slave1)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0200              /* 0xc0000000 - 0xdfffffff (unassigned/reserved).
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x003               /* 0xe0000000 - 0xe02fffff (Memory mapped devices)
                             * UART/USB/IIC/SPI/CAN/GEM/GPIO/QSPI/SD/NAND
                             * Device */
.word   SECT + 0xc06        /* S=b0 TEX=b000 AP=b11, Domain=b0, C=b0, B=b1 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0D                /* 0xe0300000 - 0xe0ffffff (unassigned/reserved).
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0010              /* 0xe1000000 - 0xe1ffffff (NAND)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0020              /* 0xe2000000 - 0xe3ffffff (NOR)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0020              /* 0xe4000000 - 0xe5ffffff (SRAM)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0120              /* 0xe6000000 - 0xf7ffffff (unassigned/reserved).
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

/* 0xf8000c00 to 0xf8000fff, 0xf8010000 to 0xf88fffff and
   0xf8f03000 to 0xf8ffffff are reserved  but due to granual size of
   1MB, it is not possible to define separate regions for them */

.rept   0x0010              /* 0xf8000000 - 0xf8ffffff (AMBA APB Peripherals)
                             * Device */
.word   SECT + 0xc06        /* S=b0 TEX=b000 AP=b11, Domain=b0, C=b0, B=b1 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0030              /* 0xf9000000 - 0xfbffffff (unassigned/reserved).
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x0020              /* 0xfc000000 - 0xfdffffff (Linear QSPI - XIP)
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

.rept   0x001F              /* 0xfe000000 - 0xffefffff (unassigned/reserved).
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000
.endr

/* 0xfff00000 to 0xfffb0000 is reserved but due to granual size of
   1MB, it is not possible to define separate region for it */

                            /* 0xfff00000 - 0xffffffff
                             * 256K OCM when mapped to high address space
                             * Generates a translation fault if accessed */
.word   SECT + 0x0          /* S=b0 TEX=b000 AP=b00, Domain=b0, C=b0, B=b0 */
.set    SECT, SECT+0x100000

.end
