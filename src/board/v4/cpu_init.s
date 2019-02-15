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

    .text
    .fpu    vfp3
    .code   32
    .balign 4

    .global __early_init
__early_init:

#if defined(THUMB_NO_INTERWORKING)
    .code   16
    mov r0, pc
    bx  r0
    .code   32
#endif

    /* Invalidate caches */
    mov r0, #0                      /* r0 = 0 */
    mcr p15, 0, r0, c7, c5, 0       /* invalidate icache */
    mcr p15, 0, r0, c7, c5, 6       /* invalidate branch predictor array */
    mcr p15, 0, r0, c15, c5, 0      /* invalidate dcache */

    /* Disable MPU */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    bic r0, r0, #0x1                /* clear M bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */

    /* Done */
#if defined(THUMB_NO_INTERWORKING)
    add r0, pc, #1
    bx r0
    .code   16
    bx lr
    .code   32
#else
    bx lr
#endif

    .global __late_init
__late_init:

#if defined(THUMB_NO_INTERWORKING)
    .code   16
    mov r0, pc
    bx  r0
    .code   32
#endif
    /*
     * Cached text initialization.
     * NOTE: It assumes that the size is a multiple of 4.
     */
    ldr     r1, =__cached_text_flash
    ldr     r2, =__cached_text_start
    ldr     r3, =__cached_text_end
cached_text_loop:
    cmp     r2, r3
    ldrlo   r0, [r1], #4
    strlo   r0, [r2], #4
    blo     cached_text_loop

#if 0
    /* Set MMU TTB0 base */
    ldr r0, =MMUTable               /* load MMU translation table base */
    orr r0, r0, #0x2                /* shareable, non-cacheable */
    mcr p15, 0, r0, c2, c0, 0       /* write TTB0 */

    /* Configure MMU domains */
    mov r0, #0xffffffff             /* all ones = manager, permissions ignored */
    mcr p15, 0, r0, c3, c0, 0       /* write DACR */

    /* Enable mpu, icache and dcache */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    orr r0, r0, #(0x1 << 12)        /* set I bit */
    orr r0, r0, #(0x1 << 2)         /* set C bit */
    //orr r0, r0, #(0x1 << 0)         /* set M bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */
    dsb                             /* allow the MPU to start up */
    isb                             /* flush prefetch buffer */
#endif

    /* Allow full access to coprocessors */
    mrc p15, 0, r0, c1, c0, 2       /* read CPACR  */
    orr r0, r0, #(0xf << 20)        /* enable full access for p10 & p11 */
    mcr p15, 0, r0, c1, c0, 2       /* write CPACR */

    /* Enable vfp */
    fmrx r0, FPEXC                  /* read FPEXC */
    orr r0, r0, #(0x1 << 30)        /* set VFP enable bit */
    fmxr FPEXC, r0                  /* write FPEXC */

    /* Enable flow prediction */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    orr r0, r0, #(0x1 << 11)        /* set Z bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */

    /* Enable unaligned data access */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    bic r0, r0, #(0x1 << 1)         /* clear A bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */

    /* Unmask asynchronous abort exception */
    mrs r0, cpsr                    /* read CPSR */
    bic r0, r0, #(0x1 << 8)         /* clear A bit */
    msr cpsr_xsf, r0                /* write CPSR<31:8> */

    /* Done */
#if defined(THUMB_NO_INTERWORKING)
    add r0, pc, #1
    bx r0
    .code   16
    bx lr
    .code   32
#else
    bx lr
#endif

.end
