/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * Zynq7000 memory setup.
 */
MEMORY
{
    flash : org = 0x1e000000, len = 0x00800000
    vring : org = 0x1e800000, len = 0x00800000
    ram0  : org = 0x1f000000, len = 0x01000000
    ram1  : org = 0x00000000, len = 0
    ram2  : org = 0x00000000, len = 0
    ram3  : org = 0x00000000, len = 0
    ram4  : org = 0x00000000, len = 0
    ram5  : org = 0x00000000, len = 0
    ram6  : org = 0x00000000, len = 0
    ram7  : org = 0x00000000, len = 0
}

/* RAM region to be used for stacks. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("STACKS_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

SECTIONS {
    /* Duplicated here so that vectors remain at the start of flash */
    startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.vectors))
    } > flash

    .mmu_tbl : ALIGN(16k)
    {
        PROVIDE(__mmu_tbl_start = .);
        KEEP(*(.mmu_tbl))
        PROVIDE(__mmu_tbl_end = .);
    } > flash

    .resource_table :
    {
        PROVIDE(__rtable_start = .);
        KEEP(*(.resource_table))
        PROVIDE(__rtable_end = .);
    } > flash

    .cached_text_align : {. = ALIGN(16);} > flash
    PROVIDE(__cached_text_flash = .);

    .cached_text : ALIGN(16) SUBALIGN(16)
    {
        . = ALIGN(4);
        PROVIDE(__cached_text_start = .);
        *(EXCLUDE_FILE(*crt0.o *crt1.o *cpu_init.o) .text)
        *(EXCLUDE_FILE(*crt0.o *crt1.o *cpu_init.o) .text.*)
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        PROVIDE(__cached_text_end = .);
    } > ram0 AT > flash
}

__main_thread_stack_base__ = __stacks_base__;
_ecoffee_fs_area = _coffee_fs_area + 524288;

INCLUDE rules.ld
