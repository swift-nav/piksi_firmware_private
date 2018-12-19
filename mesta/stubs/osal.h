/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OSAL_H_INCLUDED
#define OSAL_H_INCLUDED

#include "ch.h"

#define TRUE true
#define FALSE false

#define IRQ_ID_FPGA2 1
#define IRQ_SENSITIVITY_EDGE 2

typedef int irq_id_t;
typedef void* irq_handler_t;
typedef int irq_sensitivity_t;
typedef uint8_t irq_priority_t;

void gic_handler_register(irq_id_t irq_id,
                          irq_handler_t handler,
                          void* context);

void gic_irq_sensitivity_set(irq_id_t irq_id, irq_sensitivity_t sensitivity);
void gic_irq_priority_set(irq_id_t irq_id, irq_priority_t priority);
void gic_irq_enable(irq_id_t irq_id);

#endif
