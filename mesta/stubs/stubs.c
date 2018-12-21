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

#include "libsbp/logging.h"
#include "libsbp/sbp.h"
#include "manage.h"
#include "nap/axi_dma.h"
#include "platform/starling_platform.h"
#include "platform/starling_platform_semaphore.h"
#include "settings/settings_client.h"
#include "simulator/simulator.h"
#include "system_monitor/system_monitor.h"

AXIDMADriver AXIDMADriver1;
bool disable_raim = false;

errno_t platform_mailbox_post(mailbox_id_t id, void *msg, int blocking) {
  (void)id;
  (void)msg;
  (void)blocking;
  return 0;
}

void platform_mailbox_item_free(mailbox_id_t id, const void *ptr) {
  (void)id;
  (void)ptr;
}

errno_t platform_mailbox_fetch(mailbox_id_t id, void **msg, int blocking) {
  (void)id;
  (void)msg;
  (void)blocking;
  return 0;
}

void platform_sem_signal(platform_sem_t *sem) { (void)sem; }

platform_sem_t *platform_sem_create(void) { return NULL; }

void platform_mailbox_init(mailbox_id_t id) { (void)id; }

void *platform_mailbox_item_alloc(mailbox_id_t id) {
  (void)id;
  return NULL;
}

int platform_sem_wait_timeout(platform_sem_t *sem, unsigned long millis) {
  (void)sem;
  (void)millis;
  return 0;
}

void watchdog_notify(watchdog_notify_t thread_id) { (void)thread_id; }

void platform_mutex_lock(mtx_id_t id) { (void)id; }

void platform_mutex_unlock(mtx_id_t id) { (void)id; }

void gic_handler_register(irq_id_t irq_id,
                          irq_handler_t handler,
                          void *context) {
  (void)irq_id;
  (void)handler;
  (void)context;
}

void chMtxObjectInit(mutex_t *mp) { (void)mp; }

void chMtxLock(mutex_t *mp) { (void)mp; }

void chMtxUnlock(mutex_t *mp) { (void)mp; }

systime_t chThdSleepS(systime_t time) {
  (void)time;
  return 0;
}

thread_t *chThdCreateStatic(
    void *wsp, size_t size, tprio_t prio, tfunc_t pf, void *arg) {
  (void)wsp;
  (void)size;
  (void)prio;
  (void)pf;
  (void)arg;

  return NULL;
}

systime_t chVTGetSystemTimeX(void) { return 0; }

void chRegSetThreadNameX(thread_t *tp, const char *name) {
  (void)tp;
  (void)name;
}

void chRegSetThreadName(const char *name) { (void)name; }

systime_t chThdSleepMilliseconds(systime_t time) {
  (void)time;
  return 0;
}

void chSysLock(void) {}

void chSysUnlock(void) {}

void chSysLockFromISR(void) {}

void chSysUnlockFromISR(void) {}

extern swiftnap_t mesta_nap;

systime_t chThdSleep(systime_t time) {
  u32 *count = (u32 *)(uintptr_t)&mesta_nap.TIMING_COUNT;
  *count += 1000;
  (void)time;
  return 0;
}

void chBSemResetI(binary_semaphore_t *bsp, bool taken) {
  (void)bsp;
  (void)taken;
}
void chBSemReset(binary_semaphore_t *bsp, bool taken) {
  (void)bsp;
  (void)taken;
}

void chBSemSignalI(binary_semaphore_t *bsp) { (void)bsp; }

void chBSemSignal(binary_semaphore_t *bsp) { (void)bsp; }

void chBSemObjectInit(binary_semaphore_t *bsp, bool taken) {
  (void)bsp;
  (void)taken;
}

msg_t chBSemWaitTimeoutS(binary_semaphore_t *bsp, systime_t time) {
  (void)bsp;
  (void)time;
  return 0;
}

msg_t chBSemWaitTimeout(binary_semaphore_t *bsp, systime_t time) {
  (void)bsp;
  (void)time;
  return 0;
}

bool chMtxTryLockS(mutex_t *mp) {
  (void)mp;
  return true;
}

bool chMtxTryLock(mutex_t *mp) {
  (void)mp;
  return true;
}

void gic_irq_sensitivity_set(irq_id_t irq_id, irq_sensitivity_t sensitivity) {
  (void)irq_id;
  (void)sensitivity;
}

void gic_irq_priority_set(irq_id_t irq_id, irq_priority_t priority) {
  (void)irq_id;
  (void)priority;
}

void gic_irq_enable(irq_id_t irq_id) { (void)irq_id; }

bool simulation_enabled_for(simulation_modes_t mode_mask) {
  (void)mode_mask;
  return false;
}

void nap_rd_dna(u8 dna[]) { (void)dna; }

void axi_dma_init(void) {}

void axi_dma_start(AXIDMADriver *dp) { (void)dp; }

double nominal_doppler(const double sat_vel[3],
                       const double sat_pos[3],
                       const double receiver_pos[3],
                       const double sat_clock_error_rate) {
  (void)sat_vel;
  (void)sat_pos;
  (void)receiver_pos;
  (void)sat_clock_error_rate;
  return 0;
}

double nominal_pseudorange(const double sat_pos[3],
                           const double receiver_pos[3],
                           const double sat_clock_error) {
  (void)sat_pos;
  (void)receiver_pos;
  (void)sat_clock_error;
  return 0;
}

void starling_set_elevation_mask(float elevation_mask) { (void)elevation_mask; }

void starling_set_solution_frequency(double frequency) { (void)frequency; }

bool simulation_enabled(void) { return true; }

void pvt_engine_covariance_to_accuracy(const double covariance_ecef_in[9],
                                       const double ref_ecef_in[3],
                                       double *accuracy,
                                       double *h_accuracy,
                                       double *v_accuracy,
                                       double ecef_cov[6],
                                       double ned_cov[6]) {
  (void)covariance_ecef_in;
  (void)ref_ecef_in;
  (void)accuracy;
  (void)h_accuracy;
  (void)v_accuracy;
  (void)ecef_cov;
  (void)ned_cov;
}

measurement_state_t simulation_measurement_state(u8 channel) {
  (void)channel;
  measurement_state_t dummy = {0};
  return dummy;
}

u8 simulation_current_num_sats(void) { return 0; }

u8 *grab_samples(u32 *length, u64 *p_count) {
  (void)length;
  (void)p_count;
  return NULL;
}

u32 io_support_n_read(void *sd) {
  (void)sd;
  return 0;
}

u32 io_support_read_timeout(void *sd, u8 data[], u32 len, u32 timeout) {
  (void)sd;
  (void)data;
  (void)len;
  (void)timeout;
  return 0;
}

u32 io_support_write(void *sd, const u8 data[], u32 len) {
  (void)sd;
  (void)data;
  (void)len;
  return 0;
}

int settings_api_register(struct setting *setting, settings_type_t type) {
  (void)setting;
  (void)type;
  return 0;
}

int settings_api_register_enum(const char *const enum_names[],
                               settings_type_t *type) {
  (void)enum_names;
  (void)type;
  return 0;
}

void sbp_state_init(sbp_state_t *s) { (void)s; }

s8 sbp_register_callback(sbp_state_t *s,
                         u16 msg_type,
                         sbp_msg_callback_t cb,
                         void *context,
                         sbp_msg_callbacks_node_t *node) {
  (void)s;
  (void)msg_type;
  (void)cb;
  (void)context;
  (void)node;
  return 0;
}

s8 sbp_remove_callback(sbp_state_t *s, sbp_msg_callbacks_node_t *node) {
  (void)s;
  (void)node;
  return 0;
}

s8 sbp_send_message(sbp_state_t *s,
                    u16 msg_type,
                    u16 sender_id,
                    u8 len,
                    u8 *payload,
                    s32 (*write)(u8 *buff, u32 n, void *context)) {
  (void)s;
  (void)msg_type;
  (void)sender_id;
  (void)len;
  (void)payload;
  (void)write;
  if (SBP_MSG_LOG == msg_type) {
    printf("%s\n", payload);
  }
  return 0;
}

s8 sbp_process(sbp_state_t *s, s32 (*read)(u8 *buff, u32 n, void *context)) {
  (void)s;
  (void)read;
  return 0;
}

void sbp_fileio_remove(const char *fn) { (void)fn; }

ssize_t sbp_fileio_write(const char *filename,
                         off_t offset,
                         const u8 *buf,
                         size_t size) {
  (void)filename;
  (void)offset;
  (void)buf;
  (void)size;
  return 0;
}

ssize_t sbp_fileio_read(const char *filename,
                        off_t offset,
                        u8 *buf,
                        size_t size) {
  (void)filename;
  (void)offset;
  (void)buf;
  (void)size;
  return 0;
}

u16 sender_id_get(void) { return 0x1; /* dummy value */ }

void tp_drop_channel(tracker_t *tracker, ch_drop_reason_t reason) {
  (void)tracker;
  (void)reason;
}
