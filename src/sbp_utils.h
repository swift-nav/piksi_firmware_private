/*
 * Copyright (C) 2014, 2016 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SBP_UTILS_H
#define SWIFTNAV_SBP_UTILS_H

#include <libsbp/common.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/gnss.h>
#include <libsbp/ndb.h>
#include <libsbp/system.h>
#include <libswiftnav/time.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/ephemeris.h>

typedef struct {
  union {
    msg_ephemeris_gps_t   gps;
    msg_ephemeris_sbas_t  sbas;
    msg_ephemeris_glo_t   glo;
  };
} msg_ephemeris_t;

typedef struct {
  u16 msg_id;
  u16 size;
} msg_info_t;

typedef union {
  msg_almanac_gps_t   gps;
  msg_almanac_glo_t   glo;
} msg_almanac_t;

typedef enum {
  NDB_EVENT_UNKNOWN = 0,
  NDB_EVENT_STORE = 1,
  NDB_EVENT_FETCH = 2,
  NDB_EVENT_ERASE = 3,
} ndb_event_t;

typedef enum {
  NDB_EVENT_OTYPE_UNKNOWN = 0,
  NDB_EVENT_OTYPE_EPHEMERIS = 1,
  NDB_EVENT_OTYPE_ALMANAC = 2,
  NDB_EVENT_OTYPE_ALMANAC_WN = 3,
  NDB_EVENT_OTYPE_IONO = 4,
  NDB_EVENT_OTYPE_L2C_CAP = 5,
  NDB_EVENT_OTYPE_LGF = 6,
  NDB_EVENT_OTYPE_UTC_PARAMS = 7,
} ndb_event_obj_type_t;

#define NDB_EVENT_SENDER_ID_VOID 0

void sbp_make_gps_time(msg_gps_time_t *t_out, const gps_time_t *t_in, u8 flags);
void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh, const double llh[3],
                           double h_accuracy, double v_accuracy,
                           const gps_time_t *gps_t, u8 n_sats_used, u8 flags);
void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef, const double ecef[3],
                            double accuracy, const gps_time_t *gps_t, u8 n_sats_used,
                            u8 flags);
void sbp_make_vel_ned(msg_vel_ned_t *vel_ned, const gnss_solution *soln, double h_accuracy, double v_accuracy, u8 flags);
void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef, const gnss_solution *soln, double accuracy, u8 flags);
void sbp_make_dops(msg_dops_t *dops_out, const dops_t *dops_in, u32 tow, u8 flags);
void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, const gps_time_t *t,
                            u8 n_sats, const double b_ecef[3], double accuracy,
                            u8 flags);
void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned, const gps_time_t *t,
                           u8 n_sats, const double b_ned[3], double h_accuracy,
                           double v_accuracy, u8 flags);
void sbp_make_heading(msg_baseline_heading_t *baseline_heading, const gps_time_t *t,
                      const double heading, u8 n_sats_used, u8 flags);
void sbp_make_age_corrections(msg_age_corrections_t *age_corrections, const gps_time_t *t, double propagation_time);
void sbp_make_dgnss_status(msg_dgnss_status_t *dgnss_status, u8 num_sats, double obs_latency, u8 flags);
void sbp_make_utc_time(msg_utc_time_t *t_out, const gps_time_t *t_in, u8 flags,
                       const utc_params_t *utc_params);
void sbp_send_ndb_event(u8 event,
                        u8 obj_type,
                        u8 result,
                        u8 data_source,
                        const gnss_signal_t *object_sid,
                        const gnss_signal_t *src_sid,
                        u16 sender);

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK

#define MSG_OBS_P_MULTIPLIER             ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER           ((float)4)
#define MSG_OBS_LF_MULTIPLIER            ((double) (1 << 8))
#define MSG_OBS_DF_MULTIPLIER            ((double) (1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID         ((u8) (1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID        ((u8) (1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN   ((u8) (1 << 2))
#define MSG_OBS_FLAGS_MEAS_DOPPLER_VALID ((u8) (1 << 3))

#define MSG_HEADING_SCALE_FACTOR 1000.0

void unpack_obs_header(const observation_header_t *msg, gps_time_t* t,
                       u8* total, u8* count);

void pack_obs_header(const gps_time_t *t, u8 total, u8 count,
                     observation_header_t *msg);

u8 nm_flags_to_sbp(nav_meas_flags_t from);
nav_meas_flags_t nm_flags_from_sbp(u8 from);

void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L,
                        double *D, double *cn0, double *lock_time,
                        nav_meas_flags_t *flags, gnss_signal_t *sid);

s8 pack_obs_content(double P, double L, double D, double cn0, double lock_time,
                    nav_meas_flags_t flags, gnss_signal_t sid,
                    packed_obs_content_t *msg);

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e);

msg_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg);

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8*, void*));

msg_info_t pack_almanac(const almanac_t *a, msg_almanac_t *msg);

/** Value specifying the size of the SBP framing */
#define SBP_FRAMING_SIZE_BYTES 8
/** Value defining maximum SBP packet size */
#define SBP_FRAMING_MAX_PAYLOAD_SIZE 255

gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from);
sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from);

gnss_signal_t sid_from_sbp16(const gnss_signal16_t from);
gnss_signal16_t sid_to_sbp16(const gnss_signal_t from);

void sbp_send_iono(const ionosphere_t *iono);
void sbp_send_l2c_capabilities(const u32 *l2c_cap);
void sbp_send_group_delay(const cnav_msg_t *cnav);

#endif /* SWIFTNAV_SBP_UTILS_H */
