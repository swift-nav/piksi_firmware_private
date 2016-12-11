/*
 * Copyright (C) 2014, 2016 Swift Navigation Inc.
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
#include <libswiftnav/time.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/signal.h>


/** Header for sdiff message.
 *
* Header of a GNSS sdiff message.
 */
typedef struct __attribute__((packed)) {
  gps_time_nano_t t;                   /**< GNSS time of this observation */
  u8 n_obs;               /**< Total number of observations. First nibble is the size
of the sequence (n), second nibble is the zero-indexed
counter (ith packet of n)
 */
  double rover_pos_x;
  double rover_pos_y;
  double rover_pos_z;
  double base_pos_x;
  double base_pos_y;
  double base_pos_z;
  double propagation_time;
} sdiff_header_t;


/** GNSS sdiffs for a particular satellite signal.
 *
 * Sdiffs before updating the filter itself.
 */
typedef struct __attribute__((packed)) {
  u32 P;            /**< Pseudorange observation [2 cm] */
  carrier_phase_t L;            /**< Carrier phase observation with typical sign convention. [cycles] */
  doppler_t MD;           /**< Doppler observation with typical sign convention. [Hz] */
  doppler_t CD;           /**< Doppler observation with typical sign convention. [Hz] */
  u8 cn0;          /**< Carrier-to-Noise density.  Zero implies invalid cn0. [dB Hz / 4] */
  u8 lock;         /**< Lock timer. This value gives an indication of the time
for which a signal has maintained continuous phase lock.
Whenever a signal has lost and regained lock, this
value is reset to zero. It is encoded according to DF402 from
the RTCM 10403.2 Amendment 2 specification.  Valid values range
from 0 to 15 and the most significant nibble is reserved for future use.
 */
  u8 flags;        /**< Measurement status flags. A bit field of flags providing the
status of this observation.  If this field is 0 it means only the Cn0
estimate for the signal is valid.
 */
  gnss_signal16_t sid;          /**< GNSS signal identifier (16 bit) */
  double sat_pos_x;
  double sat_pos_y;
  double sat_pos_z;
  double sat_vel_x;
  double sat_vel_y;
  double sat_vel_z;
} packed_sdiff_content_t;


/** GPS satellite sdiffs
 *
 * The GPS sdiffs message reports all the raw sdiff pseudorange and
 * sdiff carrier phase observations for the satellites being tracked by
 * the device.
 */
#define SBP_MSG_SDIFFS               0x0110
typedef struct __attribute__((packed)) {
  observation_header_t header;    /**< Header of a GPS observation message */
  packed_obs_content_t obs[0];    /**< Pseudorange and carrier phase observation for a
satellite being tracked.
 */
} msg_sdiffs_t;

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
} msg_ephemeris_info_t;

void sbp_make_gps_time(msg_gps_time_t *t_out, const gps_time_t *t_in, u8 flags);
void sbp_make_pos_llh(msg_pos_llh_t *pos_llh, const gnss_solution *soln, u8 flags);
void sbp_make_pos_llh_vect(msg_pos_llh_t *pos_llh, const double llh[3],
                           double h_accuracy, double v_accuracy,
                           const gps_time_t *gps_t, u8 n_sats_used, u8 flags);
void sbp_make_pos_ecef(msg_pos_ecef_t *pos_ecef, const gnss_solution *soln, u8 flags);
void sbp_make_pos_ecef_vect(msg_pos_ecef_t *pos_ecef, const double ecef[3],
                            double accuracy, const gps_time_t *gps_t, u8 n_sats_used,
                            u8 flags);
void sbp_make_vel_ned(msg_vel_ned_t *vel_ned, const gnss_solution *soln, u8 flags);
void sbp_make_vel_ecef(msg_vel_ecef_t *vel_ecef, const gnss_solution *soln, u8 flags);
void sbp_make_dops(msg_dops_t *dops_out, const dops_t *dops_in, const gps_time_t *t, u8 flags);
void sbp_make_baseline_ecef(msg_baseline_ecef_t *baseline_ecef, const gps_time_t *t,
                            u8 n_sats, const double b_ecef[3], double accuracy,
                            u8 flags);
void sbp_make_baseline_ned(msg_baseline_ned_t *baseline_ned, const gps_time_t *t,
                           u8 n_sats, const double b_ned[3], double h_accuracy,
                           double v_accuracy, u8 flags);
void sbp_make_heading(msg_baseline_heading_t *baseline_heading, const gps_time_t *t,
                      const double heading, u8 n_sats_used, u8 flags);
#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK
#define MSG_OBS_TOW_MULTIPLIER ((double)1e3)
#define MSG_OBS_TOW_NS_MULTIPLIER ((double)1e9)

#define MSG_OBS_P_MULTIPLIER             ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER           ((float)4)
#define MSG_OBS_LF_MULTIPLIER            ((double) (1 << 8))
#define MSG_OBS_DF_MULTIPLIER            ((double) (1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID         ((u8) (1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID        ((u8) (1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN   ((u8) (1 << 2))
#define MSG_OBS_FLAGS_MEAS_DOPPLER_VALID ((u8) (1 << 3))

void unpack_obs_header(const observation_header_t *msg, gps_time_t* t,
                       u8* total, u8* count);

void pack_obs_header(const gps_time_t *t, u8 total, u8 count,
                     observation_header_t *msg);
void pack_sdiff_header(const gps_time_t *t, u8 total, u8 count, const double rcv_pos[3], const double base_pos[3], const double prop_time,
                       sdiff_header_t *msg);

u8 nm_flags_to_sbp(nav_meas_flags_t from);
nav_meas_flags_t nm_flags_from_sbp(u8 from);

void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L,
                        double *D, double *cn0, double *lock_time,
                        nav_meas_flags_t *flags, gnss_signal_t *sid);

s8 pack_obs_content(double P, double L, double D, double cn0, double lock_time,
                    nav_meas_flags_t flags, gnss_signal_t sid,
                    packed_obs_content_t *msg);
s8 pack_sdiff_content(double P, double L, double MD, double CD, const double sat_pos[3],
                      const double sat_vel[3], double cn0, double lock_time,
                      nav_meas_flags_t flags, gnss_signal_t sid,
                      packed_sdiff_content_t *msg);

void unpack_ephemeris(const msg_ephemeris_t *msg, ephemeris_t *e);

msg_ephemeris_info_t pack_ephemeris(const ephemeris_t *e, msg_ephemeris_t *msg);

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8*, void*));

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
u32 round_tow_ms(double tow);

#endif /* SWIFTNAV_SBP_UTILS_H */
