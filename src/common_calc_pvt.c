#include <assert.h>

#include <libswiftnav/pvt.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>

#include "common_calc_pvt.h"



/** Max position accuracy we allow to output a SPP solution */
#define MAX_SPP_ACCURACY_M 100.0
/** Max velocity accuracy we allow to output a SPP solution */
#define MAX_SPP_VEL_ACCURACY_M_PER_S 10.0

void send_observations(u8 n,
                       u32 msg_obs_max_size,
                       const navigation_measurement_t m[],
                       const gps_time_t *t)
{
  static u8 buff[256];

  if ((0 == n) || (NULL==m) || (NULL==t)) {
    gps_time_t t_dummy = GPS_TIME_UNKNOWN;
    pack_obs_header(&t_dummy, 1, 0, (observation_header_t*) buff);
    sbp_send_msg(SBP_MSG_OBS,
                 sizeof(observation_header_t),
                 buff);
    return;
  }

  /* Upper limit set by SBP framing size, preventing underflow */
  u16 msg_payload_size = MAX(
      MIN((u16)MAX(msg_obs_max_size, 0), SBP_FRAMING_MAX_PAYLOAD_SIZE),
      sizeof(observation_header_t)
    ) - sizeof(observation_header_t);

  /* Lower limit set by sending at least 1 observation */
  msg_payload_size = MAX(msg_payload_size, sizeof(packed_obs_content_t));

  /* Round down the number of observations per message */
  u16 obs_in_msg = msg_payload_size / sizeof(packed_obs_content_t);

  /* Round up the number of messages */
  u16 total = MIN((n + obs_in_msg - 1) / obs_in_msg,
                  MSG_OBS_HEADER_MAX_SIZE);

  u8 obs_i = 0;
  for (u8 count = 0; count < total && obs_i < n; count++) {
    u8 curr_n = MIN(n - obs_i, obs_in_msg);
    pack_obs_header(t, total, count, (observation_header_t*) buff);
    packed_obs_content_t *obs = (packed_obs_content_t *)&buff[sizeof(observation_header_t)];

    for (u8 i = 0; i < obs_in_msg && obs_i < n; i++) {
      if (pack_obs_content(m[obs_i].raw_pseudorange,
                           m[obs_i].raw_carrier_phase,
                           m[obs_i].raw_measured_doppler,
                           m[obs_i].cn0,
                           m[obs_i].lock_time,
                           m[obs_i].flags,
                           m[obs_i].sid,
                           &obs[i]) >= 0) {
        /* Packed. */
        obs_i++;
      }
    }

    sbp_send_msg(SBP_MSG_OBS,
          sizeof(observation_header_t) + curr_n * sizeof(packed_obs_content_t),
          buff);
  }
}


/** Extract the full covariance matrices from soln struct */
void extract_covariance(double full_covariance[9], double vel_covariance[9],
                        const gnss_solution *soln) {

  assert(soln != NULL);
  assert(full_covariance != NULL);
  assert(vel_covariance != NULL);

/* soln->cov_err has the covariance in upper triangle covariance form, so
 * copy from
 *
 *    0  1  2       0  1  2
 *    _  3  4   to  3  4  5
 *    _  _  5       6  7  8  */

  full_covariance[0] = soln->err_cov[0];
  full_covariance[1] = soln->err_cov[1];
  full_covariance[2] = soln->err_cov[2];
  full_covariance[3] = soln->err_cov[1];
  full_covariance[4] = soln->err_cov[3];
  full_covariance[5] = soln->err_cov[4];
  full_covariance[6] = soln->err_cov[2];
  full_covariance[7] = soln->err_cov[4];
  full_covariance[8] = soln->err_cov[5];

  vel_covariance[0] = soln->vel_cov[0];
  vel_covariance[1] = soln->vel_cov[1];
  vel_covariance[2] = soln->vel_cov[2];
  vel_covariance[3] = soln->vel_cov[1];
  vel_covariance[4] = soln->vel_cov[3];
  vel_covariance[5] = soln->vel_cov[4];
  vel_covariance[6] = soln->vel_cov[2];
  vel_covariance[7] = soln->vel_cov[4];
  vel_covariance[8] = soln->vel_cov[5];

}



bool gate_covariance(gnss_solution *soln) {
  assert(soln != NULL);
  double full_covariance[9];
  double vel_covariance[9];
  extract_covariance(full_covariance, vel_covariance, soln);

  double accuracy, h_accuracy, v_accuracy;
  covariance_to_accuracy(full_covariance, soln->pos_ecef,
                         &accuracy, &h_accuracy, &v_accuracy);
  if (accuracy > MAX_SPP_ACCURACY_M) {
    log_warn("SPP Position suppressed due to position confidence of %.1f exceeding %.0f m",
             accuracy, MAX_SPP_ACCURACY_M);
    return true;
  }
  covariance_to_accuracy(vel_covariance, soln->pos_ecef,
                         &accuracy, &h_accuracy, &v_accuracy);
  if (accuracy > MAX_SPP_VEL_ACCURACY_M_PER_S) {
    log_warn("SPP Position suppressed due to velocity confidence of %.1f exceeding %.0f m/s",
             accuracy, MAX_SPP_VEL_ACCURACY_M_PER_S);
    return true;
  }
  return false;
}
