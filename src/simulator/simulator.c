/*
 * Copyright (C) 2012-2014, 2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "simulator.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/almanac.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/linear_algebra.h>

#include "calc/calc_pvt_me.h"
#include "hal/piksi_systime.h"
#include "peripherals/leds.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings_client.h"
#include "simulator_data.h"
#include "track/track_sid_db.h"

/** \addtogroup simulator
 * \{ */

u8 sim_enabled;

simulation_settings_t sim_settings = {
    .base_ecef = {-2706098.845, -4261216.475, 3885597.912},
    .speed = 4.0,
    .radius = 100.0,
    .pos_sigma = 1.5,
    .speed_sigma = .15,
    .cn0_sigma = 0.3,
    .pseudorange_sigma = 4,
    .phase_sigma = 3e-2,
    .num_sats = 9,
    .mode_mask = SIMULATION_MODE_PVT | SIMULATION_MODE_TRACKING |
                 SIMULATION_MODE_FLOAT | SIMULATION_MODE_RTK};

/* Internal Simulation State Definition */
struct {
  piksi_systime_t last_update; /**< The last simulation update happened at
                                    this CPU tick count. */
  float angle;                 /**< Current simulation angle in radians */
  double pos[3];               /**< Current simulated position with no
                                    noise, in ECEF coordinates. */
  double baseline[3];          /**< Current simulated baseline with no
                                    noise, in ECEF coordinates.*/
  double covariance[9];
  u8 num_sats_selected;

  tracking_channel_state_t tracking_channel[ME_CHANNELS];
  measurement_state_t state_meas[ME_CHANNELS];
  channel_measurement_t ch_meas[ME_CHANNELS];
  obs_array_t obs_array;
  obs_array_t base_obs_array;
  dops_t dops;
  pvt_engine_result_t noisy_solution;

} sim_state = {

    .last_update = PIKSI_SYSTIME_INIT,
    .angle = 0.0,
    .pos = {0.0, 0.0, 0.0},
    .baseline = {0.0, 0.0, 0.0},
    .covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    .num_sats_selected = 0,
    /* .state_meas left uninitialized */
    /* .ch_meas left uninitialized */
    /* .tracking_channel left uninitialized */
    /* .obs_array left uninitialized */
    /* .base_obs_array left uninitialized */
    .dops =
        {
            .pdop = 1.9,
            .gdop = 1.8,
            .tdop = 1.7,
            .hdop = 1.6,
            .vdop = 1.5,
        },
    /* .noisy_solution left uninitialized */

};

/** Generates a sample from the normal distribution
 * with given variance.
 *
 * Uses the Box-Muller transform which is insensitive
 * to the long tail of gaussians.
 *
 * Performs a square-root, a sin, a log, and a rand call.
 *
 * \param variance The variance of a zero-mean gaussian to draw a sample from.
 */
double rand_gaussian(const double variance) {
  static bool hasSpare = false;
  static double rand1, rand2;

  if (hasSpare) {
    hasSpare = false;
    return sqrt(variance * rand1) * sin(rand2);
  }

  hasSpare = true;

  rand1 = rand() / ((double)RAND_MAX);
  if (rand1 < FLT_MIN) {
    rand1 = FLT_MIN;
  }
  rand1 = -2 * log(rand1);
  rand2 = (rand() / ((double)RAND_MAX)) * (M_PI * 2.0);

  return sqrt(variance * rand1) * cos(rand2);
}

/** Performs a 1D linear interpolation from a point on the line segment
 * defined between points U and V, into a point on the line segment defined by
 * points X and Y.
 *
 * Assumes V > U, and Y > X
 *
 *
 * \param t The parametric variable ranging from [0,1] along [u,v]
 * \param u The start of the input range.
 * \param v The end of the input range.
 * \param x The start of the output range.
 * \param y The end of the output range.
 */
double lerp(double t, double u, double v, double x, double y) {
  return (t - u) / (v - u) * (y - x) + x;
}

/** Performs a timestep of the simulation that flies in a circle around a point.
 * Updates the sim_state and sim_state.noisy_solution structs.
 *
 * This simulator models a system moving in a perfect circle. We use this fact
 * to write a simple but smart numerically stable simulator.
 *
 * At every step, this simulator runs a simple forward euler integrator
 * on the position of the simulated point. This new position will not be
 * on the circular path we want to follow (an example numerical instability).
 * To avoid numerical instability, this simulator makes a small angle
 * approximation using this new position and the circle's desired
 * radius to calculate the new angle around the circle the point actually is.
 * This is stored in a single system variable "angle".
 * "angle" wraps around 2*PI, and is used to calculate the new position.
 *
 * We use the angle variable to calculate a new position
 *
 * Adds IID gaussian noise to the true position calculated at every timestep.
 *
 * This function makes a small angle approximation, so the
 * elapsed time (dt) between calls must be such that the (speed * dt) is much
 * less than the radius.
 *
 */
void simulation_step(void) {
  /* First we propagate the current fake PVT solution */
  piksi_systime_t now;
  piksi_systime_get(&now);

  double elapsed = 0;

  if (piksi_systime_cmp(&PIKSI_SYSTIME_INIT, &sim_state.last_update)) {
    elapsed = piksi_systime_sub_us(&now, &sim_state.last_update);
    elapsed /= SECS_US; /* us to s */
  }

  sim_state.last_update = now;

  /* Update the time, clamping it to the solution frequency */
  double new_tow = sim_state.noisy_solution.time.tow + elapsed;
  sim_state.noisy_solution.time.tow =
      round(new_tow * soln_freq_setting) / soln_freq_setting;

  /* Handle week-rollover. */
  normalize_gps_time(&sim_state.noisy_solution.time);

  simulation_step_position_in_circle(elapsed);
  simulation_step_tracking_and_observations(elapsed);
}

/**
 * Performs a simulation step for the given duration, by moving
 * our simulated position in a circle at a given radius and speed
 * around the simulation's center point.
 */
void simulation_step_position_in_circle(double elapsed) {
  /* Update the angle, making a small angle approximation. */
  sim_state.angle += (sim_settings.speed * elapsed) / sim_settings.radius;
  if (sim_state.angle > 2 * M_PI) {
    sim_state.angle = 0;
  }

  double pos_ned[3] = {sim_settings.radius * sinf(sim_state.angle),
                       sim_settings.radius * cosf(sim_state.angle),
                       0};

  /* Fill out position simulation's gnss_solution pos_ECEF, pos_LLH structures
   */
  wgsned2ecef_d(pos_ned, sim_settings.base_ecef, sim_state.pos);

  /* Calculate an accurate baseline for simulating RTK */
  vector_subtract(3, sim_state.pos, sim_settings.base_ecef, sim_state.baseline);

  /* Add gaussian noise to PVT position */
  double* pos_ecef = sim_state.noisy_solution.baseline;
  double pos_variance = sim_settings.pos_sigma * sim_settings.pos_sigma;
  pos_ecef[0] = sim_state.pos[0] + rand_gaussian(pos_variance);
  pos_ecef[1] = sim_state.pos[1] + rand_gaussian(pos_variance);
  pos_ecef[2] = sim_state.pos[2] + rand_gaussian(pos_variance);

  /* Calculate Velocity vector tangent to the sphere */
  double noisy_speed =
      sim_settings.speed +
      rand_gaussian(sim_settings.speed_sigma * sim_settings.speed_sigma);
  double vel_ned[3];
  vel_ned[0] = noisy_speed * cosf(sim_state.angle);
  vel_ned[1] = noisy_speed * -1.0 * sinf(sim_state.angle);
  vel_ned[2] = 0.0;

  wgsned2ecef(vel_ned,
              sim_state.noisy_solution.baseline,
              sim_state.noisy_solution.velocity);
  sim_state.noisy_solution.valid = true;
  sim_state.noisy_solution.velocity_valid = true;
}

/** Simulates real observations for the current position and the satellite
 * almanac and week
 * given in simulator_data.
 *
 * NOTES:
 *
 * - This simulates the pseudorange as the true distance to the satellite +
 * noise.
 * - This simulates the carrier phase as the true distance in wavelengths + bais
 * + noise.
 * - The bias is an integer multiple of 10 for easy debugging.
 * - The satellite C/N0 is proportional to the elevation of the satellite.
 *
 * USES:
 * - Pipe observations into internals for testing
 * - For integration testing with other devices that has to carry the radio
 * signal.
 *
 * \param elapsed Number of seconds elapsed since last simulation step.
 */
void simulation_step_tracking_and_observations(double elapsed) {
  (void)elapsed;

  gps_time_t t = sim_state.noisy_solution.time;

  /* First we calculate all the current sat positions, velocities */
  for (u8 i = 0; i < simulation_num_almanacs; i++) {
    double clock_err, clock_rate_err;
    s8 r = calc_sat_state_almanac(&simulation_almanacs[i],
                                  &t,
                                  simulation_sats_pos[i],
                                  simulation_sats_vel[i],
                                  simulation_sats_acc[i],
                                  &clock_err,
                                  &clock_rate_err);
    assert(r == 0);
  }

  /* Calculate the first sim_settings.num_sats amount of visible sats */
  u8 num_sats_selected = 0;
  double az, el;
  for (u8 i = 0; i < simulation_num_almanacs; i++) {
    s8 r = calc_sat_az_el_almanac(
        &simulation_almanacs[i], &t, sim_state.pos, &az, &el);
    assert(r == 0);
    track_sid_db_azel_degrees_set(
        simulation_almanacs[i].sid, az * R2D, el * R2D, nap_timing_count());

    if (el > 0 && num_sats_selected < sim_settings.num_sats &&
        num_sats_selected < ME_CHANNELS) {
      /* Generate a code measurement which is just the pseudorange: */
      double points_to_sat[3];
      double base_points_to_sat[3];
      vector_subtract(3, simulation_sats_pos[i], sim_state.pos, points_to_sat);
      vector_subtract(3,
                      simulation_sats_pos[i],
                      sim_settings.base_ecef,
                      base_points_to_sat);

      double distance_to_sat = vector_norm(3, points_to_sat);
      /* reuse points_to_sat to store a unit vector for dot product */
      vector_normalize(3, points_to_sat);
      double base_distance_to_sat = vector_norm(3, base_points_to_sat);
      double sat_vel_mag = vector_dot(3, points_to_sat, simulation_sats_vel[i]);
      /* Fill out the observation details into the obs structure for this
       * satellite, */
      /* We simulate the pseudorange as a noisy range measurement, and */
      /* the carrier phase as a noisy range in wavelengths + an integer offset.
       */

      populate_obs(&sim_state.obs_array.observations[num_sats_selected],
                   distance_to_sat,
                   el,
                   sat_vel_mag,
                   i);

      populate_obs(&sim_state.base_obs_array.observations[num_sats_selected],
                   base_distance_to_sat,
                   el,
                   sat_vel_mag,
                   i);

      /* As for tracking, we just set each sat consecutively in each channel. */
      /* This will cause weird jumps when a satellite rises or sets. */
      /** FIXME: do we really need the offset? */
      gnss_signal_t sid = simulation_almanacs[i].sid;
      sim_state.tracking_channel[num_sats_selected].sid.sat = sid.sat;
      sim_state.tracking_channel[num_sats_selected].sid.code = sid.code;
      sim_state.tracking_channel[num_sats_selected].fcn =
          0; /* FIXME: do properly */
      sim_state.state_meas[num_sats_selected].mesid.sat = sid.sat;
      sim_state.state_meas[num_sats_selected].mesid.code = sid.code;
      float fTmpCN0 = sim_state.obs_array.observations[num_sats_selected].cn0;
      fTmpCN0 = (fTmpCN0 <= 0) ? 0 : fTmpCN0;
      fTmpCN0 = (fTmpCN0 >= 63.75) ? 63.75 : fTmpCN0;
      sim_state.tracking_channel[num_sats_selected].cn0 = rintf(fTmpCN0 * 4.0);
      sim_state.state_meas[num_sats_selected].cn0 = rintf(fTmpCN0 * 4.0);

      sim_state.ch_meas[num_sats_selected].sid = sid;
      sim_state.ch_meas[num_sats_selected].cn0 =
          sim_state.obs_array.observations[num_sats_selected].cn0;

      num_sats_selected++;
    }
  }
  sim_state.obs_array.n = num_sats_selected;
  sim_state.base_obs_array.n = num_sats_selected;
  sim_state.obs_array.t = t;
  sim_state.base_obs_array.t = t;
  sim_state.noisy_solution.num_sats_used = num_sats_selected;
  sim_state.noisy_solution.num_sigs_used = num_sats_selected;
}

/** Populate a starling_obs_t structure with simulated data for
 * the almanac_i satellite, currently dist away from simulated point at given
 * elevation.
 *
 */
void populate_obs(starling_obs_t* obs,
                  double dist,
                  double elevation,
                  double vel,
                  int almanac_i) {
  obs->sid = simulation_almanacs[almanac_i].sid;

  obs->pseudorange = dist;
  obs->pseudorange += rand_gaussian(sim_settings.pseudorange_sigma *
                                    sim_settings.pseudorange_sigma);

  obs->carrier_phase =
      dist / (GPS_C / sid_to_carr_freq(simulation_almanacs[almanac_i].sid));
  obs->carrier_phase += simulation_fake_carrier_bias[almanac_i];
  obs->carrier_phase +=
      rand_gaussian(sim_settings.phase_sigma * sim_settings.phase_sigma);

  obs->doppler =
      vel / GPS_C * sid_to_carr_freq(simulation_almanacs[almanac_i].sid);
  obs->cn0 = lerp(elevation, 0, M_PI / 2, 35, 45) +
             rand_gaussian(sim_settings.cn0_sigma * sim_settings.cn0_sigma);
  obs->flags = 0xffff;
  /* Assume 5hz solution rate for sim mode */
  if (obs->lock_time <= 0) {
    obs->lock_time = 0.2;
  }
  obs->lock_time += 0.2;
}

/** Returns true if the simulation is at all enabled
 */
inline bool simulation_enabled(void) { return (sim_enabled > 0); }

/** Returns true fi the simulation is enabled for the given mode_mask
 *
 * \param mode_mask The mode for which the simulation might be enabled.
 */
inline bool simulation_enabled_for(simulation_modes_t mode_mask) {
  return (sim_enabled > 0) && ((sim_settings.mode_mask & mode_mask) > 0);
}

/** Get current simulated PVT solution
 * The structure returned by this changes every time simulation_step is called.
 */
inline pvt_engine_result_t* simulation_current_pvt_engine_result_t(void) {
  return &sim_state.noisy_solution;
}

/** Get current simulated DOPS.
 * The structure returned by this changes when settings are updated.
 */
inline dops_t* simulation_current_dops_solution(void) {
  return &sim_state.dops;
}

/** Get current simulated baseline reference point in ECEF coordinates.
 * The structure returned by this changes when settings are updated.
 */
inline double* simulation_ref_ecef(void) { return sim_settings.base_ecef; }

/** Get current simulated baseline vector in ECEF coordinates.
 * The structure returned by this changes every time simulation_step is called.
 */
inline double* simulation_current_baseline_ecef(void) {
  return sim_state.baseline;
}

inline double* simulation_current_covariance_ecef(void) {
  return sim_state.covariance;
}

/** Returns the number of satellites being simulated.
 */
u8 simulation_current_num_sats(void) {
  return sim_state.noisy_solution.num_sats_used;
}

/** Returns the number of satellites being simulated.
 */
channel_measurement_t* simulation_current_in_view(void) {
  return sim_state.ch_meas;
}

/** Returns the current simulated tracking loops state simulated.
 * This contains only noise, no interesting simulation information.
 *
 * \param channel The simulated tracking channel.
 */
tracking_channel_state_t simulation_current_tracking_state(u8 channel) {
  u8 num_sat = simulation_current_num_sats();
  if (channel >= num_sat) {
    channel = num_sat - 1;
  }
  return sim_state.tracking_channel[channel];
}

/** Returns the current simulated measurement state simulated.
 * This contains only noise, no interesting simulation information.
 *
 * \param channel The simulated tracking channel.
 */
measurement_state_t simulation_measurement_state(u8 channel) {
  u8 num_sat = simulation_current_num_sats();
  if (channel >= num_sat) {
    channel = num_sat - 1;
  }
  return sim_state.state_meas[channel];
}

/** Returns the simulated navigation measurement of our moving position.
 */
obs_array_t* simulation_current_obs(void) { return &sim_state.obs_array; }

/** Returns the simulated navigation measurement at the base position
 * for the simulation (aka the non-moving point around which the simulation
 * moves).
 * This is useful for testing RTK algorithms in hardware.
 */
obs_array_t* simulation_current_base_obs(void) {
  return &sim_state.base_obs_array;
}

/**
 * Do any setup we need for the satellite almanacs.
 */
void simulator_setup_almanacs(void) {
  for (u8 i = 0; i < simulation_num_almanacs; i++) {
    simulation_fake_carrier_bias[i] = (rand() % 1000) * 10;
  }
}

/** Must be called from main() or equivalent function before simulator runs
 */
void simulator_setup(void) {
  sim_state.noisy_solution.time.wn = simulation_week_number - 1;
  sim_state.noisy_solution.time.tow = WEEK_SECS - 20;
  sim_state.noisy_solution.valid = 1;
  sim_state.noisy_solution.velocity_valid = 1;

  simulator_setup_almanacs();

  SETTING("simulator", "enabled", sim_enabled, SETTINGS_TYPE_BOOL);
  SETTING("simulator",
          "base_ecef_x",
          sim_settings.base_ecef[0],
          SETTINGS_TYPE_FLOAT);
  SETTING("simulator",
          "base_ecef_y",
          sim_settings.base_ecef[1],
          SETTINGS_TYPE_FLOAT);
  SETTING("simulator",
          "base_ecef_z",
          sim_settings.base_ecef[2],
          SETTINGS_TYPE_FLOAT);
  SETTING("simulator", "speed", sim_settings.speed, SETTINGS_TYPE_FLOAT);
  SETTING("simulator", "radius", sim_settings.radius, SETTINGS_TYPE_FLOAT);
  SETTING(
      "simulator", "pos_sigma", sim_settings.pos_sigma, SETTINGS_TYPE_FLOAT);
  SETTING("simulator",
          "speed_sigma",
          sim_settings.speed_sigma,
          SETTINGS_TYPE_FLOAT);
  SETTING(
      "simulator", "cn0_sigma", sim_settings.cn0_sigma, SETTINGS_TYPE_FLOAT);
  SETTING("simulator",
          "pseudorange_sigma",
          sim_settings.pseudorange_sigma,
          SETTINGS_TYPE_FLOAT);
  SETTING("simulator",
          "phase_sigma",
          sim_settings.phase_sigma,
          SETTINGS_TYPE_FLOAT);
  SETTING("simulator", "num_sats", sim_settings.num_sats, SETTINGS_TYPE_INT);
  SETTING("simulator", "mode_mask", sim_settings.mode_mask, SETTINGS_TYPE_INT);
}

/** \} */
