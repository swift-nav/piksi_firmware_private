/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>
#include <libsbp/system.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include "acq/manage.h"
#include "board/frontend.h"
#include "board/nap/track_channel.h"
#include "calc/calc_base_obs.h"
#include "calc/calc_pvt_me.h"
#include "calc/firmware_starling.h"
#include "cfg/init.h"
#include "decode/decode.h"
#include "ephemeris/ephemeris.h"
#include "ext_events/ext_events.h"
#include "glo_map_setup/glo_map_setup.h"
#include "hal/can_enabled.h"
#include "hal/is_duro.h"
#include "io_support.h"
#include "ndb/ndb.h"
#include "peripherals/can_termination.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "pps/pps.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings_client.h"
#include "simulator/simulator.h"
#include "specan/specan_main.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"
#include "track/track_state.h"
#include "version.h"

extern void ext_setup(void);

/* References:
   https://wiki.osdev.org/C++#GCC
   https://lists.debian.org/debian-gcc/2003/07/msg00057.html
   https://stackoverflow.com/questions/34308720/where-is-dso-handle-defined */
void *__dso_handle = NULL;

int main(void) {
  halInit();

  /* Kernel initialization, the main() function becomes a thread with
   * priority NORMALPRIO and the RTOS is active. */
  chSysInit();
  pre_init();
  io_support_init();

  /* SBP setup is needed to allow sbp fileio to work for factory data */
  sbp_setup();
  /* ignore return since log_error is sent by factory_params_read */
  factory_params_read();
  u16 sbp_sender_id = sender_id_get();
  /* avoid special `sbp_sender_id`s in the 0-64k range */
  while ((MSG_FORWARD_SENDER_ID == sbp_sender_id) ||
         (SBP_SENDER_ID == sbp_sender_id)) {
    sbp_sender_id = (u16)rand();
  }
  sbp_sender_id_set(sbp_sender_id);

  /* the settings interface will now pick up the correct `sbp_sender_id` */
  settings_api_setup();
  timing_setup();

  /* these messages now come out with the correct `sbp_sender_id` */
  log_info("Piksi Starting...");
  log_info("pfwp_build_id: " GIT_VERSION "");
  log_info("pfwp_build_date: " __DATE__ " " __TIME__ "");

  firmware_starling_preinit();

  init();
  signal_db_init();

  /* Initialize receiver time to the Jan 1980 with large enough uncertainty */
  gps_time_t t0 = {.tow = 0, .wn = 0};
  set_time(0, &t0, 2e9);

  static char sender_id_str[5];
  sprintf(sender_id_str, "%04X", sbp_sender_id);

  static char hw_revision_string[64] = {0};
  hw_revision_string_get(hw_revision_string);
  log_info("hw_revision: %s", hw_revision_string);

  static char nap_version_string[64] = {0};
  nap_version_string_get(nap_version_string);
  log_info("nap_build_id: %s", nap_version_string);

  static char nap_date_string[64] = {0};
  nap_date_string_get(nap_date_string);

  static char mfg_id_string[18] = {0};
  mfg_id_string_get(mfg_id_string, sizeof(mfg_id_string));
  log_info("mfg_serial_number: %s", mfg_id_string);

  static char mac_address_string[18] = {0};
  mac_address_string_get(mac_address_string);

  static char uuid_string[37] = {0};
  uuid_string_get(uuid_string);

  static char hw_version_string[16] = {0};
  hw_version_string_get(hw_version_string);
  log_info("hw_version: %s", hw_version_string);

  nap_auth_setup();
  nap_auth_check();

  ndb_setup();
  ephemeris_setup();

  frontend_setup();
  me_settings_setup();

  ext_event_setup();
  position_setup();
  glo_map_setup();
  track_setup();
  decode_setup();

  manage_acq_setup();
  system_monitor_setup();

  firmware_starling_setup();
  me_calc_pvt_setup();
  base_obs_setup();

  simulator_setup();

  ext_setup();
  pps_setup();

  SETTING_READONLY(
      "system_info", "sbp_sender_id", sender_id_str, SETTINGS_TYPE_STRING);
  SETTING_READONLY(
      "system_info", "serial_number", mfg_id_string, SETTINGS_TYPE_STRING);
  SETTING_READONLY(
      "system_info", "pfwp_build_id", GIT_VERSION, SETTINGS_TYPE_STRING);
  SETTING_READONLY("system_info",
                   "pfwp_build_date",
                   __DATE__ " " __TIME__,
                   SETTINGS_TYPE_STRING);
  SETTING_READONLY(
      "system_info", "nap_build_id", nap_version_string, SETTINGS_TYPE_STRING);
  SETTING_READONLY(
      "system_info", "nap_build_date", nap_date_string, SETTINGS_TYPE_STRING);
  static int nap_track_n_channels = ME_CHANNELS;
  SETTING_READONLY(
      "system_info", "nap_channels", nap_track_n_channels, SETTINGS_TYPE_INT);

  SETTING_READONLY(
      "system_info", "mac_address", mac_address_string, SETTINGS_TYPE_STRING);
  SETTING_READONLY("system_info", "uuid", uuid_string, SETTINGS_TYPE_STRING);

  /* Send message to inform host we are up and running. */
  u32 startup_flags = 0;
  sbp_send_msg(SBP_MSG_STARTUP, sizeof(startup_flags), (u8 *)&startup_flags);

  /* send Iono correction, L2C capabilities if valid */
  ionosphere_t iono;
  if (ndb_iono_corr_read(&iono) == NDB_ERR_NONE) {
    sbp_send_iono(&iono);
  }

  gnss_capb_send_over_sbp();

  SpecanStart();

  if (device_is_duro() && can_enabled()) {
    can1_termination_init();
  }

  while (1) {
    chThdSleepSeconds(60);
  }
}
