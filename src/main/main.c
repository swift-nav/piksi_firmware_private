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

#include <stdlib.h>
#include <string.h>

#include <libsbp/system.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include <hal.h>
#include "calc_pvt_me.h"

#include "board/frontend.h"
#include "board/nap/track_channel.h"
#include "calc_base_obs.h"
#include "decode.h"
#include "ephemeris/ephemeris.h"
#include "ext_events/ext_events.h"
#include "firmware_starling.h"
#include "glo_map_setup/glo_map_setup.h"
#include "init.h"
#include "io_support.h"
#include "manage.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "pps/pps.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "specan/specan_main.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"
#include "track/track_state.h"
#include "version.h"

extern void ext_setup(void);

void* __dso_handle(void) { return (void*)0; };

int main(void) {
  halInit();

  /* Kernel initialization, the main() function becomes a thread with
   * priority NORMALPRIO and the RTOS is active. */
  chSysInit();

  /* Piksi hardware initialization. */
  pre_init();

  io_support_init();
  sbp_setup();
  settings_setup();
  timing_setup();

  log_info("Piksi Starting...");
  log_info("pfwp_build_id: " GIT_VERSION "");
  log_info("pfwp_build_date: " __DATE__ " " __TIME__ "");

  starling_initialize_api();
  init();
  signal_db_init();

  static u16 sender_id;
  sender_id = sender_id_get();

  if (sender_id == 0) {
    /* TODO: Handle this properly! */
    sender_id = (u16)rand();
  }
  /* We only need 16 bits for sender ID for sbp */

  sbp_sender_id_set(sender_id);

  /* Initialize receiver time to the Jan 1980 with large enough uncertainty */
  gps_time_t t0 = {.tow = 0, .wn = 0};
  set_time(nap_timing_count(), &t0, 2e9);

  log_info("starting ndb");
  ndb_setup();
  log_info("finished ndb");
  ephemeris_setup();
  log_info("finished ephemeris");

  static char sender_id_str[5];
  sprintf(sender_id_str, "%04X", sender_id);

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
  log_info("hello 1");
  nap_auth_check();
  log_info("hello 2");

  frontend_setup();
  log_info("hello 3");
  me_settings_setup();
  log_info("hello 4");

  ext_event_setup();
  log_info("hello 5");
  position_setup();
  log_info("hello 6");
  glo_map_setup();
  log_info("hello 7");
  track_setup();
  log_info("hello 8");
  decode_setup();
  log_info("hello 9");

  manage_acq_setup();
  log_info("hello 10");
  system_monitor_setup();
  log_info("hello 11");

  nmea_setup();
  log_info("hello 12");

  firmware_starling_setup();
  log_info("hello 13");
  me_calc_pvt_setup();
  log_info("hello 14");
  base_obs_setup();
  log_info("hello 15");

  simulator_setup();
  log_info("hello 16");

  ext_setup();
  log_info("hello 17");
  pps_setup();
  log_info("hello 18");

  READ_ONLY_PARAMETER(
      "system_info", "sbp_sender_id", sender_id_str, TYPE_STRING);
  READ_ONLY_PARAMETER(
      "system_info", "serial_number", mfg_id_string, TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "pfwp_build_id", GIT_VERSION, TYPE_STRING);
  READ_ONLY_PARAMETER(
      "system_info", "pfwp_build_date", __DATE__ " " __TIME__, TYPE_STRING);
  READ_ONLY_PARAMETER(
      "system_info", "nap_build_id", nap_version_string, TYPE_STRING);
  READ_ONLY_PARAMETER(
      "system_info", "nap_build_date", nap_date_string, TYPE_STRING);
  READ_ONLY_PARAMETER(
      "system_info", "nap_channels", nap_track_n_channels, TYPE_INT);

  READ_ONLY_PARAMETER(
      "system_info", "mac_address", mac_address_string, TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "uuid", uuid_string, TYPE_STRING);

  /* Send message to inform host we are up and running. */
  u32 startup_flags = 0;
  log_info("sending start msg");
  sbp_send_msg(SBP_MSG_STARTUP, sizeof(startup_flags), (u8*)&startup_flags);

  /* send Iono correction, L2C capabilities if valid */
  ionosphere_t iono;
  if (ndb_iono_corr_read(&iono) == NDB_ERR_NONE) {
    sbp_send_iono(&iono);
  }

  gnss_capb_send_over_sbp();

  SpecanStart();

  while (1) {
    chThdSleepSeconds(60);
  }
}
