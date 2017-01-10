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
#include <libswiftnav/logging.h>

#include <hal.h>
#include <ch.h>

#include "peripherals/leds.h"
#include "io_support.h"
#include "board/frontend.h"
#include "sbp.h"
#include "init.h"
#include "manage.h"
#include "track.h"
#include "timing.h"
#include "ext_events.h"
#include "solution.h"
#include "base_obs.h"
#include "position.h"
#include "system_monitor.h"
#include "simulator.h"
#include "settings.h"
#include "ephemeris.h"
#include "pps.h"
#include "decode.h"
#include "signal.h"
#include "version.h"
#include "ndb.h"
#include "sbp_utils.h"

extern void ext_setup(void);

void* __dso_handle(void){
  return (void*)0;
};

int main(void)
{
  halInit();

  /* Kernel initialization, the main() function becomes a thread with
   * priority NORMALPRIO and the RTOS is active. */
  chSysInit();

  /* Piksi hardware initialization. */
  pre_init();

  io_support_init();
  sbp_setup();
  settings_setup();

  board_preinit_hook();

  log_info("Piksi Starting...");
  log_info("pfwp_build_id: " GIT_VERSION "");
  log_info("pfwp_build_date: " __DATE__ " " __TIME__ "");

  init();
  signal_init();

  static u16 sender_id;
  sender_id = sender_id_get();

  if (sender_id == 0) {
    /* TODO: Handle this properly! */
    sender_id = (u16) rand();
  }
  /* We only need 16 bits for sender ID for sbp */
  
  sbp_sender_id_set(sender_id);

  ndb_setup();
  ephemeris_setup();

  static char sender_id_str[5];
  sprintf(sender_id_str, "%04X", sender_id);

  static char hw_revision_string[64] = {0};
  hw_revision_string_get(hw_revision_string);
  log_info("HW revision: %s", hw_revision_string);

  static char nap_version_string[64] = {0};
  nap_version_string_get(nap_version_string);
  log_info("NAP build id: %s", nap_version_string);

  static char mfg_id_string[18] = {0};
  mfg_id_string_get(mfg_id_string);
  log_info("Mfg serial number: %s", mfg_id_string);

  static char mac_address_string[18] = {0};
  mac_address_string_get(mac_address_string);
  
  static char uuid_string[37] = {0};
  uuid_string_get(uuid_string);

  frontend_setup();
  timing_setup();
  ext_event_setup();
  position_setup();
  track_setup();
  decode_setup();

  manage_acq_setup();
  manage_track_setup();
  system_monitor_setup();
  base_obs_setup();
  solution_setup();

  simulator_setup();

  ext_setup();
  pps_setup();

  READ_ONLY_PARAMETER("system_info", "sbp_sender_id", sender_id_str, TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "serial_number", mfg_id_string,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "pfwp_build_id", GIT_VERSION,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "pfwp_build_date", __DATE__ " " __TIME__,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "hw_revision", hw_revision_string,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "nap_build_id", nap_version_string,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "nap_channels", nap_track_n_channels,
                      TYPE_INT);

  READ_ONLY_PARAMETER("system_info", "mac_address", mac_address_string,
                      TYPE_STRING);
  READ_ONLY_PARAMETER("system_info", "uuid", uuid_string,
                      TYPE_STRING);

  /* Send message to inform host we are up and running. */
  u32 startup_flags = 0;
  sbp_send_msg(SBP_MSG_STARTUP, sizeof(startup_flags), (u8 *)&startup_flags);


  /* send Iono correction, L2C capabilities if valid */
  ionosphere_t iono;
  if (ndb_iono_corr_read(&iono) == NDB_ERR_NONE)
  {
    sbp_send_iono(&iono);
  }

  u32 l2c_mask;
  if (ndb_gps_l2cm_l2c_cap_read(&l2c_mask) == NDB_ERR_NONE)
  {
    sbp_send_l2c_capabilities(&l2c_mask);
  }

  while (1) {
    chThdSleepSeconds(60);
  }
}
