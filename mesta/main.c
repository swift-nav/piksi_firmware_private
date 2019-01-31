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

#include "manage.h"
#include "nap/track_channel.h"
#include "platform_signal.h"
#include "settings/settings_client.h"
#include "signal_db/signal_db.h"
#include "track/track_common.h"
#include "track/track_state.h"

/* Forward decl. */
void init_starling_platform_stub_implementation(void);

swiftnap_t mesta_nap = {0};

void handle_nap_track_irq(void);

int main(void) {
  nap_track_n_channels = 73;

  init_starling_platform_stub_implementation();
  platform_track_setup();
  platform_decode_setup();
  track_setup();
  tp_set_rover_mode();
  signal_db_init();

  tracking_startup_params_t startup = {0};

  for (int sat = 1; sat <= 2; sat++) {
    startup.mesid = construct_mesid(CODE_GPS_L1CA, sat);
    /* startup.mesid = construct_mesid(CODE_GPS_L2CM, sat); */
    /* startup.mesid = construct_mesid(CODE_GLO_L1OF, sat); */
    /* startup.mesid = construct_mesid(CODE_GLO_L2OF, sat); */
    /* startup.mesid = construct_mesid(CODE_BDS2_B1, sat); */
    /* startup.mesid = construct_mesid(CODE_BDS2_B2, sat); */
    /* startup.mesid = construct_mesid(CODE_GAL_E1B, sat); */
    /* startup.mesid = construct_mesid(CODE_GAL_E7I, sat); */
    /* startup.mesid = construct_mesid(CODE_SBAS_L1CA, 120 + sat); */

    tracking_startup_request(&startup);
    manage_tracking_startup();
  }

  for (int i = 0; i < (int)ARRAY_SIZE(mesta_nap.TRK_CH_RD); i++) {
    u32 *corr = (u32 *)(uintptr_t)&mesta_nap.TRK_CH_RD[i].CORR;
    *corr = 0xFFFFFFFF;
  }

  memset((void *)mesta_nap.TRK_IRQS, 0xFF, sizeof(mesta_nap.TRK_IRQS));

  for (int i = 0; i < 10000; i++) {
    u32 *count = (u32 *)(uintptr_t)&mesta_nap.TIMING_COUNT;
    *count += 1000;
    handle_nap_track_irq();
  }

  return 0;
}
