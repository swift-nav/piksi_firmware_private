static void starling_thread(void *arg) {
  (void)arg;
  msg_t ret;

  chRegSetThreadName("starling");

  sbp_messages_t sbp_messages;

  static navigation_measurement_t nav_meas[MAX_CHANNELS];
  static ephemeris_t e_meas[MAX_CHANNELS];
  static gps_time_t obs_time;

  chMtxLock(&spp_filter_manager_lock);
  spp_filter_manager = create_filter_manager_spp();
  filter_manager_init(spp_filter_manager);
  chMtxUnlock(&spp_filter_manager_lock);

  u8 base_station_sender_id = 0;

  while (TRUE) {
    watchdog_notify(WD_NOTIFY_STARLING);

    me_msg_t *me_msg = NULL;
    ret = chMBFetch(&me_msg_mailbox, (msg_t *)&me_msg, DGNSS_TIMEOUT_MS);
    if (ret != MSG_OK) {
      if (NULL != me_msg) {
        log_error("STARLING: mailbox fetch failed with %" PRIi32, ret);
        chPoolFree(&me_msg_buff_pool, me_msg);
      }
      continue;
    }

    /* forward SBAS raw message to SPP filter manager*/
    if (ME_MSG_SBAS_RAW == me_msg->id) {
      const gps_time_t current_time = get_current_time();
      if (gps_time_valid(&current_time)) {
        sbas_raw_data_t sbas_data;
        unpack_sbas_raw_data(&me_msg->msg.sbas, &sbas_data);

        /* fill the week number from current time */
        gps_time_match_weeks(&sbas_data.time_of_transmission, &current_time);

        sbas_system_t sbas_system = get_sbas_system(sbas_data.sid);

        chMtxLock(&spp_filter_manager_lock);
        if (sbas_system != current_sbas_system &&
            SBAS_UNKNOWN != current_sbas_system) {
          /* clear existing SBAS corrections when provider changes */
          filter_manager_reinitialize_sbas(spp_filter_manager);
        }
        filter_manager_process_sbas_message(spp_filter_manager, &sbas_data);
        chMtxUnlock(&spp_filter_manager_lock);

        current_sbas_system = sbas_system;
      }
      chPoolFree(&me_msg_buff_pool, me_msg);
      continue;
    }

    if (ME_MSG_OBS != me_msg->id) {
      chPoolFree(&me_msg_buff_pool, me_msg);
      continue;
    }

    me_msg_obs_t *rover_channel_epoch = &me_msg->msg.obs;

    /* Init the messages we want to send */

    gps_time_t epoch_time = rover_channel_epoch->obs_time;
    if (!gps_time_valid(&epoch_time) && TIME_PROPAGATED <= get_time_quality()) {
      /* observations do not have valid time, but we have a reasonable estimate
       * of current GPS time, so round that to nearest epoch and use it
       */
      epoch_time = get_current_time();
      epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
    }

    sbp_messages_init(&sbp_messages, &epoch_time);

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {
      solution_simulation(&sbp_messages);
      const u8 fake_base_sender_id = 1;
      solution_send_low_latency_output(fake_base_sender_id,
                                       &sbp_messages,
                                       rover_channel_epoch->size,
                                       rover_channel_epoch->obs);
      chPoolFree(&me_msg_buff_pool, me_msg);
      continue;
    }

    if (rover_channel_epoch->size == 0 ||
        !gps_time_valid(&rover_channel_epoch->obs_time)) {
      chPoolFree(&me_msg_buff_pool, me_msg);
      solution_send_low_latency_output(0, &sbp_messages, 0, nav_meas);
      continue;
    }

    if (gpsdifftime(&rover_channel_epoch->obs_time, &obs_time) <= 0.0) {
      /* When we change the solution rate down, we sometimes can round the
       * time to an epoch earlier than the previous one processed, in that
       * case we want to ignore any epochs with an earlier timestamp */
      chPoolFree(&me_msg_buff_pool, me_msg);
      continue;
    }

    starling_frequency = soln_freq_setting;

    u8 n_ready = rover_channel_epoch->size;
    memset(nav_meas, 0, sizeof(nav_meas));

    if (n_ready) {
      MEMCPY_S(nav_meas,
               sizeof(nav_meas),
               rover_channel_epoch->obs,
               n_ready * sizeof(navigation_measurement_t));
    }

    memset(e_meas, 0, sizeof(e_meas));

    if (n_ready) {
      MEMCPY_S(e_meas,
               sizeof(e_meas),
               rover_channel_epoch->ephem,
               n_ready * sizeof(ephemeris_t));
    }

    obs_time = rover_channel_epoch->obs_time;

    chPoolFree(&me_msg_buff_pool, me_msg);

    ionosphere_t i_params;
    /* get iono parameters if available, otherwise use default ones */
    if (ndb_iono_corr_read(&i_params) != NDB_ERR_NONE) {
      i_params = DEFAULT_IONO_PARAMS;
    }
    chMtxLock(&time_matched_iono_params_lock);
    has_time_matched_iono_params = true;
    time_matched_iono_params = i_params;
    chMtxUnlock(&time_matched_iono_params_lock);
    chMtxLock(&spp_filter_manager_lock);
    filter_manager_update_iono_parameters(spp_filter_manager, &i_params, false);
    chMtxUnlock(&spp_filter_manager_lock);

    dops_t dops;

    // This will duplicate pointers to satellites with mutliple frequencies,
    // but this scenario is expected and handled
    const ephemeris_t *stored_ephs[MAX_CHANNELS];
    memset(stored_ephs, 0, sizeof(stored_ephs));
    for (u8 i = 0; i < n_ready; i++) {
      navigation_measurement_t *nm = &nav_meas[i];
      ephemeris_t *e = NULL;

      /* Find the original index of this measurement in order to point to
       * the correct ephemeris. (Do not load it again from NDB because it may
       * have changed meanwhile.) */
      for (u8 j = 0; j < n_ready; j++) {
        if (sid_is_equal(nm->sid, e_meas[j].sid)) {
          e = &e_meas[j];
          break;
        }
      }

      if (e == NULL || 1 != ephemeris_valid(e, &nm->tot)) {
        continue;
      }

      stored_ephs[i] = e;
    }

    pvt_engine_result_t result_spp;
    result_spp.valid = false;
    bool successful_spp = false;

    chMtxLock(&spp_filter_manager_lock);
    const PVT_ENGINE_INTERFACE_RC spp_call_filter_ret =
        call_pvt_engine_filter(spp_filter_manager,
                               &obs_time,
                               n_ready,
                               nav_meas,
                               stored_ephs,
                               starling_frequency,
                               &result_spp,
                               &dops);
    chMtxUnlock(&spp_filter_manager_lock);

    if (spp_call_filter_ret == PVT_ENGINE_SUCCESS) {
      solution_make_sbp(&result_spp, &dops, &sbp_messages);
      successful_spp = true;
    } else {
      if (dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        /* If we can't report a SPP position, something is wrong and no point
         * continuing to process this epoch - send out solution and
         * observation failed messages if not in time matched mode.
         */
        solution_send_low_latency_output(0, &sbp_messages, n_ready, nav_meas);
      }
      continue;
    }

    if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY && successful_spp) {
      chMtxLock(&low_latency_filter_manager_lock);

      pvt_engine_result_t result_rtk;
      result_rtk.valid = false;
      const PVT_ENGINE_INTERFACE_RC rtk_call_filter_ret =
          call_pvt_engine_filter(low_latency_filter_manager,
                                 &obs_time,
                                 n_ready,
                                 nav_meas,
                                 stored_ephs,
                                 starling_frequency,
                                 &result_rtk,
                                 &dops);
      base_station_sender_id = current_base_sender_id;
      chMtxUnlock(&low_latency_filter_manager_lock);

      if (rtk_call_filter_ret == PVT_ENGINE_SUCCESS) {
        solution_make_baseline_sbp(
            &result_rtk, result_spp.baseline, &dops, &sbp_messages);
      }
    }

    /* This is posting the rover obs to the mailbox to the time matched thread,
     * we want these sent on at full rate */
    /* Post the observations to the mailbox. */
    post_observations(n_ready, nav_meas, &obs_time, &result_spp);

    solution_send_low_latency_output(
        base_station_sender_id, &sbp_messages, n_ready, nav_meas);
  }
}
