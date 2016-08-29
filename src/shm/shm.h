



/* Possible satellite code states */
typedef enum {
  CODE_NAV_STATE_UNKNOWN,
  CODE_NAV_STATE_INVALID,
  CODE_NAV_STATE_VALID,
} code_nav_state_t;

void shm_gps_set_shi1(u16 sat, u8 new_value);
void shm_gps_set_shi4(u16 sat, bool new_value);
void shm_gps_set_shi6(u16 sat, bool new_value);

code_nav_state_t shm_get_sat_state(gnss_signal_t sid);

bool shm_gps_l1ca_tracking_allowed(u16 sat);
bool shm_gps_l2cm_tracking_allowed(u16 sat);
bool shm_gps_navigation_suitable(gnss_signal_t sid);



