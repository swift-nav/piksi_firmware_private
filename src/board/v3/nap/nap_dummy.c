
#include <string.h>

#include "board/nap/track_channel.h"

u8 nap_acq_fft_index_bits;
u8 nap_acq_downsample_stages;

u32 nap_error_rd_blocking(void)
{
  return 0;
}

bool nap_timing_strobe_wait(u32 timeout)
{
  (void)timeout;
  return true;
}

