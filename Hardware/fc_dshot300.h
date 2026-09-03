#ifndef FC_DSHOT300_H
#define FC_DSHOT300_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FC_DSHOT_VALUE_MIN      48U   /* first armed throttle value */
#define FC_DSHOT_VALUE_MAX      2047U
#define FC_DSHOT_TELEMETRY_BIT  0x0800U

/* Build a complete 16-bit DShot frame including CRC nibble. */
uint16_t fc_dshot_encode(uint16_t value, bool telemetry_request);

/* Map normalized motor command 0..1 to a DShot throttle frame. */
uint16_t fc_dshot_value_from_normalized(float throttle01);

/* Encode four normalized motor commands to four DShot frames. */
void fc_dshot_build_frames(const float motor[4], uint16_t frames[4],
                           bool telemetry_request);

#ifdef __cplusplus
}
#endif

#endif /* FC_DSHOT300_H */
