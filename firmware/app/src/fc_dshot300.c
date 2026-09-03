#include "fc_dshot300.h"

uint16_t fc_dshot_encode(uint16_t value, bool telemetry_request)
{
    if (value > FC_DSHOT_VALUE_MAX) {
        value = FC_DSHOT_VALUE_MAX;
    }

    uint16_t frame = value & 0x07FFU;
    if (telemetry_request) {
        frame |= FC_DSHOT_TELEMETRY_BIT;
    }

    /* CRC is the XOR of the four nibbles of the 12-bit payload. */
    uint8_t crc = (uint8_t)((frame >> 8) & 0x0FU) ^
                  (uint8_t)((frame >> 4) & 0x0FU) ^
                  (uint8_t)(frame & 0x0FU);

    return (uint16_t)((frame << 4) | (crc & 0x0FU));
}

uint16_t fc_dshot_value_from_normalized(float throttle01)
{
    if (throttle01 <= 0.0f) {
        return 0U;
    }
    if (throttle01 >= 1.0f) {
        return FC_DSHOT_VALUE_MAX;
    }
    float v = FC_DSHOT_VALUE_MIN +
              throttle01 * (float)(FC_DSHOT_VALUE_MAX - FC_DSHOT_VALUE_MIN);
    return (uint16_t)(v + 0.5f);
}

void fc_dshot_build_frames(const float motor[4], uint16_t frames[4],
                           bool telemetry_request)
{
    for (int i = 0; i < 4; ++i) {
        uint16_t value = fc_dshot_value_from_normalized(motor[i]);
        frames[i] = fc_dshot_encode(value, telemetry_request && (i == 0));
    }
}
