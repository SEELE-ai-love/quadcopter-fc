#include "fc_safety.h"

void fc_safety_init(fc_safety_t *fs)
{
    if (fs == 0) {
        return;
    }
    fs->state = FC_STATE_DISARMED;
    fs->link_ok = 0;
    fs->low_voltage = 0;
    fs->imu_ok = 0;
    fs->throttle_low = 0;
}

void fc_safety_request_arm(fc_safety_t *fs)
{
    if (fs == 0) {
        return;
    }
    if (fs->state == FC_STATE_DISARMED && fs->link_ok && fs->imu_ok &&
        !fs->low_voltage && fs->throttle_low) {
        fs->state = FC_STATE_ARMED;
    }
}

void fc_safety_request_disarm(fc_safety_t *fs)
{
    if (fs == 0) {
        return;
    }
    fs->state = FC_STATE_DISARMED;
}

void fc_safety_update(fc_safety_t *fs)
{
    if (fs == 0) {
        return;
    }

    if (!fs->imu_ok) {
        if (fs->state != FC_STATE_DISARMED) {
            fs->state = FC_STATE_EMERGENCY;
        }
        return;
    }

    switch (fs->state) {
    case FC_STATE_DISARMED:
    case FC_STATE_EMERGENCY:
        break;
    case FC_STATE_ARMED:
        if (!fs->link_ok) {
            fs->state = FC_STATE_FAILSAFE;
        } else if (fs->low_voltage) {
            fs->state = FC_STATE_LANDING;
        }
        break;
    case FC_STATE_LANDING:
        if (!fs->link_ok) {
            fs->state = FC_STATE_FAILSAFE;
        }
        break;
    case FC_STATE_FAILSAFE:
        /* Motor manager observes this state and lands/locks the aircraft. */
        break;
    }
}

const char *fc_safety_name(fc_state_t state)
{
    switch (state) {
    case FC_STATE_DISARMED:  return "DISARMED";
    case FC_STATE_ARMED:     return "ARMED";
    case FC_STATE_LANDING:   return "LANDING";
    case FC_STATE_FAILSAFE:  return "FAILSAFE";
    case FC_STATE_EMERGENCY: return "EMERGENCY";
    default:                 return "UNKNOWN";
    }
}
