#include "fc_height.h"

void fc_height_init(fc_height_t *h)
{
    if (h == 0) {
        return;
    }
    h->height = 0.0f;
    h->rate = 0.0f;
    h->alpha = 0.25f;
    h->initialized = 0;
    h->prev_raw = 0.0f;
}

void fc_height_update(fc_height_t *h, float raw_height_m, float dt)
{
    if ((h == 0) || (dt <= 0.0f)) {
        return;
    }

    if (!h->initialized) {
        h->height = raw_height_m;
        h->prev_raw = raw_height_m;
        h->initialized = 1;
        return;
    }

    float smoothed = h->height + h->alpha * (raw_height_m - h->height);
    float meas_rate = (smoothed - h->prev_raw) / dt;
    h->rate = 0.85f * h->rate + 0.15f * meas_rate;
    h->height = smoothed;
    h->prev_raw = raw_height_m;
}
