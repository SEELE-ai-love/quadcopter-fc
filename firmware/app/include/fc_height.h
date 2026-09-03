#ifndef FC_HEIGHT_H
#define FC_HEIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float height;       /* filtered height, m */
    float rate;         /* vertical rate, m/s (positive up) */
    float alpha;        /* measurement smoothing 0..1 */
    int   initialized;
    float prev_raw;
} fc_height_t;

void fc_height_init(fc_height_t *h);
void fc_height_update(fc_height_t *h, float raw_height_m, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_HEIGHT_H */
