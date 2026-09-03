#ifndef FC_PID_H
#define FC_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_limit;   /* absolute output limit */
    float int_limit;   /* absolute integral limit */
    float integral;
    float prev_error;
} fc_pid_t;

void  fc_pid_reset(fc_pid_t *pid);
float fc_pid_update(fc_pid_t *pid, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_PID_H */
