/* Discrete PI speed controller, same structure as the Simulink "Discrete PID Controller" used in
 * atlas_wheel_speed_loop.slx: forward-Euler integrator, output saturation, clamping anti-windup. */
#ifndef ATLAS_PI_H
#define ATLAS_PI_H
typedef struct { float kp, ki, ts, umin, umax, integ; } atlas_pi_t;
void  atlas_pi_init(atlas_pi_t *c, float kp, float ki, float ts, float umin, float umax);
float atlas_pi_step(atlas_pi_t *c, float ref, float meas);
void  atlas_pi_reset(atlas_pi_t *c);
#endif
