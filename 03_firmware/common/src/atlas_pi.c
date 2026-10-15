#include "atlas_pi.h"
void atlas_pi_init(atlas_pi_t *c, float kp, float ki, float ts, float umin, float umax)
{ c->kp = kp; c->ki = ki; c->ts = ts; c->umin = umin; c->umax = umax; c->integ = 0.0f; }
void atlas_pi_reset(atlas_pi_t *c) { c->integ = 0.0f; }
float atlas_pi_step(atlas_pi_t *c, float ref, float meas)
{
    float e = ref - meas;
    float u_unsat = c->kp * e + c->integ;
    float u = u_unsat > c->umax ? c->umax : (u_unsat < c->umin ? c->umin : u_unsat);
    /* clamping: stop integrating while saturated AND the error would push further into saturation */
    int saturated = (u != u_unsat);
    int same_sign = (e * u_unsat) > 0.0f;
    if (!(saturated && same_sign)) c->integ += c->ki * c->ts * e;
    return u;
}
