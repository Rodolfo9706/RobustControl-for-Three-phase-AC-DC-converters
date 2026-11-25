#ifndef ROBUST_CONTROLLER_H
#define ROBUST_CONTROLLER_H

/*
 * Adaptive passivity-based controller, law (16) of TCST 26-1303,
 * discretized with forward Euler at the sampling period ts.
 * Equation numbers refer to the revised manuscript.
 */

#include <math.h>

typedef struct {
    /* Plant parameters (known to the controller) */
    float Em;               /* peak phase voltage, fundamental [V]     */
    float R;                /* line resistance [Ohm]                   */
    float L;                /* line inductance [H]                     */
    float omega;            /* source angular frequency [rad/s],
                               updated every sample from the PLL       */

    /* Voltage reference with optional ramp */
    float vod;              /* reference used by the law [V]           */
    float vod_target;       /* commanded reference [V]                 */
    float vod_rate;         /* max ramp rate [V/s], 0 = step           */

    /* Gains, Table V */
    float ks, kq;           /* port damping (16a)-(16b)                */
    float gamma;            /* estimator gain (16d)                    */
    float sigma;            /* sigma-modification (16d)                */
    float gc;               /* sigma-modification center [S]           */
    float kappa;            /* filtered proportional gain (16c) [S/V]  */
    float lambda_f;         /* filter bandwidth (16c) [rad/s]          */
    float nu_bar;           /* tanh bound (16c) [V]                    */
    float ad, aq;           /* saturation rectangle S, Eq. (5)         */

    /* Projection (14) */
    float g_low, g_up;      /* [g_low, g_up] [S]                       */
    float eps_p;            /* projection band [S]                     */

    /* Controller states */
    float g_hat;            /* conductance estimate, in G_eps          */
    float nu;               /* filtered voltage error                  */

    /* Monitored signals */
    float g_r, I_ref, e1, e3, s;

    float ts;               /* sampling period [s]                     */
} ControllerParams;

/* Saturation onto [-a, a] */
static inline float sat(float u, float a)
{
    if (u >  a) return  a;
    if (u < -a) return -a;
    return u;
}

/* Smooth projection (14), keeps g_hat in [g_low - eps_p, g_up + eps_p] */
static inline float Projection(float g, float y, float g_low, float g_up, float eps_p)
{
    float cp = (g - g_up) / eps_p;
    float cm = (g_low - g) / eps_p;
    cp = cp < 0.0f ? 0.0f : (cp > 1.0f ? 1.0f : cp);
    cm = cm < 0.0f ? 0.0f : (cm > 1.0f ? 1.0f : cm);
    return y - cp * (y > 0.0f ? y : 0.0f) - cm * (y < 0.0f ? y : 0.0f);
}

/* Equilibrium current I(g), Eq. (4) (smaller root of (3)) */
static inline float Compute_I_eq(const ControllerParams *c, float g)
{
    float arg = c->Em * c->Em - (8.0f / 3.0f) * c->R * c->vod * c->vod * g;
    if (arg < 0.0f) arg = 0.0f;            /* only if (17) is violated */
    return (c->Em - sqrtf(arg)) / (2.0f * c->R);
}

/*
 * One sample of (16). Inputs: vo, id, iq (dq frame of the PLL angle) and the
 * source frequency w [rad/s] (e.g. pll.w_out). Outputs: saturated pd, pq.
 * The output is computed with the current states, then the states are
 * advanced one step (forward Euler of (16c)-(16d)).
 */
static inline void Controller_run(ControllerParams *c, float vo, float id, float iq,
                                  float w, float *pd_out, float *pq_out)
{
    /* Reference ramp (the analysis assumes a constant vod) */
    if (c->vod_rate > 0.0f) {
        float dv = c->vod_target - c->vod, dmax = c->vod_rate * c->ts;
        c->vod += (dv > dmax) ? dmax : ((dv < -dmax) ? -dmax : dv);
    } else {
        c->vod = c->vod_target;
    }
    c->omega = w;

    /* Reference conductance (16c) */
    float psi = c->nu_bar * tanhf(c->nu / c->nu_bar);
    c->g_r = c->g_hat - c->kappa * psi;

    /* Current reference and feedforward, Eqs. (2) and (4) */
    c->I_ref   = Compute_I_eq(c, c->g_r);
    float pdeq =  2.0f * (c->Em - c->R * c->I_ref) / c->vod;
    float pqeq = -2.0f * c->L * c->omega * c->I_ref / c->vod;

    /* Errors (6) and passive output (9) */
    c->e1 = id - c->I_ref;
    c->e3 = vo - c->vod;
    c->s  = c->I_ref * c->e3 - c->vod * c->e1;

    /* Saturated port law (16a)-(16b) */
    *pd_out = sat(pdeq - (2.0f * c->ks / 3.0f) * c->s,          c->ad);
    *pq_out = sat(pqeq + (2.0f * c->kq / (3.0f * c->vod)) * iq, c->aq);

    /* State update: filter (16c) and projected estimator (16d) */
    float y = -2.0f * c->gamma * c->vod * c->e3
              - c->gamma * c->sigma * (c->g_hat - c->gc);
    c->g_hat += c->ts * Projection(c->g_hat, y, c->g_low, c->g_up, c->eps_p);
    c->nu    += c->ts * c->lambda_f * (c->e3 - c->nu);
}

#endif /* ROBUST_CONTROLLER_H */
