#ifndef TRANSFORMS_AND_PLL_H
#define TRANSFORMS_AND_PLL_H

/*
 * Clarke/Park transforms and SRF-PLL used with the controller (16) of
 * TCST 26-1303. Amplitude-invariant transforms, d-axis aligned with the
 * synchronizing vector (coefficients 1/2 and 3/4 of model (1)).
 */

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI (2.0f * (float)M_PI)

/* Signals of the transform chain */
typedef struct {
    float ia, ib, ic;        /* measured phase currents [A]            */
    float ialpha, ibeta;     /* stationary frame                        */
    float id, iq;            /* rotating frame                          */
    float pd, pq;            /* modulation indices from the controller  */
    float palpha, pbeta;     /* modulation indices, stationary frame    */
    float sin_th, cos_th;    /* sine and cosine of the PLL angle        */
} TransformVars;

/* Synchronous-reference-frame PLL */
typedef struct {
    float kp, ki;            /* PI loop-filter gains                    */
    float integrator;        /* PI integrator state [rad/s]             */
    float phase;             /* estimated angle theta in [0, 2*pi)      */
    float w_base;            /* nominal source frequency [rad/s]        */
    float w_min, w_max;      /* frequency clamp [rad/s]                 */
    float w_out;             /* estimated frequency [rad/s]             */
    float ts;                /* sampling period [s]                     */
} SrfPLL;

/* f_base: nominal source frequency in Hz (measured, e.g. ~446 Hz for the
 * BLDC generator, NOT 60 Hz). ts: sampling period (1/10 kHz = 1e-4 s).   */
static inline void PLL_init(SrfPLL *pll, float kp, float ki, float f_base, float ts)
{
    pll->kp = kp;
    pll->ki = ki;
    pll->integrator = 0.0f;
    pll->phase = 0.0f;
    pll->w_base = TWO_PI * f_base;
    pll->w_min = 0.5f * pll->w_base;
    pll->w_max = 1.5f * pll->w_base;
    pll->w_out = pll->w_base;
    pll->ts = ts;
}

/* Clarke, amplitude invariant, three-wire system (ia + ib + ic = 0) */
static inline void Clarke_Transform(TransformVars *v)
{
    v->ialpha = v->ia;
    v->ibeta  = (v->ia + 2.0f * v->ib) * 0.57735026919f;   /* 1/sqrt(3) */
}

/* Park: alpha/beta -> d/q, d-axis at angle theta */
static inline void Park_Transform(TransformVars *v)
{
    v->id =  v->ialpha * v->cos_th + v->ibeta * v->sin_th;
    v->iq = -v->ialpha * v->sin_th + v->ibeta * v->cos_th;  /* fixed: cos_th */
}

/* Inverse Park: d/q -> alpha/beta */
static inline void Inv_Park_Transform(TransformVars *v)
{
    v->palpha = v->pd * v->cos_th - v->pq * v->sin_th;
    v->pbeta  = v->pd * v->sin_th + v->pq * v->cos_th;
}

/*
 * SRF-PLL. (xalpha, xbeta) is the synchronizing vector:
 *   - phase voltages (valpha, vbeta): d-axis aligned with the source
 *     voltage, as assumed by model (1)  -> recommended;
 *   - phase currents (ialpha, ibeta): d-axis aligned with the current,
 *     which makes iq = 0 hold by construction (see Reviewer 6, comment 4).
 * The vector is normalized so that the loop gain does not depend on its
 * amplitude; below min_mag the error is set to zero (free-running).
 */
static inline void PLL_run(SrfPLL *pll, TransformVars *v,
                           float xalpha, float xbeta, float min_mag)
{
    float mag = sqrtf(xalpha * xalpha + xbeta * xbeta);
    float err = 0.0f;

    if (mag > min_mag) {
        /* q component of the normalized vector = sin(theta_x - theta) */
        err = (-xalpha * v->sin_th + xbeta * v->cos_th) / mag;
    }

    /* PI loop filter with frequency clamp (anti-windup) */
    pll->integrator += pll->ki * err * pll->ts;
    pll->w_out = pll->w_base + pll->kp * err + pll->integrator;
    if (pll->w_out > pll->w_max) {
        pll->w_out = pll->w_max;
        pll->integrator -= pll->ki * err * pll->ts;
    } else if (pll->w_out < pll->w_min) {
        pll->w_out = pll->w_min;
        pll->integrator -= pll->ki * err * pll->ts;
    }

    /* Angle integration, wrapped to [0, 2*pi) */
    pll->phase += pll->w_out * pll->ts;
    if (pll->phase >= TWO_PI) pll->phase -= TWO_PI;
    if (pll->phase < 0.0f)    pll->phase += TWO_PI;

    v->sin_th = sinf(pll->phase);
    v->cos_th = cosf(pll->phase);
}

#endif /* TRANSFORMS_AND_PLL_H */
