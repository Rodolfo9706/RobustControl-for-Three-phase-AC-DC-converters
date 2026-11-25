/*
 * LAUNCHXL-F28379D implementation of the controller (16), TCST 26-1303.
 * ISR at 10 kHz, synchronized with ePWM1:
 *   ADC -> Clarke -> SRF-PLL -> Park -> controller (16) -> inverse Park -> SVPWM
 *
 * NOTE: ADC, ePWM (10 kHz, 2.5 us dead time) and ADC-SOC configuration are
 * not included here; keep the configuration used in the experiments
 * (initADC/initEPWM or SysConfig) and call it where indicated.
 */
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "transforms_and_pll.h"
#include "robust_controller.h"

#define TS          1.0e-4f     /* 10 kHz                                   */
#define F_SOURCE    446.0f      /* nominal source frequency [Hz] (measure)  */
#define USE_CERTIFIED_GAINS 0   /* 1: gains satisfying (17)-(18), 0: Table V */

TransformVars    g_tf;
SrfPLL           g_pll;
ControllerParams g_ctrl;

/* Current-sensor conditioning (ACS712 + analog stage): CALIBRATE.
 * Offsets/gain must refer to the voltage seen at the ADC pin (0-3 V),
 * i.e. after the conditioning stage, not to the 5 V sensor output.    */
float g_offset_ia = 1.5f, g_offset_ib = 1.5f;   /* [V] at ADC pin      */
float g_gain_i    = 10.0f;                      /* [A/V] at ADC pin    */
float g_gain_vo   = 10.0f;                      /* divider ratio       */

__interrupt void epwm1ISR(void);

/* SVPWM by min-max zero-sequence injection. p_alpha, p_beta are the
 * modulation indices of model (1): leg duty d_j = (p_j + 1)/2, linear
 * range |p| <= 2/sqrt(3), which contains the rectangle S of (5).      */
static void Update_SVPWM(float palpha, float pbeta)
{
    float va = palpha;
    float vb = -0.5f * palpha + 0.86602540378f * pbeta;
    float vc = -0.5f * palpha - 0.86602540378f * pbeta;

    float vmax = fmaxf(va, fmaxf(vb, vc));
    float vmin = fminf(va, fminf(vb, vc));
    float vz   = -0.5f * (vmax + vmin);

    float da = fminf(fmaxf(0.5f * (va + vz + 1.0f), 0.0f), 1.0f);
    float db = fminf(fmaxf(0.5f * (vb + vz + 1.0f), 0.0f), 1.0f);
    float dc = fminf(fmaxf(0.5f * (vc + vz + 1.0f), 0.0f), 1.0f);

    uint16_t tbprd = EPWM_getTimeBasePeriod(EPWM1_BASE);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(da * tbprd));
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(db * tbprd));
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(dc * tbprd));
}

void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    /* TODO: initADC(); initEPWM(); initADCSOC();  (experimental configuration) */

    /* Plant parameters, Table IV */
    g_ctrl.Em = 12.0f;          /* CHECK: fundamental peak phase voltage   */
    g_ctrl.R  = 0.1f;
    g_ctrl.L  = 2.0e-3f;
    g_ctrl.omega = 2.0f * (float)M_PI * F_SOURCE;

    /* Reference: 24 V, ramps to vod_target at vod_rate */
    g_ctrl.vod = 24.0f;  g_ctrl.vod_target = 24.0f;  g_ctrl.vod_rate = 1.1f;

    /* Gains, Table V */
    g_ctrl.ks = 0.1f;   g_ctrl.kq = 100.0f;
    g_ctrl.sigma = 0.01f;  g_ctrl.gc = 1.0f / 45.0f;
    g_ctrl.ad = 1.013f;  g_ctrl.aq = 0.55f;
    g_ctrl.g_low = 0.01f;  g_ctrl.g_up = 0.025f;  g_ctrl.eps_p = 1.0e-3f;
#if USE_CERTIFIED_GAINS
    g_ctrl.gamma = 6.0e-4f;  g_ctrl.kappa = 2.0e-3f;
    g_ctrl.lambda_f = 200.0f;  g_ctrl.nu_bar = 0.5f;
#else
    g_ctrl.gamma = 2.5e-3f;  g_ctrl.kappa = 8.0e-3f;
    g_ctrl.lambda_f = 1500.0f;  g_ctrl.nu_bar = 1.0f;
#endif

    /* Initial states: g_hat(0) in G_eps */
    g_ctrl.g_hat = g_ctrl.gc;
    g_ctrl.nu = 0.0f;
    g_ctrl.ts = TS;

    PLL_init(&g_pll, 10.0f, 200.0f, F_SOURCE, TS);

    Interrupt_register(INT_EPWM1, &epwm1ISR);
    Interrupt_enable(INT_EPWM1);
    EINT;
    ERTM;

    for (;;) {
        /* background: set g_ctrl.vod_target here for reference changes */
    }
}

__interrupt void epwm1ISR(void)
{
    /* 1. Measurements */
    const float k_adc = 3.0f / 4095.0f;
    float adc_ia = (float)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) * k_adc;
    float adc_ib = (float)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0) * k_adc;
    float adc_vo = (float)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0) * k_adc;

    g_tf.ia = (adc_ia - g_offset_ia) * g_gain_i;
    g_tf.ib = (adc_ib - g_offset_ib) * g_gain_i;
    g_tf.ic = -g_tf.ia - g_tf.ib;                 /* three-wire system */
    float vo = adc_vo * g_gain_vo;

    /* 2. Clarke, PLL, Park. Synchronized with the currents as in the
     *    experiments; pass (valpha, vbeta) instead if voltage sensors are
     *    added, so that the d axis is aligned with the source voltage.  */
    Clarke_Transform(&g_tf);
    PLL_run(&g_pll, &g_tf, g_tf.ialpha, g_tf.ibeta, 0.05f);
    Park_Transform(&g_tf);

    /* 3. Controller (16), with the frequency estimated by the PLL */
    Controller_run(&g_ctrl, vo, g_tf.id, g_tf.iq, g_pll.w_out, &g_tf.pd, &g_tf.pq);

    /* 4. Inverse Park and SVPWM */
    Inv_Park_Transform(&g_tf);
    Update_SVPWM(g_tf.palpha, g_tf.pbeta);

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}
