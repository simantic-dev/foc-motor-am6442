/*
 * Field-oriented control of a PMSM on the AM6442 R5FSS0 core 0 (TI MCU+ SDK, NORTOS).
 *
 * ePWM0/1/2 drive phases A/B/C: 20 kHz up-down count from the 250 MHz FICLK, high side on
 * while TBCNT < CMPA. ePWM0 SOCA at TBCNT = PRD (centre of the low-side window) is routed by
 * CTRL_MMR (SOCA_SEL, ADC0_CTRL.TRIG_SEL) to ADC0, which converts two hardware-synchronised
 * steps: AIN0 (phase A shunt) and AIN1 (phase B). ADC0 END_OF_SEQUENCE runs the current loop.
 * eQEP0 counts a 500-line encoder (2000 counts per turn).
 *
 * The ISR: Clarke, Park, two PI current loops, inverse Park, min-max SVPWM. Every 20th
 * pass (1 kHz) it runs the speed PI. The main loop prints telemetry and checks the result.
 */
#include <math.h>
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include <drivers/eqep.h>
#include <drivers/soc.h>
#include <drivers/hw_include/cslr.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define PI_F        3.14159265f
#define FICLK_HZ    250000000u                      /* ePWM functional clock, MAIN_SYSCLK0 / 2 */
#define PWM_HZ      10000u
#define TBPRD       (FICLK_HZ / (2u * PWM_HZ))      /* up-down */
#define TS          (1.0f / PWM_HZ)
#define SPEED_DIV   20u                             /* speed loop at 1 kHz */
#define VBUS        24.0f
#define POLE_PAIRS  7
#define COUNTS      2000                            /* encoder counts per mechanical turn */
#define AMP_PER_LSB (1.8f / 4096.0f / 0.05f)        /* 1.8 V ADC, 5 mOhm shunt x10 amplifier: 0.05 V/A */
#define IQ_MAX      5.0f
#define RPM(w)      ((w) * 60.0f / (2.0f * PI_F))
#define TICKS(ms)   ((ms) * (PWM_HZ / 1000u))       /* ISR passes in ms */

static const uint32_t PWM[3] = { CSL_EPWM0_EPWM_BASE, CSL_EPWM1_EPWM_BASE, CSL_EPWM2_EPWM_BASE };
#define ADC   CSL_ADC0_BASE
#define QEP   CSL_EQEP0_REG_BASE

/* CTRL_MMR0 (TRM SPRUIM2J 5.1.2): ePWM time-base clock enables and the ADC trigger mux. */
#define CTRL_MMR_EPWM_TB_CLKEN  (CSL_CTRL_MMR0_CFG0_BASE + 0x4130u)
#define CTRL_MMR_SOCA_SEL       (CSL_CTRL_MMR0_CFG0_BASE + 0x4170u)
#define CTRL_MMR_ADC0_CTRL      (CSL_CTRL_MMR0_CFG0_BASE + 0x4710u)
/* ePWM event trigger SOCA (TRM 12.4.3.4.8.4; the v0 driver has no SOC API). */
#define EPWM_ETSEL  0x32u
#define EPWM_ETPS   0x34u

typedef struct { float kp, ki, i, lim; } Pi;

/* PI with conditional integration: the integrator holds while the output is saturated,
 * so a current-limited speed step does not wind it up and overshoot. */
static float pi_run(Pi *c, float err)
{
    float u = c->kp * err + c->i + c->ki * err;
    if (u > c->lim) return c->lim;
    if (u < -c->lim) return -c->lim;
    c->i += c->ki * err;
    return u;
}

/* Current loop at 1.5 kHz bandwidth for R = 0.2 Ohm, L = 100 uH; speed loop at 15 Hz for
 * J = 1e-4 kg m^2, Kt = 1.5 * 7 * 0.004 = 0.042 Nm/A. */
static Pi gId = { 0.94f, 1885.0f * TS, 0, VBUS / 1.7320508f };
static Pi gIq = { 0.94f, 1885.0f * TS, 0, VBUS / 1.7320508f };
static Pi gW  = { 0.224f, 5.3f / 1000.0f, 0, IQ_MAX };

static volatile uint32_t gTicks, gIsrCycles, gIsrMax;
static volatile float gRpmRef, gRpm, gIq_meas, gIdRef;
static volatile int gMode;                     /* 0 offset cal, 1 align, 2 closed loop */
static float gOffA, gOffB;
static int32_t gZero, gHist[5];
static int64_t gAlignSum;
static uint32_t gAlignN;
static HwiP_Object gHwi;

static void set_duty(const float d[3])
{
    for (int k = 0; k < 3; k++) {
        float x = d[k] < 0.02f ? 0.02f : (d[k] > 0.98f ? 0.98f : d[k]);
        EPWM_counterComparatorCfg(PWM[k], EPWM_CC_CMP_A, (uint32_t)(x * TBPRD), EPWM_SHADOW_REG_CTRL_ENABLE,
                                  EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
    }
}

static void foc_isr(void *arg)
{
    uint32_t t0 = CycleCounterP_getCount32();
    (void)arg;
    float r[2] = { 0, 0 };
    for (uint32_t n = ADCGetFIFOWordCount(ADC, ADC_FIFO_NUM_0); n > 0; n--) {
        uint32_t w = ADCGetFIFOData(ADC, ADC_FIFO_NUM_0);
        uint32_t step = (w & ADC_FIFODATA_ADCCHNLID_MASK) >> ADC_FIFODATA_ADCCHNLID_SHIFT;
        if (step < 2) r[step] = (float)((w & ADC_FIFODATA_ADCDATA_MASK) >> ADC_FIFODATA_ADCDATA_SHIFT);
    }
    ADCClearIntrStatus(ADC, ADCGetIntrStatus(ADC));
    ADCWriteEOI(ADC);
    float ra = r[0], rb = r[1];
    uint32_t tick = ++gTicks;
    int32_t pos = (int32_t)EQEP_getPosition(QEP);

    if (gMode == 0) {                      /* 50% duty, zero current: learn the amplifier offsets */
        gOffA += ra; gOffB += rb;
        if (tick == TICKS(50)) { gOffA /= TICKS(50); gOffB /= TICKS(50); gMode = 1; }
        return;
    }
    float ia = (ra - gOffA) * AMP_PER_LSB, ib = (rb - gOffB) * AMP_PER_LSB;
    float ial = ia, ibe = (ia + 2.0f * ib) / 1.7320508f;          /* Clarke */

    float th = 0.0f;
    if (gMode == 2) {
        int32_t m = (pos - gZero) % COUNTS;
        th = (float)m * (2.0f * PI_F * POLE_PAIRS / COUNTS);
    }
    float c = cosf(th), s = sinf(th);
    float id = ial * c + ibe * s, iq = -ial * s + ibe * c;          /* Park */

    static float iqRef;
    if (gMode == 1) {                       /* hold 3 A on the d axis; zero = mean position */
        if (tick > TICKS(250)) { gAlignSum += pos; gAlignN++; }
        if (tick == TICKS(400)) {
            gZero = (int32_t)(gAlignSum / (int64_t)gAlignN);
            for (int k = 0; k < 5; k++) gHist[k] = pos;
            gIdRef = 0.0f; gId.i = 0.0f;
            gMode = 2;
        }
    } else if (tick % SPEED_DIV == 0) {     /* 1 kHz speed loop, speed over a 5 ms window */
        uint32_t k = (tick / SPEED_DIV) % 5;
        float w = (float)(pos - gHist[k]) * (2.0f * PI_F / COUNTS) / 0.005f;
        gHist[k] = pos;
        gRpm = RPM(w);
        iqRef = pi_run(&gW, (gRpmRef - gRpm) * (2.0f * PI_F / 60.0f));
        gIq_meas = iq;
    }
    float vd = pi_run(&gId, gIdRef - id), vq = pi_run(&gIq, iqRef - iq);

    float va = vd * c - vq * s, vb = vd * s + vq * c;               /* inverse Park */
    float v[3] = { va, -0.5f * va + 0.8660254f * vb, -0.5f * va - 0.8660254f * vb };
    float mx = fmaxf(v[0], fmaxf(v[1], v[2])), mn = fminf(v[0], fminf(v[1], v[2]));
    float off = -(mx + mn) * 0.5f, d[3];
    for (int k = 0; k < 3; k++) d[k] = 0.5f + (v[k] + off) / VBUS;
    set_duty(d);

    uint32_t dt = CycleCounterP_getCount32() - t0;
    gIsrCycles = dt;
    if (dt > gIsrMax && tick > TICKS(50)) gIsrMax = dt;
}

static void pwm_init(void)
{
    CSL_REG32_WR(CTRL_MMR_EPWM_TB_CLKEN, 0);       /* hold all time bases */
    for (int k = 0; k < 3; k++) {
        uint32_t b = PWM[k];
        EPWM_tbTimebaseClkCfg(b, FICLK_HZ, FICLK_HZ);
        EPWM_tbPwmFreqCfg(b, FICLK_HZ, PWM_HZ, EPWM_TB_COUNTER_DIR_UP_DOWN, EPWM_SHADOW_REG_CTRL_ENABLE);
        EPWM_tbSyncDisable(b);
        EPWM_tbSetSyncOutMode(b, EPWM_TB_SYNC_OUT_EVT_DISABLE);
        EPWM_tbSetEmulationMode(b, EPWM_TB_EMU_MODE_FREE_RUN);
        EPWM_tbWriteTbCount(b, 0);
        EPWM_counterComparatorCfg(b, EPWM_CC_CMP_A, TBPRD / 2, EPWM_SHADOW_REG_CTRL_ENABLE,
                                  EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
        EPWM_AqActionCfg aq = {
            .zeroAction = EPWM_AQ_ACTION_DONOTHING, .prdAction = EPWM_AQ_ACTION_DONOTHING,
            .cmpAUpAction = EPWM_AQ_ACTION_LOW, .cmpADownAction = EPWM_AQ_ACTION_HIGH,
            .cmpBUpAction = EPWM_AQ_ACTION_DONOTHING, .cmpBDownAction = EPWM_AQ_ACTION_DONOTHING,
        };
        EPWM_aqActionOnOutputCfg(b, EPWM_OUTPUT_CH_A, &aq);
        EPWM_deadbandBypass(b);
    }
    CSL_REG16_WR(PWM[0] + EPWM_ETSEL, 1u << 11 | 2u << 8);    /* SOCAEN, SOCASEL = CTR == PRD */
    CSL_REG16_WR(PWM[0] + EPWM_ETPS, 1u << 8);                 /* SOCAPRD: every event */
    CSL_REG32_WR(CTRL_MMR_SOCA_SEL, 0);                        /* SOCA = the ePWM SOCA OR */
    CSL_REG32_WR(CTRL_MMR_ADC0_CTRL, 2);                       /* TRIG_SEL: ePWM SOCA */
}

static void adc_init(void)
{
    ADCClearIntrStatus(ADC, ADC_INTR_STATUS_ALL);
    ADCPowerUp(ADC, TRUE);
    for (volatile int i = 0; i < 4000; i++) {}                 /* >= 4 us AFE power-up */
    ADCInit(ADC, FALSE, 0U, 0U);
    adcStepConfig_t cfg = {
        .mode = ADC_STEPCONFIG_MODE_HW_SYNC_CONTINUOUS, .openDelay = 0, .sampleDelay = 0,
        .rangeCheckEnable = 0, .averaging = ADC_AVERAGING_NONE, .fifoNum = ADC_FIFO_NUM_0,
    };
    for (uint32_t k = 0; k < 2; k++) {
        cfg.channel = ADC_CHANNEL_1 + k;
        DebugP_assert(ADCSetStepParams(ADC, ADC_STEP_1 + k, &cfg) == SystemP_SUCCESS);
        ADCStepEnable(ADC, ADC_STEP_1 + k, TRUE);
    }
    ADCStepIdTagEnable(ADC, TRUE);
    ADCEnableIntr(ADC, ADC_INTR_SRC_END_OF_SEQUENCE);
    /* CONTROL.HW_EVENT_MAPPING (TRM 14.x ADC CTRL bit 8): the trigger runs the HW steps. */
    CSL_REG32_WR(ADC + ADC_CTRL, CSL_REG32_RD(ADC + ADC_CTRL) | 1u << 8);
    ADCStart(ADC, TRUE);
}

static void qep_init(void)
{
    EQEP_setDecoderConfig(QEP, EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_NO_SWAP);
    EQEP_setEmulationMode(QEP, EQEP_EMULATIONMODE_RUNFREE);
    EQEP_setPositionCounterConfig(QEP, EQEP_POSITION_RESET_MAX_POS, 0xFFFFFFFFu);
    EQEP_enableModule(QEP);
}

/* Speed profile (ms since closed loop): 0 rpm, then 1000, then 3000. */
static float profile(uint32_t ms) { return ms < 50 ? 0.0f : (ms < 300 ? 1000.0f : 3000.0f); }

void foc_main(void)
{
    Drivers_open();
    Board_driversOpen();
    DebugP_log("== foc-motor-am6442: FOC of a PMSM on the AM6442 R5F ==\r\n");
    DebugP_log("pwm %u Hz centre-aligned, TBPRD %u, current loop %u Hz, speed loop %u Hz\r\n",
               PWM_HZ, TBPRD, PWM_HZ, PWM_HZ / SPEED_DIV);

    CycleCounterP_reset();
    pwm_init();
    adc_init();
    qep_init();

    HwiP_Params p;
    HwiP_Params_init(&p);
    p.intNum = CONFIG_ADC0_INTR;
    p.priority = 0;
    p.callback = foc_isr;
    DebugP_assert(HwiP_construct(&gHwi, &p) == SystemP_SUCCESS);
    CSL_REG32_WR(CTRL_MMR_EPWM_TB_CLKEN, 0x7);    /* start A, B, C together */

    while (gMode == 0) {}
    DebugP_log("offsets  a=%d b=%d (LSB)\r\n", (int)gOffA, (int)gOffB);
    gIdRef = 3.0f;
    while (gMode == 1) {}
    DebugP_log("aligned  encoder zero = %d counts\r\n", (int)gZero);

    uint32_t start = gTicks, last = 0, ok1 = 0, ok3 = 0, n1 = 0, n3 = 0;
    for (;;) {
        uint32_t ms = (gTicks - start) / (PWM_HZ / 1000);
        gRpmRef = profile(ms);
        if (ms >= last + 20) {
            last = ms - ms % 20;
            float rpm = gRpm, ref = gRpmRef;
            DebugP_log("t=%4u ms ref=%5d rpm speed=%5d rpm iq=%5d mA isr=%u cycles\r\n",
                       ms, (int)ref, (int)rpm, (int)(gIq_meas * 1000), gIsrCycles);
            if (ms >= 200 && ms < 300) { n1++; ok1 += fabsf(rpm - 1000) < 20; }
            if (ms >= 500 && ms < 600) { n3++; ok3 += fabsf(rpm - 3000) < 60; }
            if (ms >= 1100) break;
        }
    }
    uint32_t mhz = (uint32_t)(SOC_getSelfCpuClk() / 1000000u);
    DebugP_log("isr worst %u cycles = %u ns at %u MHz (budget %u ns)\r\n",
               gIsrMax, gIsrMax * 1000u / mhz, mhz, 1000000000u / PWM_HZ);
    DebugP_log("hold 1000 rpm: %u/%u, hold 3000 rpm: %u/%u\r\n", ok1, n1, ok3, n3);
    DebugP_log("RESULT: %s\r\n", (ok1 == n1 && ok3 == n3 && n1 && n3) ? "PASS" : "FAIL");
    DebugP_log("DONE\r\n");
    for (int k = 0; k < 3; k++)                    /* TBCTL.CTRMODE = stop-freeze: bridge off */
        CSL_REG16_WR(PWM[k], CSL_REG16_RD(PWM[k]) | 3u);
    for (;;) {}
}
