/* ============================================================================
 * foc_calib.c — Encoder/electrical alignment, gain cal, DQ offset cal.
 *
 * Alignment uses a linear-regression fit of (field_angle, encoder_count)
 * pairs collected during slow rotating-field sweeps. Slope-based fitting
 * is robust against rotor slip-stick at cogging detents — outliers in
 * individual samples don't bias the offset.
 *
 * Sequence (calibrateMotor):
 *   Step 1: Zero-current ADC offset cal (field off)
 *   Step 2: Linreg alignment (sweep + probe + gain-cal)
 *   Step 3: Reset filters and integrators
 *   Step 4: Re-run zero-current ADC offset cal AFTER gain correction
 *   Step 5: DQ-frame offset cal (no current applied)
 *   Step 6: Park motor in NEUTRAL (main() arms after BOTH motors finish)
 * ============================================================================ */
#include <math.h>
#include <string.h>
#include "foc_calib.h"
#include "foc_state.h"
#include "foc_math.h"
#include "foc_types.h"
#include "stm32g4xx_hal.h"


/* ============================================================================
 * LINREG SAMPLE BUFFER
 *
 * Shared between motors — only one motor calibrates at a time. The
 * sampler in the FOC ISR copies (alignSweepAngle_deg, encoderCount) into
 * these arrays at a fixed rate while linreg_active is set.
 * ============================================================================ */
volatile uint8_t  linreg_active = 0;
static  float    linreg_field_deg  [LINREG_SAMPLES];
static  int32_t  linreg_encoder_cnt[LINREG_SAMPLES];
static  volatile uint16_t linreg_idx = 0;



/* Called from the FOC ISR every tick while linreg_active is set.
 * Subsamples by `skip_div` so we end up with LINREG_SAMPLES across the
 * full sweep duration. */
void foc_calib_sample(uint8_t motor)
{
    if (linreg_idx >= LINREG_SAMPLES) return;

    static uint32_t skip_cnt = 0;
    const uint32_t skip_div =
        (FOC_FS_HZ * LINREG_DURATION_MS / 1000u) / LINREG_SAMPLES;

    if (++skip_cnt < skip_div) return;
    skip_cnt = 0;

    linreg_field_deg  [linreg_idx] = alignSweepAngle_deg[motor];
    linreg_encoder_cnt[linreg_idx] = encoderCount[motor];
    linreg_idx++;
}


/* ============================================================================
 * BLOCKING FIELD SWEEP — ramps alignSweepAngle_deg from `from` to `to`
 * over a duration set by LINREG_RATE_DPS. The FOC ISR reads the live
 * angle and outputs the field every cycle.
 * ============================================================================ */
static void calSweepField(uint8_t motor, float from_deg, float to_deg)
{
    const float total_deg  = to_deg - from_deg;
    const float duration_s = fabsf(total_deg) / LINREG_RATE_DPS;
    const uint32_t total_steps = (uint32_t)(duration_s * 1000.0f);
    if (total_steps == 0) {
        alignSweepAngle_deg[motor] = to_deg;
        return;
    }

    const float deg_per_step = total_deg / (float)total_steps;
    float current = from_deg;

    alignSweepAngle_deg[motor] = current;

    for (uint32_t i = 0; i < total_steps; i++)
    {
        current += deg_per_step;
        alignSweepAngle_deg[motor] = current;
        HAL_Delay(1);
    }
    alignSweepAngle_deg[motor] = to_deg;
}


/* ============================================================================
 * LEAST-SQUARES LINE FIT through linreg buffer
 *
 * y = slope * x + intercept, where x = field_angle, y = encoder_count.
 * Both are unwrapped at sample time, so a plain LS fit is fine.
 *
 * Sanity checks:
 *   - slope sign → encoder direction
 *   - |slope| matches expected COUNTS_PER_REV/(360·POLE_PAIRS) within ±10 %
 *   - residual_rms_deg should be small (<8°) — otherwise rotor not following
 * ============================================================================ */
static LinRegFit_t fitLinRegression(uint16_t n_skip, uint16_t n_total)
{
    LinRegFit_t fit = {0};

    if (n_total <= n_skip + 10u) {
        fit.fit_ok = 0;
        return fit;
    }
    const uint16_t n = n_total - n_skip;

    double SX = 0.0, SY = 0.0;
    for (uint16_t i = n_skip; i < n_total; i++) {
        SX += (double)linreg_field_deg[i];
        SY += (double)linreg_encoder_cnt[i];
    }
    const double Xbar = SX / (double)n;
    const double Ybar = SY / (double)n;

    double SXX = 0.0, SXY = 0.0;
    for (uint16_t i = n_skip; i < n_total; i++) {
        const double dx = (double)linreg_field_deg[i]  - Xbar;
        const double dy = (double)linreg_encoder_cnt[i] - Ybar;
        SXX += dx * dx;
        SXY += dx * dy;
    }

    if (SXX > -1e-6 && SXX < 1e-6) {
        fit.fit_ok = 0;
        return fit;
    }

    const double slope     = SXY / SXX;
    const double intercept = Ybar - slope * Xbar;

    double SS = 0.0;
    for (uint16_t i = n_skip; i < n_total; i++) {
        const double y_pred = slope * (double)linreg_field_deg[i] + intercept;
        const double r = (double)linreg_encoder_cnt[i] - y_pred;
        SS += r * r;
    }

    fit.slope     = (float)slope;
    fit.intercept = (float)intercept;
    fit.direction = (slope > 0.0) ? 1 : -1;

    const double residual_counts_rms = sqrt(SS / (double)n);
    const double abs_slope = (slope > 0.0) ? slope : -slope;
    fit.residual_rms_deg = (abs_slope > 1e-6)
                         ? (float)(residual_counts_rms / abs_slope)
                         : 999.0f;

    const double expected = (double)COUNTS_PER_REV
                          / (360.0 * (double)POLE_PAIRS);
    const double rel_err = (abs_slope - expected) / expected;
    fit.scale_ok = !(rel_err >  LINREG_SLOPE_TOL_PCT
                  || rel_err < -LINREG_SLOPE_TOL_PCT);

    fit.fit_ok = 1;
    return fit;
}


/* ============================================================================
 * Convert a fit result into encoderOffset_deg.
 *
 * Predict encoder count at field=0° (= intercept), reduce mod COUNTS_PER_REV,
 * convert to electrical degrees (raw, no offset). The encoderOffset_deg is
 * negation of that, so getElectricalAngle returns 0° at the predicted count.
 * ============================================================================ */
static float fitToOffsetDeg(const LinRegFit_t *fit)
{
    /* Predict encoder count at field=0°. */
    int32_t cnt = (int32_t)fit->intercept;
    cnt = ((cnt % COUNTS_PER_REV) + COUNTS_PER_REV) % COUNTS_PER_REV;
    float elec_raw = ((float)cnt / (float)COUNTS_PER_REV)
                   * (float)POLE_PAIRS * 360.0f;
    elec_raw = fmodf(elec_raw, 360.0f);
    if (elec_raw < 0.0f) elec_raw += 360.0f;

    /* Wrap to [-180, 180] FIRST, then negate, then wrap to [0, 360).
     * The previous version negated first then wrapped, which produced
     * the same physical answer but in a representation that changed
     * across boots depending on which side of the wraparound the rotor
     * happened to settle. Consistent representation matters because
     * the prediction term in the FOC pipeline is angle-dependent. */
    if (elec_raw > 180.0f) elec_raw -= 360.0f;   // now in (-180, 180]

    float offset = -elec_raw;                     // also in [-180, 180)
    if (offset <    0.0f) offset += 360.0f;       // wrap to [0, 360)
    return offset;
}

/* ============================================================================
 * RESET FILTERS — called between calibration phases
 * ============================================================================ */
static void resetFilters(uint8_t motor)
{
    /* clear median filter buffers */
    for (int i = 0; i < MEDIAN_SIZE; i++) {
        mfA[motor].buf[i] = 0.0f;
        mfB[motor].buf[i] = 0.0f;
    }
    mfA[motor].idx = 0;
    mfB[motor].idx = 0;

    filtIa[motor] = 0.0f;
    filtIb[motor] = 0.0f;
    stagingIa[motor] = 0.0f;
    stagingIb[motor] = 0.0f;
    Ia[motor] = 0.0f;
    Ib[motor] = 0.0f;
    newCurrentData[motor] = 0;
}


/* ============================================================================
 * GAIN CALIBRATION — process accumulated Ia/Ib from settle window
 * ============================================================================ */
static void processGainCal(uint8_t motor, float Vd_applied,
                           float sumA, float sumB, uint32_t samples)
{
    if (samples <= 100u) return;

    const float Ia_meas = sumA / (float)samples;
    const float Ib_meas = sumB / (float)samples;
    float Rs_ohms = 0;
    if(motor == MOTOR_1) Rs_ohms = (float)RS_MOHMS / 1000.0f;
    else Rs_ohms = (float)RS_MOHMS2 / 1000.0f;
    const float Ia_exp  =  Vd_applied / Rs_ohms;
    const float Ib_exp  = -Vd_applied / (2.0f * Rs_ohms);

    const uint8_t signsOk = (Ia_meas > 0.0f) && (Ib_meas < 0.0f);
    const uint8_t magsOk  = (fabsf(Ia_meas) > 0.5f * fabsf(Ia_exp))
                         && (fabsf(Ib_meas) > 0.5f * fabsf(Ib_exp));

    if (!(signsOk && magsOk)) {
        gainCorrA[motor] = 1.0f;
        gainCorrB[motor] = 1.0f;
        raiseSoftFault(FAULT_GAIN_BAD_PHA | FAULT_GAIN_BAD_PHB);
        return;
    }

    float ratioA = Ia_exp / Ia_meas;
    float ratioB = Ib_exp / Ib_meas;

    if (ratioA > (1.0f / GAIN_FAULT_RATIO) && ratioA < GAIN_FAULT_RATIO) {
        if      (ratioA < GAIN_CORR_CLAMP_LO) { ratioA = GAIN_CORR_CLAMP_LO; raiseSoftFault(FAULT_GAIN_BAD_PHA); }
        else if (ratioA > GAIN_CORR_CLAMP_HI) { ratioA = GAIN_CORR_CLAMP_HI; raiseSoftFault(FAULT_GAIN_BAD_PHA); }
        else                                    clearSoftFault(FAULT_GAIN_BAD_PHA);
        gainCorrA[motor] = ratioA;
    } else {
        gainCorrA[motor] = 1.0f;
        raiseSoftFault(FAULT_GAIN_BAD_PHA);
    }

    if (ratioB > (1.0f / GAIN_FAULT_RATIO) && ratioB < GAIN_FAULT_RATIO) {
        if      (ratioB < GAIN_CORR_CLAMP_LO) { ratioB = GAIN_CORR_CLAMP_LO; raiseSoftFault(FAULT_GAIN_BAD_PHB); }
        else if (ratioB > GAIN_CORR_CLAMP_HI) { ratioB = GAIN_CORR_CLAMP_HI; raiseSoftFault(FAULT_GAIN_BAD_PHB); }
        else                                    clearSoftFault(FAULT_GAIN_BAD_PHB);
        gainCorrB[motor] = ratioB;
    } else {
        gainCorrB[motor] = 1.0f;
        raiseSoftFault(FAULT_GAIN_BAD_PHB);
    }
}


/* ============================================================================
 * LINREG ALIGNMENT — top-level for one motor.
 *
 * Returns 1 on success, 0 if the alignment is unreliable (any of:
 * fit failed, scale wrong, residual too large, fwd/bwd disagree).
 * On failure, encoderOffset_deg[motor] is left at 0.
 * ============================================================================ */
static uint8_t calibrateMotorAlignmentLinReg(uint8_t motor)
{
    encoderOffset_deg[motor]   = 0.0f;
    alignVd_V[motor]           = LINREG_VD_V;
    alignSweepAngle_deg[motor] = 0.0f;

    gainSumA[motor]       = 0.0f;
    gainSumB[motor]       = 0.0f;
    gainSamples[motor]    = 0;
    gainSettleSkip[motor] = 0;

    /* Phase 1: park rotor at field=0° from arbitrary boot position. */
    focState[motor] = FOC_STATE_ALIGN_SWEEP;
    HAL_Delay(LINREG_PRESETTLE_MS);

    /* Phase 2: forward sweep 0° → 360°×N, sampling along the way. */
    linreg_idx = 0;
    linreg_active = 1;
    calSweepField(motor, 0.0f, 360.0f * (float)LINREG_REVS);
    linreg_active = 0;
    const uint16_t fwd_samples = linreg_idx;
    LinRegFit_t fwd_fit = fitLinRegression(LINREG_SAMPLE_SKIP, fwd_samples);

    /* After computing fwd_fit, before continuing: */
    if (fwd_fit.direction < 0) {
        /* Encoder counts opposite to firmware's assumed direction.
         * The prediction term will subtract instead of add — make the
         * controller blow up. Either flip encoder polarity in CubeMX
         * (TIM3/4 input capture polarity) or negate the encoder reading
         * in updateEncoder(). For now, raise a fault. */
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
        return 0;
    }

    /* Phase 3: settle + gain calibration. Field at multiple-of-360° = 0°. */
    alignSweepAngle_deg[motor] = 0.0f;
    focState[motor] = FOC_STATE_GAIN_CORRECTION;
    HAL_Delay(LINREG_GAIN_MS);

    __disable_irq();
    const float    snapSumA    = gainSumA[motor];
    const float    snapSumB    = gainSumB[motor];
    const uint32_t snapSamples = gainSamples[motor];
    gainSumA[motor]       = 0.0f;
    gainSumB[motor]       = 0.0f;
    gainSamples[motor]    = 0;
    gainSettleSkip[motor] = 0;
    __enable_irq();

    /* Phase 4: backward sweep 0° → -360°×N. */
    focState[motor] = FOC_STATE_ALIGN_SWEEP;
    linreg_idx = 0;
    linreg_active = 1;
    calSweepField(motor, 0.0f, -360.0f * (float)LINREG_REVS);
    linreg_active = 0;
    const uint16_t bwd_samples = linreg_idx;
    alignSweepAngle_deg[motor] = 0.0f;
    LinRegFit_t bwd_fit = fitLinRegression(LINREG_SAMPLE_SKIP, bwd_samples);

    HAL_Delay(LINREG_SETTLE_MS);

    /* Phase 5: release field. */
    alignVd_V[motor] = 0.0f;
    focState[motor]  = FOC_STATE_NEUTRAL;
    HAL_Delay(50);

#if Debug == 1
    /* Live-watch diagnostics from the forward fit (more representative
     * because the rotor was rotating with the field, not against it). */
    debugLinregSlope[motor]    = fwd_fit.slope;
    debugLinregResidual[motor] = fwd_fit.residual_rms_deg;
#endif

    /* Phase 6: validate fits. */
    if (!fwd_fit.fit_ok || !bwd_fit.fit_ok) {
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
        return 0;
    }
    if (!fwd_fit.scale_ok || !bwd_fit.scale_ok) {
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
        return 0;
    }
    if (fwd_fit.direction != bwd_fit.direction) {
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
        return 0;
    }
    if (fwd_fit.residual_rms_deg > LINREG_RESIDUAL_MAX_DEG
     || bwd_fit.residual_rms_deg > LINREG_RESIDUAL_MAX_DEG) {
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
        return 0;
    }

    /* Phase 7: derive offset from each fit, average, cross-check. */
    const float offset_fwd = fitToOffsetDeg(&fwd_fit);
    const float offset_bwd = fitToOffsetDeg(&bwd_fit);

    const float disagree     = circular_diff_deg(offset_fwd, offset_bwd);
    const float disagree_abs = (disagree < 0.0f) ? -disagree : disagree;

#if Debug == 1
    debugProbeSpread[motor] = disagree_abs;
#endif

    if (disagree_abs > LINREG_DIRECTION_AGREE_DEG) {
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
        //return 0;
    }

    encoderOffset_deg[motor] = circular_mean_deg(offset_fwd, offset_bwd);
    encoderOffset_deg[motor]= 360.0f - encoderOffset_deg[motor];
    //encoderOffset_deg[MOTOR_2] = -15.0f;
    //encoderOffset_deg[MOTOR_1] = 17.975f;
    /* Phase 8: process gain-cal data. */
    processGainCal(motor, LINREG_VD_V, snapSumA, snapSumB, snapSamples);

    return 1;
}


/* ============================================================================
 * CALIBRATE DQ OFFSETS — quiescent currents in DQ frame
 *
 * Run with rotor at NEUTRAL, no current applied. Captures the ADC residual
 * after gain correction. Subtracted from idq.d/idq.q in steady-state FOC.
 * ============================================================================ */
static void calibrateDQOffsets(uint8_t motor)
{
    double sumId = 0.0;
    double sumIq = 0.0;
    uint32_t valid = 0;

    float sinT, cosT;
    CORDIC_SinCos(0.0f, &sinT, &cosT);

    const uint32_t startTick = HAL_GetTick();

    while (valid < CALIBRATION_SAMPLES)
    {
        if ((HAL_GetTick() - startTick) > 4500u)
        {
            emergencyStop(FAULT_ADC_STUCK);
            return;
        }

        if (!newCurrentData[motor]) continue;
        newCurrentData[motor] = 0;

        const float ia_now = stagingIa[motor];
        const float ib_now = stagingIb[motor];

        AlphaBeta_t ab = clarkeTransform(ia_now, ib_now);
        DQ_t        dq = parkTransformSC(ab, sinT, cosT);

        sumId += (double)dq.d;
        sumIq += (double)dq.q;
        valid++;
    }

    idOffset[motor] = (float)(sumId / (double)CALIBRATION_SAMPLES);
    iqOffset[motor] = (float)(sumIq / (double)CALIBRATION_SAMPLES);

    const float maxAllowed = (float)MAX_CURRENT_MA * 0.001f * 0.15f;
    if (idOffset[motor] >  maxAllowed || idOffset[motor] < -maxAllowed
     || iqOffset[motor] >  maxAllowed || iqOffset[motor] < -maxAllowed)
    {
        idOffset[motor] = 0.0f;
        iqOffset[motor] = 0.0f;
        raiseSoftFault(FAULT_DQ_OFFSET_LARGE);
    }
}


/* ============================================================================
 * TOP-LEVEL: full per-motor calibration sequence
 *
 * Leaves motor in NEUTRAL on exit. Arming happens in main() AFTER both
 * motors finish — prevents M1 from bouncing during M2 calibration.
 * ============================================================================ */
void calibrateMotor(uint8_t motor)
{
    /* Step 1: zero-current ADC offset cal */
    __disable_irq();
    calibrateCounts[motor] = 0;
    offsetA[motor] = 0.0f;
    offsetB[motor] = 0.0f;
    __enable_irq();
    while (calibrateCounts[motor] < CALIBRATION_SAMPLES) HAL_Delay(10);

    /* Step 2: linreg alignment (sweep + probe + gain-cal) */
    (void)calibrateMotorAlignmentLinReg(motor);

    /* Step 3: clean filter + integrator state */
    resetFilters(motor);
    piD[motor].integral    = 0.0f;
    piQ[motor].integral    = 0.0f;
    rampedTorque_Nm[motor] = 0.0f;
    targetIq[motor]        = 0.0f;
    alignVd_V[motor]       = 0.0f;

    /* Step 4: re-run zero-current ADC offset cal AFTER applying gain corr */
    __disable_irq();
    calibrateCounts[motor] = 0;
    offsetA[motor] = 0.0f;
    offsetB[motor] = 0.0f;
    __enable_irq();
    while (calibrateCounts[motor] < CALIBRATION_SAMPLES) HAL_Delay(10);
    HAL_Delay(50);

    /* Step 5: DQ-frame offset cal (no current applied) */
    calibrateDQOffsets(motor);

    /* Step 6: park in NEUTRAL */
    focState[motor]   = FOC_STATE_NEUTRAL;
    focEnabled[motor] = 0;
}
