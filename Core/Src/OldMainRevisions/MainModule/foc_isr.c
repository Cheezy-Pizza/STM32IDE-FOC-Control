/* ============================================================================
 * foc_isr.c — Real-time ISR pipeline for FOC controller.
 *   - HAL_TIM_PeriodElapsedCallback dispatches TIM8/TIM20 → FOCroutine.
 *   - HAL_ADCEx_InjectedConvCpltCallback dispatches ADC2/ADC3 → ADCloop.
 *   - HAL_TIM_PeriodElapsedCallback also handles TIM7 (UART poll) and TIM17
 *     (HAL tick) as a single dispatch point.
 *
 * NVIC priority assumption: ADC1_2_IRQn = ADC3_IRQn = TIM8_UP_IRQn =
 * TIM20_UP_IRQn = priority 2. Same priority means ISRs cannot preempt
 * each other; they serialize cleanly within the 100 µs PWM window.
 * ============================================================================ */
#include <math.h>
#include "foc_isr.h"
#include "foc_state.h"
#include "foc_math.h"
#include "foc_calib.h"
#include "uart_proto.h"
#include "tim.h"
#include "adc.h"


/* Park 50 % duty on all 3 phases. */
void applyNeutralOutput(uint8_t motor)
{
    const uint32_t neutral = (uint32_t)(PWM_ARR / 2u);
    TIM_HandleTypeDef *htim = (motor == MOTOR_1) ? &htim8 : &htim20;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, neutral);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, neutral);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, neutral);
}


/* Local: read encoder counter for one motor via direct register access. */
static inline int32_t readEncoderCnt(uint8_t motor)
{
    return (motor == MOTOR_1) ? (int32_t)htim3.Instance->CNT
                              : (int32_t)htim4.Instance->CNT;
}


/* Local: shared encoder accumulator update. Used by both the FOC pipeline
 * and the alignment-sweep handlers. */
static inline void updateEncoder(uint8_t motor)
{
    const int32_t currentCount = readEncoderCnt(motor);
    int32_t delta = currentCount - lastCount[motor];
    if (delta >  32767) delta -= 65536;
    if (delta < -32768) delta += 65536;
    encoderCount[motor] += delta;
    lastCount[motor]     = currentCount;
}


#if dataAqu == 1
static uint32_t aquGi = 0;
static uint16_t aquPi = 0;
#endif


/* ============================================================================
 * FOC ROUTINE — runs at 10 kHz from TIM8 (M1) or TIM20 (M2) update event
 * ============================================================================ */
void FOCroutine(uint8_t motor)
{
    if (focState[motor] == FOC_STATE_FAULT)
    {
        applyNeutralOutput(motor);
        return;
    }

    if (focState[motor] == FOC_STATE_NEUTRAL)
    {
        applyNeutralOutput(motor);
        return;
    }

    /* ── ROTATING-FIELD HANDLER ───────────────────────────────────────
     *
     * Used during calibration. Field angle is set from main context via
     * calSweepField; this handler reads it, applies the field, tracks the
     * encoder, and (if sampling is active) feeds the linreg buffer. */
    if (focState[motor] == FOC_STATE_ALIGN_SWEEP)
    {
        float sinA, cosA;
        CORDIC_SinCos(alignSweepAngle_deg[motor], &sinA, &cosA);

        const AlphaBeta_t vAlpha = {
            .alpha = alignVd_V[motor] * cosA,
            .beta  = alignVd_V[motor] * sinA
        };
        SVPWM(vAlpha, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);

        updateEncoder(motor);
        foc_calib_maybe_sample(motor);
        return;
    }

    /* ── HOLD-AT-ZERO HANDLER (legacy ALIGN + gain-cal) ──────────────── */
    if (focState[motor] == FOC_STATE_ALIGN
     || focState[motor] == FOC_STATE_GAIN_CORRECTION)
    {
        const AlphaBeta_t vAlpha = { .alpha = alignVd_V[motor], .beta = 0.0f };
        SVPWM(vAlpha, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);
        updateEncoder(motor);
        return;
    }


#if openLoop == 1
    {
        DQ_t        vdq = { .d = openLoopVd[motor], .q = openLoopVq[motor] };
        const float elec = getElectricalAngle(motor);
        float       sinT, cosT;
        CORDIC_SinCos(elec, &sinT, &cosT);
        AlphaBeta_t vab = inverseParkSC(vdq, sinT, cosT);
        SVPWM(vab, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);

        if (newCurrentData[motor]) {
            Ia[motor] = stagingIa[motor];
            Ib[motor] = stagingIb[motor];
            newCurrentData[motor] = 0;
        }
        AlphaBeta_t iab = clarkeTransform(Ia[motor], Ib[motor]);
        DQ_t idq = parkTransformSC(iab, sinT, cosT);
#if Debug == 1
        debugIa[motor] = Ia[motor];
        debugIb[motor] = Ib[motor];
        debugId[motor] = idq.d;
        debugIq[motor] = idq.q;
        debugElecAngle[motor] = elec;
        debugVd[motor] = vdq.d;
        debugVq[motor] = vdq.q;
        debugEncoderCount[motor] = encoderCount[motor];
#endif
        return;
    }
#endif


    /* ── STAGE 0: copy current snapshot from staging ─────────────────── */
    if (newCurrentData[motor])
    {
        Ia[motor] = stagingIa[motor];
        Ib[motor] = stagingIb[motor];
        newCurrentData[motor] = 0;
    }
#if Debug == 1
    debugIa[motor] = Ia[motor];
    debugIb[motor] = Ib[motor];
#endif


    /* ── STAGE 1: encoder update ─────────────────────────────────────── */
    const int32_t currentCount = readEncoderCnt(motor);
    int32_t delta = currentCount - lastCount[motor];
    if (delta >  32767) delta -= 65536;
    if (delta < -32768) delta += 65536;

    if (delta > 100 || delta < -100) {
        emergencyStop(FAULT_ENCODER_DISCONTINUITY);
        return;
    }
    encoderCount[motor] += delta;
    lastCount[motor]     = currentCount;

#if Debug == 1
    debugEncoderCount[motor] = encoderCount[motor];
#endif


    /* ── STAGE 2: angle + velocity estimate + prediction ─────────────── */
    #define TWO_PI_F            6.28318530717958647693f
    #define ELEC_RAD_PER_COUNT  ((TWO_PI_F / (float)COUNTS_PER_REV) * (float)POLE_PAIRS)

    const float deltaTheta_e = (float)delta * ELEC_RAD_PER_COUNT;

    static float deltaTheta_e_filt[MTR_AMT] = {0.0f, 0.0f};
    {
        float ferr = deltaTheta_e - deltaTheta_e_filt[motor];
        if      (ferr >  ELEC_RAD_PER_COUNT) ferr =  ELEC_RAD_PER_COUNT;
        else if (ferr < -ELEC_RAD_PER_COUNT) ferr = -ELEC_RAD_PER_COUNT;
        deltaTheta_e_filt[motor] += 0.2f * ferr;
    }

    #define RAD_TO_DEG  57.2957795130823f

    const float absSpeed     = fabsf(deltaTheta_e_filt[motor]);
    const float predictScale = absSpeed / (absSpeed + ELEC_RAD_PER_COUNT);
    const float predictRad   = ANGLE_PREDICT_TS_FULL * predictScale * deltaTheta_e_filt[motor];

    const float elecAngle = getElectricalAngle(motor);
    float       predAngle = elecAngle + predictRad * RAD_TO_DEG;
    predAngle = fmodf(predAngle, 360.0f);
    if (predAngle < 0.0f) predAngle += 360.0f;

#if Debug == 1
    debugElecAngle[motor] = elecAngle;
#endif

    float sinPred, cosPred;
    CORDIC_SinCos(predAngle, &sinPred, &cosPred);

    /* ── STAGE 3: Clarke + Park ──────────────────────────────────────── */
    AlphaBeta_t iab = clarkeTransform(Ia[motor], Ib[motor]);
    DQ_t        idq = parkTransformSC(iab, sinPred, cosPred);
    idq.d -= idOffset[motor];
    idq.q -= iqOffset[motor];

#if Debug == 1
    debugId[motor] = idq.d;
    debugIq[motor] = idq.q;
#endif


    /* ── STAGE 4: torque ramp + PI controllers ───────────────────────── */
#if openLoop == 0
    if (rampedTorque_Nm[motor] < targetTorque_Nm[motor]) {
        rampedTorque_Nm[motor] += TORQUE_RAMP_RATE;
        if (rampedTorque_Nm[motor] > targetTorque_Nm[motor]) rampedTorque_Nm[motor] = targetTorque_Nm[motor];
    }
    else if (rampedTorque_Nm[motor] > targetTorque_Nm[motor]) {
        rampedTorque_Nm[motor] -= TORQUE_RAMP_RATE;
        if (rampedTorque_Nm[motor] < targetTorque_Nm[motor]) rampedTorque_Nm[motor] = targetTorque_Nm[motor];
    }

    targetIq[motor] = rampedTorque_Nm[motor] / KT_NM_PER_AMP;

    const float maxIq = (float)MAX_CURRENT_MA / 1000.0f;
    if (targetIq[motor] >  maxIq) targetIq[motor] =  maxIq;
    if (targetIq[motor] < -maxIq) targetIq[motor] = -maxIq;

    const float Vd = PI_Update(&piD[motor], 0.0f,            idq.d);
    const float Vq = PI_Update(&piQ[motor], targetIq[motor], idq.q);

  #if Debug == 1
    debugVd[motor] = Vd;
    debugVq[motor] = Vq;
  #endif


    /* ── STAGE 5: inverse Park + SVPWM ───────────────────────────────── */
    DQ_t        vdq = { .d = Vd, .q = Vq };
    AlphaBeta_t vab = inverseParkSC(vdq, sinPred, cosPred);
    SVPWM(vab, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);
#endif


    /* ── STAGE 6: data acquisition (M1 only) ─────────────────────────── */
#if dataAqu == 1
    if (motor == MOTOR_1)
    {
        if (aquPi++ >= 40u)
        {
            aquPi = 0;
            if (aquGi < AQU_AMT)
            {
                IdG[aquGi]           = idq.d;
                IqG[aquGi]           = idq.q;
                VdG[aquGi]           = vdq.d;
                VqG[aquGi]           = vdq.q;
                IaG[aquGi]           = Ia[motor];
                IbG[aquGi]           = Ib[motor];
                ElecAngleG[aquGi]    = elecAngle;
                EncoderCountG[aquGi] = encoderCount[motor];
                aquGi++;
            }
        }
    }
#endif
}


/* ============================================================================
 * ADC INJECTED COMPLETE — current-sample pipeline
 * ============================================================================ */
void ADCloop(uint8_t motor)
{
    ADC_HandleTypeDef *adc = (motor == MOTOR_1) ? &hadc2 : &hadc3;
    const float countsA = (float)HAL_ADCEx_InjectedGetValue(adc, ADC_INJECTED_RANK_1);
    const float countsB = (float)HAL_ADCEx_InjectedGetValue(adc, ADC_INJECTED_RANK_2);

    /* Phase 1: zero-current offset calibration */
    if (calibrateCounts[motor] < CALIBRATION_SAMPLES)
    {
        offsetA[motor] += countsA;
        offsetB[motor] += countsB;
        calibrateCounts[motor]++;

        if (calibrateCounts[motor] == CALIBRATION_SAMPLES)
        {
            offsetA[motor] /= (float)CALIBRATION_SAMPLES;
            offsetB[motor] /= (float)CALIBRATION_SAMPLES;
        }
        return;
    }

    const float Ia_amps = (countsA - offsetA[motor]) * current_scalar * gainCorrA[motor];
    const float Ib_amps = (countsB - offsetB[motor]) * current_scalar * gainCorrB[motor];

    /* Per-cycle OCP — gate on THIS motor's state */
    const float ocpLimit = (float)MAX_CURRENT_MA * 0.001f * OCP_MULTIPLIER;
    if (focState[MOTOR_1] != FOC_STATE_ALIGN_SWEEP
     && focState[MOTOR_1] != FOC_STATE_GAIN_CORRECTION
     && focState[MOTOR_1] != FOC_STATE_NEUTRAL
	 && focState[MOTOR_2] != FOC_STATE_NEUTRAL
	 && focState[MOTOR_2] != FOC_STATE_GAIN_CORRECTION
	 && focState[MOTOR_2] != FOC_STATE_ALIGN_SWEEP)
    {
        if (Ia_amps >  ocpLimit || Ia_amps < -ocpLimit) {
            emergencyStop(FAULT_OVERCURRENT_PHA);
            return;
        }
        if (Ib_amps >  ocpLimit || Ib_amps < -ocpLimit) {
            emergencyStop(FAULT_OVERCURRENT_PHB);
            return;
        }
    }

    const float medA = medianFilter(&mfA[motor], Ia_amps);
    const float medB = medianFilter(&mfB[motor], Ib_amps);
    filtIa[motor] = filtIa[motor] + ADC_FILTER_ALPHA * (medA - filtIa[motor]);
    filtIb[motor] = filtIb[motor] + ADC_FILTER_ALPHA * (medB - filtIb[motor]);

    /* Phase 2: gain-cal accumulator while in GAIN_CORRECTION */
    if (focState[motor] == FOC_STATE_GAIN_CORRECTION)
    {
        if (gainSettleSkip[motor] < GAIN_SETTLE_SAMPLES) {
            gainSettleSkip[motor]++;
        }
        else {
            gainSumA[motor] += filtIa[motor];
            gainSumB[motor] += filtIb[motor];
            gainSamples[motor]++;
        }
    }

    stagingIa[motor] = filtIa[motor];
    stagingIb[motor] = filtIb[motor];
    newCurrentData[motor] = 1;
}


/* ============================================================================
 * HAL CALLBACKS — single dispatch point per IRQ family
 * ============================================================================ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2) { ADCloop(MOTOR_1); return; }
    if (hadc->Instance == ADC3) { ADCloop(MOTOR_2); return; }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM17) {
        HAL_IncTick();
        return;
    }
    if (htim->Instance == TIM7) {
        uart_poll_rx();
        uart_check_timeout();
        uart_update_velocity();
        return;
    }
    if (htim->Instance == TIM8)  { FOCroutine(MOTOR_1); return; }
    if (htim->Instance == TIM20) { FOCroutine(MOTOR_2); return; }
}
