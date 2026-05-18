/* ============================================================================
 * foc_math.c — Implementation of math primitives.
 * ============================================================================ */
#include <math.h>
#include <string.h>
#include "foc_math.h"
#include "foc_state.h"
#include "cordic.h"
#include "tim.h"
#include <math.h>


/* ============================================================================
 * CORDIC SIN/COS
 *
 * No interrupt protection — all callers are at NVIC priority 2 and cannot
 * preempt one another. Two reads + one write, all on memory-mapped CORDIC
 * registers, complete in well under 1 µs.
 * ============================================================================ */
void CORDIC_SinCos(float angle_deg, float *sinVal, float *cosVal)
{
    /* Wrap into [-180, 180] regardless of input magnitude. fmodf is exact
     * enough here because the input is degrees and we don't need <0.001°
     * precision for FOC. */
    float wrapped = fmodf(angle_deg, 360.0f);
    if (wrapped >  180.0f) wrapped -= 360.0f;
    if (wrapped < -180.0f) wrapped += 360.0f;

    float norm = wrapped / 180.0f;
    /* norm is now in [-1, 1] guaranteed */

    int32_t angleQ31 = (int32_t)(norm * 2147483647.0f);

    CORDIC->CSR = LL_CORDIC_FUNCTION_COSINE
                | LL_CORDIC_PRECISION_6CYCLES
                | LL_CORDIC_SCALE_0
                | LL_CORDIC_NBWRITE_1
                | LL_CORDIC_NBREAD_2
                | LL_CORDIC_INSIZE_32BITS
                | LL_CORDIC_OUTSIZE_32BITS;

    CORDIC->WDATA = (uint32_t)angleQ31;
    int32_t cosQ31 = (int32_t)CORDIC->RDATA;
    int32_t sinQ31 = (int32_t)CORDIC->RDATA;

    *cosVal = (float)cosQ31 / 2147483648.0f;
    *sinVal = (float)sinQ31 / 2147483648.0f;
}

/* ============================================================================
 * MEDIAN FILTER (size MEDIAN_SIZE, in-place insertion sort)
 * ============================================================================ */
float medianFilter(MedianFilter_t *f, float newVal)
{
    f->buf[f->idx] = newVal;
    f->idx = (uint8_t)((f->idx + 1u) % MEDIAN_SIZE);

    float sorted[MEDIAN_SIZE];
    memcpy(sorted, f->buf, sizeof(sorted));

    for (int i = 1; i < MEDIAN_SIZE; i++)
    {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key)
        {
            sorted[j+1] = sorted[j];
            j--;
        }
        sorted[j+1] = key;
    }
    return sorted[MEDIAN_SIZE / 2];
}


/* ============================================================================
 * CLARKE / PARK / INVERSE PARK TRANSFORMS (amplitude-invariant)
 * ============================================================================ */
AlphaBeta_t clarkeTransform(float Ia_in, float Ib_in)
{
    AlphaBeta_t out;
    out.alpha = Ia_in;
    out.beta  = (Ia_in + 2.0f * Ib_in) * 0.57735027f;   /* 1/sqrt(3) */
    return out;
}


DQ_t parkTransformSC(AlphaBeta_t ab, float sinTheta, float cosTheta)
{
    DQ_t out;
    out.d =  ab.alpha * cosTheta + ab.beta * sinTheta;
    out.q = -ab.alpha * sinTheta + ab.beta * cosTheta;
    return out;
}


AlphaBeta_t inverseParkSC(DQ_t dq, float sinTheta, float cosTheta)
{
    AlphaBeta_t out;
    out.alpha = dq.d * cosTheta - dq.q * sinTheta;
    out.beta  = dq.d * sinTheta + dq.q * cosTheta;
    return out;
}


/* ============================================================================
 * SPACE VECTOR PWM with zero-sequence injection
 * ============================================================================ */
void SVPWM(AlphaBeta_t vab, float vBus, uint32_t arr, uint8_t motor)
{
    const float vHalf = vBus * 0.5f;

    float Va = vab.alpha / vHalf;
    float Vb = (-vab.alpha * 0.5f + vab.beta * 0.86602540f) / vHalf;
    float Vc = (-vab.alpha * 0.5f - vab.beta * 0.86602540f) / vHalf;

    float Vmax = Va; if (Vb > Vmax) Vmax = Vb; if (Vc > Vmax) Vmax = Vc;
    float Vmin = Va; if (Vb < Vmin) Vmin = Vb; if (Vc < Vmin) Vmin = Vc;
    const float Voffset = -0.5f * (Vmax + Vmin);

    float tA = 0.5f + 0.5f * (Va + Voffset);
    float tB = 0.5f + 0.5f * (Vb + Voffset);
    float tC = 0.5f + 0.5f * (Vc + Voffset);

    if (tA > 0.95f) tA = 0.95f; else if (tA < 0.0f) tA = 0.0f;
    if (tB > 0.95f) tB = 0.95f; else if (tB < 0.0f) tB = 0.0f;
    if (tC > 0.95f) tC = 0.95f; else if (tC < 0.0f) tC = 0.0f;

    TIM_HandleTypeDef *htim = (motor == MOTOR_1) ? &htim8 : &htim20;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(tA * (float)arr));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(tB * (float)arr));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(tC * (float)arr));
}


/* ============================================================================
 * PI CONTROLLER (with deadband + anti-windup)
 * ============================================================================ */
float PI_Update(PI_t *pi, float setpoint, float measured)
{
    const float error   = setpoint - measured;
    const float pTerm   = pi->kp * error;
    const float u_unsat = pTerm + pi->integral;

    const int saturated_high = (u_unsat >  pi->outMax);
    const int saturated_low  = (u_unsat <  pi->outMin);
    const int in_deadband    = (error <  PI_DEADBAND_A)
                            && (error > -PI_DEADBAND_A);
    const int wind_high      = saturated_high && (error > 0.0f);
    const int wind_low       = saturated_low  && (error < 0.0f);

    if (!in_deadband && !wind_high && !wind_low)
    {
        pi->integral += pi->ki * error;
        if (pi->integral > pi->outMax) pi->integral = pi->outMax;
        if (pi->integral < pi->outMin) pi->integral = pi->outMin;
    }

    float output = pTerm + pi->integral;
    if (output > pi->outMax) output = pi->outMax;
    if (output < pi->outMin) output = pi->outMin;
    return output;
}


/* ============================================================================
 * ANGLE HELPERS
 * ============================================================================ */
float getElectricalAngle(uint8_t motor)
{
    int32_t posCount = ((encoderCount[motor] % COUNTS_PER_REV)
                        + COUNTS_PER_REV) % COUNTS_PER_REV;

    float mech = (float)posCount / (float)COUNTS_PER_REV;
    float elec = mech * (float)POLE_PAIRS * 360.0f;

    elec += encoderOffset_deg[motor];

    elec = fmodf(elec, 360.0f);
    if (elec < 0.0f) elec += 360.0f;
    return elec;
}


float encoderRawElecDeg(uint8_t motor)
{
    const int32_t posCount = ((encoderCount[motor] % COUNTS_PER_REV)
                              + COUNTS_PER_REV) % COUNTS_PER_REV;
    float elec = ((float)posCount / (float)COUNTS_PER_REV)
               * (float)POLE_PAIRS * 360.0f;
    elec = fmodf(elec, 360.0f);
    if (elec < 0.0f) elec += 360.0f;
    return elec;
}


float circular_mean_deg(float a, float b)
{
    const float DEG2RAD = 0.01745329252f;
    const float RAD2DEG = 57.29577951f;
    const float ax = cosf(a * DEG2RAD), ay = sinf(a * DEG2RAD);
    const float bx = cosf(b * DEG2RAD), by = sinf(b * DEG2RAD);
    float result = atan2f(ay + by, ax + bx) * RAD2DEG;
    if (result < 0.0f) result += 360.0f;
    return result;
}


float circular_diff_deg(float a, float b)
{
    float d = a - b;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
