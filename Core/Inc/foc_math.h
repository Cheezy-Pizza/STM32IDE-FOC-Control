/* ============================================================================
 * foc_math.h — Math primitives for FOC controller.
 *   - CORDIC-based sin/cos
 *   - Median filter
 *   - Clarke / Park / inverse Park transforms
 *   - Space-vector PWM
 *   - PI controller
 *   - Encoder→electrical angle helpers
 *   - Circular angle helpers (mean, signed difference)
 * ============================================================================ */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>
#include "foc_config.h"
#include "foc_types.h"


/* ---------- Trigonometry ---------- */
void        CORDIC_SinCos     (float angle_deg, float *sinVal, float *cosVal);

/* ---------- Filtering ---------- */
float       medianFilter      (MedianFilter_t *f, float newVal);

/* ---------- Reference-frame transforms ---------- */
AlphaBeta_t clarkeTransform   (float Ia_in, float Ib_in);
DQ_t        parkTransformSC   (AlphaBeta_t ab, float sinTheta, float cosTheta);
AlphaBeta_t inverseParkSC     (DQ_t dq, float sinTheta, float cosTheta);

/* ---------- PWM modulator ---------- */
void        SVPWM             (AlphaBeta_t vab, float vBus, uint32_t arr, uint8_t motor);

/* ---------- PI controller ---------- */
float       PI_Update         (PI_t *pi, float setpoint, float measured);

/* ---------- Angle helpers ---------- */
float       getElectricalAngle(uint8_t motor);   /* with offset applied */
float       encoderRawElecDeg (uint8_t motor);   /* offset NOT applied */
float       circular_mean_deg (float a, float b);
float       circular_diff_deg (float a, float b);


#endif /* FOC_MATH_H */
