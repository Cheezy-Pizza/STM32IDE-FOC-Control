/* ============================================================================
 * foc_types.h — Type definitions for FOC controller.
 * ============================================================================ */
#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>
#include "foc_config.h"


/* ============================================================================
 * TRANSFORM RESULT TYPES
 * ============================================================================ */
typedef struct { float alpha; float beta; } AlphaBeta_t;
typedef struct { float d;     float q;    } DQ_t;


/* ============================================================================
 * PI CONTROLLER
 * ============================================================================ */
typedef struct {
    float kp;
    float ki;
    float integral;
    float outMin;
    float outMax;
} PI_t;


/* ============================================================================
 * MEDIAN FILTER
 * ============================================================================ */
typedef struct {
    float   buf[MEDIAN_SIZE];
    uint8_t idx;
} MedianFilter_t;


/* ============================================================================
 * FOC STATE MACHINE
 * ============================================================================ */
typedef enum {
    FOC_STATE_NEUTRAL          = 0,
    FOC_STATE_ALIGN_SWEEP      = 1,    /* field at alignSweepAngle_deg */
    FOC_STATE_ALIGN            = 2,    /* legacy hold @ 0° elec */
    FOC_STATE_GAIN_CORRECTION  = 3,    /* field @ 0° + ADC gain accum */
    FOC_STATE_WAIT_UART        = 4,
    FOC_STATE_UART_ENABLED     = 5,
    FOC_STATE_FAULT            = 6
} FocState_t;


/* ============================================================================
 * FAULT FLAGS
 * ============================================================================ */
typedef enum {
    /* HARD — focState→FAULT, PWM off, no auto-recovery */
    FAULT_OVERCURRENT_PHA      = (1u << 0),
    FAULT_OVERCURRENT_PHB      = (1u << 1),
    FAULT_BUS_OVERVOLTAGE      = (1u << 2),
    FAULT_BUS_UNDERVOLTAGE     = (1u << 3),
    FAULT_ENCODER_DISCONTINUITY= (1u << 4),
    FAULT_ADC_STUCK            = (1u << 5),

    /* SOFT — torque clamp + UART notification */
    FAULT_GAIN_BAD_PHA         = (1u << 8),
    FAULT_GAIN_BAD_PHB         = (1u << 9),
    FAULT_HOST_TIMEOUT         = (1u << 10),
    FAULT_ALIGN_NOT_MOVED      = (1u << 11),
    FAULT_DQ_OFFSET_LARGE      = (1u << 12),
    FAULT_TORQUE_CMD_CLAMPED   = (1u << 13),

    FAULT_HARD_MASK            = 0x00FF,
    FAULT_SOFT_MASK            = 0xFF00
} FaultFlags_t;


/* ============================================================================
 * LINEAR-REGRESSION FIT RESULT
 * ============================================================================ */
typedef struct {
    float    slope;             /* encoder counts per elec degree */
    float    intercept;         /* count at field=0° */
    float    residual_rms_deg;  /* fit error in elec degrees */
    int8_t   direction;         /* +1 or -1 */
    uint8_t  scale_ok;
    uint8_t  fit_ok;
} LinRegFit_t;


#endif /* FOC_TYPES_H */
