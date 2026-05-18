/* ============================================================================
 * foc_config.h — Build-time constants for FOC controller.
 * ============================================================================ */
#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include <stdint.h>


/* ============================================================================
 * BUILD-TIME OPTIONS
 * ============================================================================ */
#define openLoop                0
#define Debug                   1
#define dataAqu                 0
#define AQU_AMT                 3000
#define BENCH_TEST_LOOP         1
#define ENABLE_MOTOR_2          1


/* ============================================================================
 * MOTOR PARAMETERS
 * ============================================================================ */
#define POLE_PAIRS              20
#define COUNTS_PER_REV          2048
#define KT_NM_PER_AMP           0.0676f
#define RS_MOHMS                106
#define RS_MOHMS2               120


/* ============================================================================
 * POWER STAGE PARAMETERS
 * ============================================================================ */
#define BUS_VOLTAGE_MV          16000UL
#define AMP_GAIN                40UL
#define SHUNT_UOHMS             1000UL
#define BUS_OV_MV               (BUS_VOLTAGE_MV * 12 / 10)
#define BUS_UV_MV               (BUS_VOLTAGE_MV * 8  / 10)


/* ============================================================================
 * ADC / SAMPLING / PWM
 * ============================================================================ */
#define VREF_MV                 3300UL
#define ADC_COUNTS              4095UL
#define PWM_ARR                 8499UL
#define FOC_FS_HZ               10000UL
#define FOC_TS_S                (1.0f / (float)FOC_FS_HZ)
#define OUT_MAX                 ((float)BUS_VOLTAGE_MV / (1000.0f * 1.7320508f))


/* ============================================================================
 * CURRENT LIMITS
 * ============================================================================ */
#define MAX_CURRENT_MA          24000
#define OCP_MULTIPLIER          1.25f


/* ============================================================================
 * NOISE / FILTER PARAMETERS
 * ============================================================================ */
#define ADC_FILTER_ALPHA        0.95f
#define MEDIAN_SIZE             3
#define PI_DEADBAND_A           0.05f
#define CALIBRATION_SAMPLES     4000UL
#define TORQUE_RAMP_RATE        0.0001f


/* ============================================================================
 * MOTOR INDICES
 * ============================================================================ */
#define MTR_AMT 2
#define MOTOR_1 0
#define MOTOR_2 1


/* ============================================================================
 * CALIBRATION (LINEAR-REGRESSION ALIGNMENT)
 * ============================================================================ */
#define LINREG_VD_V                  1.7f
#define LINREG_RATE_DPS              360.0f
#define LINREG_REVS                  3u
#define LINREG_DURATION_MS           ((uint32_t)(1000u * 360u * LINREG_REVS / (uint32_t)LINREG_RATE_DPS))
#define LINREG_PRESETTLE_MS          1500u
#define LINREG_SETTLE_MS             800u
#define LINREG_SAMPLES               160u
#define LINREG_SAMPLE_SKIP           30u
#define LINREG_GAIN_MS               400u

#define LINREG_SLOPE_TOL_PCT         0.10f
#define LINREG_RESIDUAL_MAX_DEG      12.0f
#define LINREG_DIRECTION_AGREE_DEG   60.0f

#define GAIN_SETTLE_SAMPLES          2000UL
#define GAIN_FAULT_RATIO             8.0f
#define GAIN_CORR_CLAMP_LO           0.05f
#define GAIN_CORR_CLAMP_HI           2.20f


/* ============================================================================
 * UART PROTOCOL
 * ============================================================================ */
#define UART_RX_BUF_SIZE        64u
#define UART_CMD_FRAME_LEN      10u
#define UART_RSP_FRAME_LEN      18u
#define UART_SYNC_RX            0xA5u
#define UART_SYNC_TX            0x5Au
#define UART_CMD_TIMEOUT_MS     400u
#define UART_INIT_NAN           0x7FC00001
#define UART_INIT_NAN_SEND      0x7FC00002
#define UART_ZERO_ENCODERS_NAN  0x7FC00013


/* ============================================================================
 * FOC PREDICTION
 * ============================================================================ */
#define ANGLE_PREDICT_TS_FULL   0.5f


#endif /* FOC_CONFIG_H */
