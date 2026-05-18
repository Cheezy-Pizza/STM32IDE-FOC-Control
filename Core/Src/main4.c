/* ============================================================================
 * main.c — STM32G474VET6 FOC Motor Control (rev 5)
 *
 * CHANGELOG vs. main4.c
 *   - Consolidated alignment routine: single bidirectional sweep does
 *     alignment + encoder-offset probe + gain calibration in ~3 s/motor
 *     (was ~16 s/motor across alignRotor + probeEncoderOffset).
 *   - Bidirectional cogging-cancellation: rotor approached from both
 *     directions, encoder offset is circular-mean of the two settle
 *     positions. encoderOffset_deg is now MEASURED per boot, not tuned.
 *   - CAL_VD_V doubled to 1.0 V (was 0.5) — adequate alignment torque
 *     to overpower cogging on motors like M2.
 *   - CAL_SWEEP_RATE_DPS = 720 deg/s (was 90) — rotor's mean position
 *     tracks field's mean position better at faster sweep rates.
 *   - ANGLE_PREDICT_TS_FULL reduced from 2.55 to 1.0 — closer to the
 *     real ADC→PWM-update delay (~70 µs).
 *   - encoderOffset_deg starts at {0, 0}; the consolidated calibration
 *     overrides it per boot.
 *   - Fixed main(): both motors stay in NEUTRAL through the entire
 *     calibration phase, then both arm together. Prevents M1 from
 *     bouncing during M2 calibration.
 *   - Dropped broken 180° PWM phase-shift code in startHardwareSync.
 *     CR1.DIR is read-only in center-aligned mode; the write was a
 *     no-op. Both timers now start at CNT=0 in phase. ADC ISRs at NVIC
 *     priority 2 serialize cleanly within the 100 µs PWM window.
 *   - Removed __disable_irq/__enable_irq from CORDIC_SinCos. All callers
 *     are at NVIC priority 2 and cannot preempt each other.
 *   - FOC_STATE_ALIGN_SWEEP no longer auto-increments alignSweepAngle_deg.
 *     The angle is driven from main context by calSweepField for clean
 *     control over both the rate and the endpoint.
 *   - FOC_STATE_ALIGN_SWEEP now tracks encoder count during sweeps so
 *     the readings at settle reflect actual rotor position.
 *
 * Targeted hardware (unchanged):
 *   MCU: STM32G474VET6 @ 170 MHz
 *   Driver: DRV8353RHRGZR — 6× PWM, 40 V/V, 200 mA IDRIVE, 0.2 V VDS
 *   FETs: NCEP020N10LL
 *   Shunts: WSHP28181L000FEA (1 mΩ)
 *   Motor: AT4310 (20 pole pairs, dyno Kt ≈ 0.0676)
 *   Encoder: 2048 CPR magnetic on shaft
 *   Bus: 20 V bench
 *
 * IMPORTANT — set in CubeMX before compiling:
 *   TIM20 OC4 Pulse = 10  (must match TIM8.PulseWidth = 10).
 *   Without this, M2's ADC samples at the wrong PWM phase and currents
 *   read garbage.
 *
 * SAFETY ARCHITECTURE — three nested layers (unchanged):
 *   1. DRV8353R hardware: VDS shoot-through, gate UVLO, OTSD.
 *   2. Firmware ADC layer: per-cycle current ceiling, hard fault.
 *   3. Firmware command layer: torque ramp + watchdog timeout, soft fault.
 * ============================================================================ */

/* USER CODE BEGIN Header */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "crc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include <string.h>
#include <stdint.h>


/* ============================================================================
 * BUILD-TIME OPTIONS
 * ============================================================================ */

/* 1 = open-loop sensor / SVPWM characterisation, 0 = closed-loop torque. */
#define openLoop                0

#if openLoop == 1
static volatile float openLoopVd[2] = {0.0f, 0.0f};
static volatile float openLoopVq[2] = {0.2f, 0.2f};
#endif

/* 1 = exposes live-watch debug variables (debugIa[], debugIq[], etc.). */
#define Debug                   1

/* 1 = circular log buffers, 250 Hz sampling (every 40th FOC cycle). */
#define dataAqu                 0
#define AQU_AMT                 3000

/* 1 = bench torque-square-wave test in main while(1) without UART. */
#define BENCH_TEST_LOOP         0

/* 1 = enable MOTOR_2 init+FOC. */
#define ENABLE_MOTOR_2          0


/* ============================================================================
 * MOTOR PARAMETERS — edit per motor unit
 *
 * Kt of 0.0676 Nm/A is the mid-range mean from dyno calibration.
 * ============================================================================ */
#define POLE_PAIRS              20
#define COUNTS_PER_REV          2048
#define KT_NM_PER_AMP           0.0676f
#define RS_MOHMS                106


/* ============================================================================
 * POWER STAGE PARAMETERS
 * ============================================================================ */
#define BUS_VOLTAGE_MV          20000UL
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
#define MAX_CURRENT_MA          20000
#define OCP_MULTIPLIER          1.75f


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
 * ALIGNMENT / CALIBRATION CONSTANTS
 *
 * Single consolidated calibration:
 *   - Pre-settle:    park rotor at field=0° from arbitrary boot position.
 *   - Forward sweep: 0° → 360° elec at CAL_SWEEP_RATE_DPS. Rotor follows;
 *                    ends approaching 0° from below.
 *   - Settle + gain: hold field at 0°, accumulate Ia/Ib for gain cal,
 *                    read encoder → posBelow.
 *   - Backward sweep: 0° → -360° elec. Rotor approaches 0° from above.
 *   - Settle:        hold field at 0°, read encoder → posAbove.
 *   - Compute offset = -circular_mean(posBelow, posAbove).
 *
 * Total time per motor: ~3.4 s.
 *
 * If the probe spread (|posBelow − posAbove|) exceeds CAL_MAX_SPREAD_DEG,
 * cogging is winning over the field. Bump CAL_VD_V, or check the motor
 * mechanically.
 * ============================================================================ */
#define CAL_VD_V                  0.6f      /* Vd applied during calibration */
#define CAL_SWEEP_RATE_DPS        360.0f    /* elec deg/sec — fast sweep */
#define CAL_TARGET_DEG            0.0f      /* target electrical angle */
#define CAL_PRESETTLE_MS          300u
#define CAL_GAIN_MS               300u
#define CAL_SETTLE_MS             400u
#define CAL_MAX_SPREAD_DEG        45.0f
#define GAIN_SETTLE_SAMPLES       1000UL
#define GAIN_FAULT_RATIO          8.0f
#define GAIN_CORR_CLAMP_LO        0.05f
#define GAIN_CORR_CLAMP_HI        2.20f


/* Alignment scratch globals */
static volatile float alignSweepAngle_deg[MTR_AMT] = {0.0f, 0.0f};
volatile float        encoderOffset_deg[MTR_AMT]   = {0.0f, -8.0f};


/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */
typedef struct { float alpha; float beta; } AlphaBeta_t;
typedef struct { float d;     float q;    } DQ_t;
typedef struct {
    float kp;
    float ki;
    float integral;
    float outMin;
    float outMax;
} PI_t;
typedef struct {
    float   buf[MEDIAN_SIZE];
    uint8_t idx;
} MedianFilter_t;

typedef enum {
    FOC_STATE_NEUTRAL          = 0,
    FOC_STATE_ALIGN_SWEEP      = 1,    /* field at alignSweepAngle_deg */
    FOC_STATE_ALIGN            = 2,    /* legacy hold @ 0° elec */
    FOC_STATE_GAIN_CORRECTION  = 3,    /* field @ 0° elec + ADC accum */
    FOC_STATE_WAIT_UART        = 4,
    FOC_STATE_UART_ENABLED     = 5,
    FOC_STATE_FAULT            = 6
} FocState_t;

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
 * GLOBALS
 * ============================================================================ */

/* --- Encoder --- */
volatile int32_t  encoderCount[MTR_AMT] = {0, 0};
volatile int32_t  lastCount[MTR_AMT]    = {0, 0};
volatile int32_t  encoderSpeed[MTR_AMT] = {0, 0};

/* --- Current sense offsets --- */
volatile float    offsetA[MTR_AMT]         = {0.0f, 0.0f};
volatile float    offsetB[MTR_AMT]         = {0.0f, 0.0f};
volatile uint32_t calibrateCounts[MTR_AMT] = {0, 0};

const float current_scalar = (float)VREF_MV
                           / ((float)ADC_COUNTS
                              * ((float)SHUNT_UOHMS / 1000.0f)
                              * (float)AMP_GAIN);

/* --- Per-channel sensor gain calibration --- */
static float gainCorrA[MTR_AMT] = {1.0f, 1.0f};
static float gainCorrB[MTR_AMT] = {1.0f, 1.0f};

/* --- ADC filter state --- */
static MedianFilter_t mfA[MTR_AMT] = { { {0.0f}, 0 }, { {0.0f}, 0 } };
static MedianFilter_t mfB[MTR_AMT] = { { {0.0f}, 0 }, { {0.0f}, 0 } };

static MedianFilter_t mfVel[MTR_AMT] = { { {0.0f}, 0 }, { {0.0f}, 0 } };

static float filtIa[MTR_AMT] = {0.0f, 0.0f};
static float filtIb[MTR_AMT] = {0.0f, 0.0f};

/* --- Staging buffer (ADC ISR → FOC ISR) --- */
static volatile float   stagingIa[MTR_AMT]    = {0.0f, 0.0f};
static volatile float   stagingIb[MTR_AMT]    = {0.0f, 0.0f};
static volatile uint8_t newCurrentData[MTR_AMT] = {0, 0};

/* --- FOC ISR working copies --- */
static float Ia[MTR_AMT] = {0.0f, 0.0f};
static float Ib[MTR_AMT] = {0.0f, 0.0f};

/* --- DQ frame offsets --- */
static float idOffset[MTR_AMT] = {0.0f, 0.0f};
static float iqOffset[MTR_AMT] = {0.0f, 0.0f};

/* --- PI controllers --- */
static PI_t piD[MTR_AMT] = {
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX },
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX }
};
static PI_t piQ[MTR_AMT] = {
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX },
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX }
};

/* --- Torque command --- */
volatile float    targetTorque_Nm[MTR_AMT]  = {0.0f, 0.0f};
static   float    rampedTorque_Nm[MTR_AMT]  = {0.0f, 0.0f};
static   float    targetIq[MTR_AMT]         = {0.0f, 0.0f};

/* --- FOC state machine --- */
volatile FocState_t focState[MTR_AMT]   = {FOC_STATE_NEUTRAL, FOC_STATE_NEUTRAL};
volatile uint8_t    focEnabled[MTR_AMT] = {0, 0};
static   float      alignVd_V[MTR_AMT]  = {0.0f, 0.0f};

/* --- Faults --- */
volatile uint32_t faultFlags = 0u;
volatile uint8_t  gainFaultA = 0;
volatile uint8_t  gainFaultB = 0;

/* --- Gain calibration accumulators --- */
static float    gainSumA[MTR_AMT]       = {0.0f, 0.0f};
static float    gainSumB[MTR_AMT]       = {0.0f, 0.0f};
static uint32_t gainSamples[MTR_AMT]    = {0, 0};
static uint32_t gainSettleSkip[MTR_AMT] = {0, 0};

/* --- Debug live-watch --- */
#if Debug == 1
volatile float   debugId[MTR_AMT]           = {0.0f, 0.0f};
volatile float   debugIq[MTR_AMT]           = {0.0f, 0.0f};
volatile float   debugElecAngle[MTR_AMT]    = {0.0f, 0.0f};
volatile int32_t debugEncoderCount[MTR_AMT] = {0, 0};
volatile float   debugVd[MTR_AMT]           = {0.0f, 0.0f};
volatile float   debugVq[MTR_AMT]           = {0.0f, 0.0f};
volatile float   debugIa[MTR_AMT]           = {0.0f, 0.0f};
volatile float   debugIb[MTR_AMT]           = {0.0f, 0.0f};
volatile float   debugProbeSpread[MTR_AMT]  = {0.0f, 0.0f};
#endif

/* --- Data acquisition arrays (M1 only) --- */
#if dataAqu == 1
volatile float   IdG[AQU_AMT]           = {0};
volatile float   IqG[AQU_AMT]           = {0};
volatile float   ElecAngleG[AQU_AMT]    = {0};
volatile float   VdG[AQU_AMT]           = {0};
volatile float   VqG[AQU_AMT]           = {0};
volatile float   IaG[AQU_AMT]           = {0};
volatile float   IbG[AQU_AMT]           = {0};
volatile int32_t EncoderCountG[AQU_AMT] = {0};
static uint32_t  Gi = 0;
static uint16_t  Pi = 0;
#endif

/* --- HAL handles --- */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;


/* ============================================================================
 * FORWARD DECLARATIONS
 * ============================================================================ */
static void        SystemClock_Config       (void);
static void        CORDIC_SinCos            (float angle_deg, float *sinVal, float *cosVal);
static float       medianFilter             (MedianFilter_t *f, float newVal);
static AlphaBeta_t clarkeTransform          (float Ia_in, float Ib_in);
static DQ_t        parkTransformSC          (AlphaBeta_t ab, float sinTheta, float cosTheta);
static AlphaBeta_t inverseParkSC            (DQ_t dq, float sinTheta, float cosTheta);
static void        SVPWM                    (AlphaBeta_t vab, float vBus, uint32_t arr, uint8_t motor);
static float       PI_Update                (PI_t *pi, float setpoint, float measured);
static float       getElectricalAngle       (uint8_t motor);
static void        applyNeutralOutput       (uint8_t motor);
static void        emergencyStop            (uint32_t reason);
static void        resetFilters             (uint8_t motor);
static void        calibrateDQOffsets       (uint8_t motor);
static void        raiseSoftFault           (uint32_t flag);
static void        clearSoftFault           (uint32_t flag);
static void        FOCroutine               (uint8_t motor);
static void        ADCloop                  (uint8_t motor);
static void        startHardwareSync        (void);
static void        calibrateMotor           (uint8_t motor);
static uint8_t     calibrateMotorAlignment  (uint8_t motor);
static void        calSweepField            (uint8_t motor, float from_deg, float to_deg);
static float       encoderRawElecDeg        (uint8_t motor);
static float       circular_mean_deg        (float a, float b);
static float       circular_diff_deg        (float a, float b);
/* ============================================================================
 * UART HOST PROTOCOL  (unchanged on the wire)
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

static uint8_t          uartRxBuf[UART_RX_BUF_SIZE];
static uint16_t         uartRxTail   = 0;
static uint8_t          uartTxBuf[UART_RSP_FRAME_LEN];
static volatile uint8_t uartTxBusy   = 0;

volatile uint32_t       uartFramesAccepted = 0;
volatile uint32_t       uartFramesDropped  = 0;
volatile uint32_t       uartSyncHunts      = 0;
volatile uint32_t       uartSeqErrors      = 0;
volatile uint8_t        isInit             = 0;
volatile float          pos_M1_Offset      = 0.0f;
volatile float          pos_M2_Offset      = 0.0f;

static uint8_t          uartLastSeq     = 0;
static uint8_t          uartFirstFrame  = 1;
static uint32_t         uartLastValidMs = 0;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        uartTxBusy = 0;
}


static inline float uart_load_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}
static inline void uart_store_f32(uint8_t *p, float v)
{
    memcpy(p, &v, 4);
}


static void uart_send_response(uint8_t seq)
{
    if (uartTxBusy)
    {
        uartFramesDropped++;
        return;
    }

    float pos_M1 = (float)encoderCount[MOTOR_1] / (float)COUNTS_PER_REV - pos_M1_Offset;
    float vel_M1raw = (float)encoderSpeed[MOTOR_1] / (float)COUNTS_PER_REV;

    float pos_M2 = (float)encoderCount[MOTOR_2] / (float)COUNTS_PER_REV - pos_M2_Offset;
    float vel_M2 = (float)encoderSpeed[MOTOR_2] / (float)COUNTS_PER_REV;

    const float vel_M1 = medianFilter(&mfVel[MOTOR_1], vel_M1raw);

    uartTxBuf[0] = UART_SYNC_TX;
    uartTxBuf[1] = seq;

    if (isInit == 1)
    {
        uint32_t initSend = UART_INIT_NAN_SEND;
        isInit = 0;
        memcpy(&uartTxBuf[2], &initSend, 4);
    }
    else
    {
        uart_store_f32(&uartTxBuf[2], pos_M1);
    }
    uart_store_f32(&uartTxBuf[6],  pos_M2);
    uart_store_f32(&uartTxBuf[10], vel_M1);
    uart_store_f32(&uartTxBuf[14], vel_M2);

    uartTxBusy = 1;
    if (HAL_UART_Transmit_IT(&huart1, uartTxBuf, UART_RSP_FRAME_LEN) != HAL_OK)
    {
        uartTxBusy = 0;
        uartFramesDropped++;
    }
}


static void uart_apply_frame(const uint8_t *frame)
{
    const uint8_t seq    = frame[1];
    float         tq_M1  = uart_load_f32(&frame[2]);
    float         tq_M2  = uart_load_f32(&frame[6]);

    if (!uartFirstFrame)
    {
        if (seq != (uint8_t)(uartLastSeq + 1u))
            uartSeqErrors++;
    }
    uartFirstFrame = 0;
    uartLastSeq    = seq;

    const float TQ_LIMIT = (float)MAX_CURRENT_MA * 0.001f * KT_NM_PER_AMP;

    /* Special command: init NaN. Enables ALL motors waiting for arm. */
    uint32_t initCheck;
    memcpy(&initCheck, &frame[2], 4);
    if (initCheck == UART_INIT_NAN)
    {
        uint8_t armed = 0;
        for (int m = 0; m < MTR_AMT; m++)
        {
            focState[m] = FOC_STATE_UART_ENABLED;
            armed = 1;
        }
        if (armed) isInit = 1;
    }
    if (initCheck == UART_ZERO_ENCODERS_NAN)
    {
        pos_M1_Offset = (float)encoderCount[MOTOR_1] / (float)COUNTS_PER_REV;
        pos_M2_Offset = (float)encoderCount[MOTOR_2] / (float)COUNTS_PER_REV;
    }

    /* Per-motor NaN/clamp handling */
    uint8_t cmdClamped = 0;
    if (tq_M1 != tq_M1) tq_M1 = 0.0f;
    if (tq_M1 >  TQ_LIMIT) { tq_M1 =  TQ_LIMIT; cmdClamped = 1; }
    else if (tq_M1 < -TQ_LIMIT) { tq_M1 = -TQ_LIMIT; cmdClamped = 1; }

    if (tq_M2 != tq_M2) tq_M2 = 0.0f;
    if (tq_M2 >  TQ_LIMIT) { tq_M2 =  TQ_LIMIT; cmdClamped = 1; }
    else if (tq_M2 < -TQ_LIMIT) { tq_M2 = -TQ_LIMIT; cmdClamped = 1; }

    if (cmdClamped) raiseSoftFault(FAULT_TORQUE_CMD_CLAMPED);
    else            clearSoftFault(FAULT_TORQUE_CMD_CLAMPED);

    if (faultFlags & (FAULT_HARD_MASK | FAULT_ALIGN_NOT_MOVED))
    {
        tq_M1 = 0.0f;
        tq_M2 = 0.0f;
    }

    targetTorque_Nm[MOTOR_1] = (focState[MOTOR_1] == FOC_STATE_UART_ENABLED) ? tq_M1 : 0.0f;
    targetTorque_Nm[MOTOR_2] = (focState[MOTOR_2] == FOC_STATE_UART_ENABLED) ? tq_M2 : 0.0f;

    uartLastValidMs = HAL_GetTick();
    uartFramesAccepted++;
    clearSoftFault(FAULT_HOST_TIMEOUT);

    uart_send_response(seq);
}


void uart_poll_rx(void)
{
    uint16_t dmaHead = (uint16_t)(UART_RX_BUF_SIZE
                                  - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));

    while (uartRxTail != dmaHead)
    {
        uint16_t avail = (uint16_t)((dmaHead - uartRxTail + UART_RX_BUF_SIZE)
                                    & (UART_RX_BUF_SIZE - 1u));
        if (avail < UART_CMD_FRAME_LEN)
            return;

        if (uartRxBuf[uartRxTail] != UART_SYNC_RX)
        {
            uartSyncHunts++;
            uartRxTail = (uint16_t)((uartRxTail + 1u) & (UART_RX_BUF_SIZE - 1u));
            continue;
        }

        uint8_t frame[UART_CMD_FRAME_LEN];
        for (uint16_t i = 0; i < UART_CMD_FRAME_LEN; i++)
            frame[i] = uartRxBuf[(uartRxTail + i) & (UART_RX_BUF_SIZE - 1u)];

        uartRxTail = (uint16_t)((uartRxTail + UART_CMD_FRAME_LEN)
                                & (UART_RX_BUF_SIZE - 1u));

        uart_apply_frame(frame);

        dmaHead = (uint16_t)(UART_RX_BUF_SIZE
                             - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    }
}


void uart_check_timeout(void)
{
    if (uartFirstFrame) return;

    if ((HAL_GetTick() - uartLastValidMs) > UART_CMD_TIMEOUT_MS)
    {
        for (int motor = 0; motor < MTR_AMT; motor++)
        {
            targetTorque_Nm[motor] = 0.0f;
        }
        raiseSoftFault(FAULT_HOST_TIMEOUT);
    }
}


void uart_update_velocity(void)
{
    static uint32_t lastTick = 0;
    static int32_t  lastCnt[MTR_AMT] = {0, 0};

    uint32_t now = HAL_GetTick();
    if ((now - lastTick) >= 10u) //if time between samples is >10ms
    {
        for (int motor = 0; motor < MTR_AMT; motor++)
        {
            const int32_t delta = encoderCount[motor] - lastCnt[motor];
            encoderSpeed[motor] = delta * 100; //10ms * 100 is 1 second.
            lastCnt[motor]      = encoderCount[motor];
        }
        lastTick = now;
    }
}


void uart_init(void)
{
    HAL_UART_Receive_DMA(&huart1, uartRxBuf, UART_RX_BUF_SIZE);
}


/* ============================================================================
 * MATH HELPERS
 * ============================================================================ */

/* CORDIC-based sin/cos. No __disable_irq needed: all callers are at NVIC
 * priority 2 and cannot preempt one another. CORDIC accesses are also
 * single-write/double-read, so even race-prone access is bounded. */
static void CORDIC_SinCos(float angle_deg, float *sinVal, float *cosVal)
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

static float medianFilter(MedianFilter_t *f, float newVal)
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


static AlphaBeta_t clarkeTransform(float Ia_in, float Ib_in)
{
    AlphaBeta_t out;
    out.alpha = Ia_in;
    out.beta  = (Ia_in + 2.0f * Ib_in) * 0.57735027f;
    return out;
}


static DQ_t parkTransformSC(AlphaBeta_t ab, float sinTheta, float cosTheta)
{
    DQ_t out;
    out.d =  ab.alpha * cosTheta + ab.beta * sinTheta;
    out.q = -ab.alpha * sinTheta + ab.beta * cosTheta;
    return out;
}


static AlphaBeta_t inverseParkSC(DQ_t dq, float sinTheta, float cosTheta)
{
    AlphaBeta_t out;
    out.alpha = dq.d * cosTheta - dq.q * sinTheta;
    out.beta  = dq.d * sinTheta + dq.q * cosTheta;
    return out;
}


static float getElectricalAngle(uint8_t motor)
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


/* Raw electrical angle from encoder count, IGNORING encoderOffset_deg.
 * Used during calibration to measure where the rotor actually is. */
static float encoderRawElecDeg(uint8_t motor)
{
    const int32_t posCount = ((encoderCount[motor] % COUNTS_PER_REV)
                              + COUNTS_PER_REV) % COUNTS_PER_REV;
    float elec = ((float)posCount / (float)COUNTS_PER_REV)
               * (float)POLE_PAIRS * 360.0f;
    elec = fmodf(elec, 360.0f);
    if (elec < 0.0f) elec += 360.0f;
    return elec;
}


/* Vector mean of two angles, robust to wraparound. Returns [0, 360). */
static float circular_mean_deg(float a, float b)
{
    const float DEG2RAD = 0.01745329252f;
    const float RAD2DEG = 57.29577951f;

    const float ax = cosf(a * DEG2RAD), ay = sinf(a * DEG2RAD);
    const float bx = cosf(b * DEG2RAD), by = sinf(b * DEG2RAD);

    float result = atan2f(ay + by, ax + bx) * RAD2DEG;
    if (result < 0.0f) result += 360.0f;
    return result;
}


/* Signed angular difference a − b, in [-180, 180]. */
static float circular_diff_deg(float a, float b)
{
    float d = a - b;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
/* ============================================================================
 * SVPWM — zero-sequence injection
 * ============================================================================ */
static void SVPWM(AlphaBeta_t vab, float vBus, uint32_t arr, uint8_t motor)
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
 * PI CONTROLLER
 * ============================================================================ */
static float PI_Update(PI_t *pi, float setpoint, float measured)
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
 * UTILITY FUNCTIONS
 * ============================================================================ */
static void applyNeutralOutput(uint8_t motor)
{
    const uint32_t neutral = (uint32_t)(PWM_ARR / 2u);
    TIM_HandleTypeDef *htim = (motor == MOTOR_1) ? &htim8 : &htim20;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, neutral);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, neutral);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, neutral);
}


static void resetFilters(uint8_t motor)
{
    memset(&mfA[motor], 0, sizeof(MedianFilter_t));
    memset(&mfB[motor], 0, sizeof(MedianFilter_t));
    filtIa[motor] = 0.0f;
    filtIb[motor] = 0.0f;
    stagingIa[motor] = 0.0f;
    stagingIb[motor] = 0.0f;
    Ia[motor] = 0.0f;
    Ib[motor] = 0.0f;
    newCurrentData[motor] = 0;
}


static void emergencyStop(uint32_t reason)
{
    for (int m = 0; m < MTR_AMT; m++)
    {
        focState[m]        = FOC_STATE_FAULT;
        focEnabled[m]      = 0;
        piD[m].integral    = 0.0f;
        piQ[m].integral    = 0.0f;
        rampedTorque_Nm[m] = 0.0f;
        targetIq[m]        = 0.0f;
        targetTorque_Nm[m] = 0.0f;
    }
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim20);

    faultFlags |= reason;
}


static void raiseSoftFault(uint32_t flag)
{
    faultFlags |= flag;
    if (flag & FAULT_GAIN_BAD_PHA) gainFaultA = 1;
    if (flag & FAULT_GAIN_BAD_PHB) gainFaultB = 1;
}

static void clearSoftFault(uint32_t flag)
{
    faultFlags &= ~flag;
    if (flag & FAULT_GAIN_BAD_PHA) gainFaultA = 0;
    if (flag & FAULT_GAIN_BAD_PHB) gainFaultB = 0;
}


/* ============================================================================
 * FOC ROUTINE — runs at 10 kHz from TIM8 (M1) or TIM20 (M2) period elapsed
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
     * Used during the consolidated calibration. The angle is set from main
     * context (calSweepField). This handler just reads the angle, applies
     * the field, and tracks the encoder count — the rotor IS moving here. */
    if (focState[motor] == FOC_STATE_ALIGN_SWEEP)
    {
        float sinA, cosA;
        CORDIC_SinCos(alignSweepAngle_deg[motor], &sinA, &cosA);

        const AlphaBeta_t vAlpha = {
            .alpha = alignVd_V[motor] * cosA,
            .beta  = alignVd_V[motor] * sinA
        };
        SVPWM(vAlpha, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);

        /* Track encoder during sweep */
        int32_t currentCount;
        if (motor == MOTOR_1) currentCount = htim3.Instance->CNT;
        else                  currentCount = htim4.Instance->CNT;

        int32_t delta = currentCount - lastCount[motor];
        if (delta >  32767) delta -= 65536;
        if (delta < -32768) delta += 65536;
        encoderCount[motor] += delta;
        lastCount[motor]     = currentCount;

        return;
    }

    /* ── HOLD-AT-ZERO HANDLER (for legacy ALIGN and gain-cal) ───────── */
    if (focState[motor] == FOC_STATE_ALIGN
     || focState[motor] == FOC_STATE_GAIN_CORRECTION)
    {
        const AlphaBeta_t vAlpha = { .alpha = alignVd_V[motor], .beta = 0.0f };
        SVPWM(vAlpha, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);

        /* Also track encoder — rotor may slip at cogging detents. */
        int32_t currentCount;
        if (motor == MOTOR_1) currentCount = htim3.Instance->CNT;
        else                  currentCount = htim4.Instance->CNT;

        int32_t delta = currentCount - lastCount[motor];
        if (delta >  32767) delta -= 65536;
        if (delta < -32768) delta += 65536;
        encoderCount[motor] += delta;
        lastCount[motor]     = currentCount;

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
    int32_t currentCount;
    if (motor == MOTOR_1) currentCount = htim3.Instance->CNT;
    else                  currentCount = htim4.Instance->CNT;

    int32_t delta = currentCount - lastCount[motor];

    if (delta >  32767) delta -= 65536;
    if (delta < -32768) delta += 65536;

    if (delta > 100 || delta < -100)
    {
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

    /* Reduced from 2.55 — that value over-predicted by ~3.6× given the
     * actual ADC→PWM-update delay (~70 µs). 1.0 corresponds to ~100 µs of
     * prediction, much closer to physical delay. */
    #define ANGLE_PREDICT_TS_FULL   0.5f
    #define RAD_TO_DEG              57.2957795130823f

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

    float sinTheta, cosTheta, sinPred, cosPred;
    CORDIC_SinCos(elecAngle, &sinTheta, &cosTheta);
    CORDIC_SinCos(predAngle, &sinPred,  &cosPred);


    /* ── STAGE 3: Clarke + Park ──────────────────────────────────────── */
    AlphaBeta_t iab = clarkeTransform(Ia[motor], Ib[motor]);
    DQ_t        idq = parkTransformSC(iab, sinTheta, cosTheta);
    idq.d -= idOffset[motor];
    idq.q -= iqOffset[motor];

#if Debug == 1
    debugId[motor] = idq.d;
    debugIq[motor] = idq.q;
#endif


    /* ── STAGE 4: torque ramp + PI controllers ───────────────────────── */
#if openLoop == 0
    if (rampedTorque_Nm[motor] < targetTorque_Nm[motor])
    {
        rampedTorque_Nm[motor] += TORQUE_RAMP_RATE;
        if (rampedTorque_Nm[motor] > targetTorque_Nm[motor]) rampedTorque_Nm[motor] = targetTorque_Nm[motor];
    }
    else if (rampedTorque_Nm[motor] > targetTorque_Nm[motor])
    {
        rampedTorque_Nm[motor] -= TORQUE_RAMP_RATE;
        if (rampedTorque_Nm[motor] < targetTorque_Nm[motor]) rampedTorque_Nm[motor] = targetTorque_Nm[motor];
    }

    targetIq[motor] = rampedTorque_Nm[motor] / KT_NM_PER_AMP;

    const float maxIq = (float)MAX_CURRENT_MA / 1000.0f;
    if (targetIq[motor] >  maxIq) targetIq[motor] =  maxIq;
    if (targetIq[motor] < -maxIq) targetIq[motor] = -maxIq;

    const float Vd = PI_Update(&piD[motor], 0.0f,             idq.d);
    const float Vq = PI_Update(&piQ[motor], targetIq[motor],  idq.q);

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
        if (Pi++ >= 40u)
        {
            Pi = 0;
            if (Gi < AQU_AMT)
            {
                IdG[Gi]           = idq.d;
                IqG[Gi]           = idq.q;
                VdG[Gi]           = vdq.d;
                VqG[Gi]           = vdq.q;
                IaG[Gi]           = Ia[motor];
                IbG[Gi]           = Ib[motor];
                ElecAngleG[Gi]    = elecAngle;
                EncoderCountG[Gi] = encoderCount[motor];
                Gi++;
            }
        }
    }
#endif
}


/* ============================================================================
 * ADC INJECTED COMPLETE — 10 kHz, triggered by TIM8 / TIM20 TRGO2
 * ============================================================================ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        ADCloop(MOTOR_1);
        return;
    }
    if (hadc->Instance == ADC3)
    {
        ADCloop(MOTOR_2);
        return;
    }
}


static void ADCloop(uint8_t motor)
{
    ADC_HandleTypeDef *adc = (motor == MOTOR_1) ? &hadc2 : &hadc3;
    const float countsA = (float)HAL_ADCEx_InjectedGetValue(adc, ADC_INJECTED_RANK_1);
    const float countsB = (float)HAL_ADCEx_InjectedGetValue(adc, ADC_INJECTED_RANK_2);

    /* PHASE 1: zero-current offset calibration */
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
    if (focState[motor] != FOC_STATE_GAIN_CORRECTION
     && focState[motor] != FOC_STATE_ALIGN
     && focState[motor] != FOC_STATE_ALIGN_SWEEP)
    {
        if (Ia_amps >  ocpLimit || Ia_amps < -ocpLimit)
        {
            emergencyStop(FAULT_OVERCURRENT_PHA);
            return;
        }
        if (Ib_amps >  ocpLimit || Ib_amps < -ocpLimit)
        {
            emergencyStop(FAULT_OVERCURRENT_PHB);
            return;
        }
    }

    const float medA = medianFilter(&mfA[motor], Ia_amps);
    const float medB = medianFilter(&mfB[motor], Ib_amps);
    filtIa[motor] = filtIa[motor] + ADC_FILTER_ALPHA * (medA - filtIa[motor]);
    filtIb[motor] = filtIb[motor] + ADC_FILTER_ALPHA * (medB - filtIb[motor]);

    /* PHASE 2: gain calibration accumulator while in GAIN_CORRECTION */
    if (focState[motor] == FOC_STATE_GAIN_CORRECTION)
    {
        if (gainSettleSkip[motor] < GAIN_SETTLE_SAMPLES)
        {
            gainSettleSkip[motor]++;
        }
        else
        {
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
 * TIMER PERIOD-ELAPSED CALLBACK
 * ============================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM17)
    {
        HAL_IncTick();
        return;
    }

    if (htim->Instance == TIM7)
    {
        uart_poll_rx();
        uart_check_timeout();
        uart_update_velocity();
        return;
    }

    if (htim->Instance == TIM8)
    {
        FOCroutine(MOTOR_1);
        return;
    }

    if (htim->Instance == TIM20)
    {
        FOCroutine(MOTOR_2);
        return;
    }
}
/* ============================================================================
 * CONSOLIDATED ALIGNMENT — replaces alignRotor + probeEncoderOffset
 *
 * Single function does in one pass:
 *   - Forward field rotation     (rotor approaches 0° from below)
 *   - Settle + read encoder      (records posBelow)
 *   - Backward field rotation    (rotor approaches 0° from above)
 *   - Settle + read encoder      (records posAbove)
 *   - Gain cal during settle 1
 *   - Encoder offset = -circular_mean(posBelow, posAbove)
 *   - Validation: probe spread must be < CAL_MAX_SPREAD_DEG
 *
 * Total time per motor: ~3.4 s.
 *
 * Returns 1 on success, 0 on probe-spread fault.
 * ============================================================================ */
static void calSweepField(uint8_t motor, float from_deg, float to_deg)
{
    const float total_deg = to_deg - from_deg;
    const float duration_s = fabsf(total_deg) / CAL_SWEEP_RATE_DPS;
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


static uint8_t calibrateMotorAlignment(uint8_t motor)
{
    /* Fresh start: clear offset for raw measurement. */
    encoderOffset_deg[motor] = 0.0f;

    alignVd_V[motor]           = CAL_VD_V;
    alignSweepAngle_deg[motor] = CAL_TARGET_DEG;

    gainSumA[motor]       = 0.0f;
    gainSumB[motor]       = 0.0f;
    gainSamples[motor]    = 0;
    gainSettleSkip[motor] = 0;

    /* Phase 1: park rotor at field=0° from arbitrary boot position. */
    focState[motor] = FOC_STATE_ALIGN_SWEEP;
    HAL_Delay(CAL_PRESETTLE_MS);

    /* Phase 2: forward sweep 0° → 360°. Rotor follows;
     * ends at 0° elec approached from below. */
    calSweepField(motor, CAL_TARGET_DEG, CAL_TARGET_DEG + 360.0f);
    /* Wrap angle representation back to [0, 360) cosmetically. */
    alignSweepAngle_deg[motor] = CAL_TARGET_DEG;

    /* Phase 3: settle at 0° + accumulate gain-cal samples. */
    focState[motor] = FOC_STATE_GAIN_CORRECTION;
    HAL_Delay(CAL_GAIN_MS);

    /* Snapshot gain-cal results before phase change. */
    __disable_irq();
    const float    snapSumA      = gainSumA[motor];
    const float    snapSumB      = gainSumB[motor];
    const uint32_t snapSamples   = gainSamples[motor];
    const float    elec_from_below = encoderRawElecDeg(motor);
    /* Reset accumulators for cleanliness. */
    gainSumA[motor] = 0.0f;
    gainSumB[motor] = 0.0f;
    gainSamples[motor] = 0;
    gainSettleSkip[motor] = 0;
    __enable_irq();

    /* Phase 4: backward sweep 0° → -360°. Rotor approaches from above. */
    focState[motor] = FOC_STATE_ALIGN_SWEEP;
    calSweepField(motor, CAL_TARGET_DEG, CAL_TARGET_DEG - 360.0f);
    alignSweepAngle_deg[motor] = CAL_TARGET_DEG;

    /* Phase 5: settle, read encoder. */
    HAL_Delay(CAL_SETTLE_MS);

    __disable_irq();
    const float elec_from_above = encoderRawElecDeg(motor);
    __enable_irq();

    /* Phase 6: release field. */
    alignVd_V[motor] = 0.0f;
    focState[motor]  = FOC_STATE_NEUTRAL;
    HAL_Delay(50);

    /* Phase 7: validate spread. */
    const float spread_signed = circular_diff_deg(elec_from_below, elec_from_above);
    const float spread_abs    = (spread_signed < 0.0f) ? -spread_signed : spread_signed;

#if Debug == 1
    debugProbeSpread[motor] = spread_abs;
#endif

    if (spread_abs > CAL_MAX_SPREAD_DEG)
    {
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
        return 0;
    }

    /* Compute true rotor angle as circular mean of the two approaches.
     * The cogging-induced bias from each direction cancels here. */
    const float true_elec_at_target = circular_mean_deg(elec_from_below, elec_from_above);

    /* The rotor is physically at CAL_TARGET_DEG, but the encoder reads
     * true_elec_at_target. We want getElectricalAngle() = CAL_TARGET_DEG
     * when encoder reads true_elec_at_target. Since:
     *   getElectricalAngle = encoderRawElecDeg + encoderOffset_deg (mod 360)
     * we set:
     *   encoderOffset_deg = CAL_TARGET_DEG - true_elec_at_target (mod 360) */
    float offset = CAL_TARGET_DEG - true_elec_at_target;
    while (offset <    0.0f) offset += 360.0f;
    while (offset >= 360.0f) offset -= 360.0f;
    encoderOffset_deg[motor] = offset;

    /* Phase 8: process gain-cal data from the settle window. */
    if (snapSamples > 100u)
    {
        const float Ia_meas = snapSumA / (float)snapSamples;
        const float Ib_meas = snapSumB / (float)snapSamples;
        const float Rs_ohms = (float)RS_MOHMS / 1000.0f;
        const float Ia_exp  =  CAL_VD_V / Rs_ohms;
        const float Ib_exp  = -CAL_VD_V / (2.0f * Rs_ohms);

        const uint8_t signsOk = (Ia_meas > 0.0f) && (Ib_meas < 0.0f);
        const uint8_t magsOk  = (fabsf(Ia_meas) > 0.5f * fabsf(Ia_exp))
                             && (fabsf(Ib_meas) > 0.5f * fabsf(Ib_exp));

        if (signsOk && magsOk)
        {
            float ratioA = Ia_exp / Ia_meas;
            float ratioB = Ib_exp / Ib_meas;

            if (ratioA > (1.0f / GAIN_FAULT_RATIO) && ratioA < GAIN_FAULT_RATIO)
            {
                if (ratioA < GAIN_CORR_CLAMP_LO) { ratioA = GAIN_CORR_CLAMP_LO; raiseSoftFault(FAULT_GAIN_BAD_PHA); }
                else if (ratioA > GAIN_CORR_CLAMP_HI) { ratioA = GAIN_CORR_CLAMP_HI; raiseSoftFault(FAULT_GAIN_BAD_PHA); }
                else clearSoftFault(FAULT_GAIN_BAD_PHA);
                gainCorrA[motor] = ratioA;
            }
            else
            {
                gainCorrA[motor] = 1.0f;
                raiseSoftFault(FAULT_GAIN_BAD_PHA);
            }

            if (ratioB > (1.0f / GAIN_FAULT_RATIO) && ratioB < GAIN_FAULT_RATIO)
            {
                if (ratioB < GAIN_CORR_CLAMP_LO) { ratioB = GAIN_CORR_CLAMP_LO; raiseSoftFault(FAULT_GAIN_BAD_PHB); }
                else if (ratioB > GAIN_CORR_CLAMP_HI) { ratioB = GAIN_CORR_CLAMP_HI; raiseSoftFault(FAULT_GAIN_BAD_PHB); }
                else clearSoftFault(FAULT_GAIN_BAD_PHB);
                gainCorrB[motor] = ratioB;
            }
            else
            {
                gainCorrB[motor] = 1.0f;
                raiseSoftFault(FAULT_GAIN_BAD_PHB);
            }
        }
        else
        {
            gainCorrA[motor] = 1.0f;
            gainCorrB[motor] = 1.0f;
        }
    }

    return 1;
}


/* ============================================================================
 * CALIBRATE DQ OFFSETS
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
 * MOTOR CALIBRATION SEQUENCER
 *
 * Runs the per-motor calibration. Leaves the motor in NEUTRAL on exit;
 * arming happens in main() AFTER both motors finish.
 * ============================================================================ */
static void calibrateMotor(uint8_t motor)
{
    /* Step 1: zero-current ADC offset calibration */
    __disable_irq();
    calibrateCounts[motor] = 0;
    offsetA[motor] = 0.0f;
    offsetB[motor] = 0.0f;
    __enable_irq();
    while (calibrateCounts[motor] < CALIBRATION_SAMPLES) HAL_Delay(10);

    /* Step 2: consolidated alignment (sweep + probe + gain-cal) */
    (void)calibrateMotorAlignment(motor);

    /* Step 3: clean filter + integrator state */
    resetFilters(motor);
    piD[motor].integral    = 0.0f;
    piQ[motor].integral    = 0.0f;
    rampedTorque_Nm[motor] = 0.0f;
    targetIq[motor]        = 0.0f;
    alignVd_V[motor]       = 0.0f;

    /* Step 4: re-run zero-current ADC offset cal AFTER applying gain
     * correction (so offsets are calibrated through the corrected scale) */
    __disable_irq();
    calibrateCounts[motor] = 0;
    offsetA[motor] = 0.0f;
    offsetB[motor] = 0.0f;
    __enable_irq();
    while (calibrateCounts[motor] < CALIBRATION_SAMPLES) HAL_Delay(10);
    HAL_Delay(50);

    /* Step 5: DQ-frame offset cal (no current applied). */
    calibrateDQOffsets(motor);

    /* Step 6: explicit park in NEUTRAL. main() arms motors after both finish. */
    focState[motor]   = FOC_STATE_NEUTRAL;
    focEnabled[motor] = 0;
}


/* ============================================================================
 * HARDWARE SYNC & INITIALIZATION
 *
 * Both timers start at CNT=0, in phase. The 180° phase-shift attempt in
 * the previous version relied on writing CR1.DIR, which is read-only in
 * center-aligned mode. With the priority-2 ADC ISRs serializing within
 * the 100 µs PWM window, in-phase operation is fine.
 * ============================================================================ */
static void startHardwareSync(void)
{
    /* 1. Lock ISRs in NEUTRAL so they don't apply torque prematurely. */
    for (int m = 0; m < MTR_AMT; m++) {
        focState[m] = FOC_STATE_NEUTRAL;
        focEnabled[m] = 0;
    }

    /* 2. ADC hardware calibration */
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

    /* 3. Start encoders */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    lastCount[MOTOR_1] = htim3.Instance->CNT;
    lastCount[MOTOR_2] = htim4.Instance->CNT;

    /* 4. Enable gate drivers (DRV8353), wait for charge pumps */
    HAL_GPIO_WritePin(ENABLE_3_GPIO_Port, ENABLE_3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENABLE_1_GPIO_Port, ENABLE_1_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    applyNeutralOutput(MOTOR_1);
    applyNeutralOutput(MOTOR_2);

    /* 5. Start PWM generation */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1); HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2); HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_3); HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_3);

    /* 6. Start ADCs in injected mode */
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);

    /* 7. Synchronize the two FOC timers — both at CNT=0, then enable.
     * In-phase, but priority-2 ADC ISRs serialize cleanly. */
    htim8.Instance->CR1  &= ~TIM_CR1_CEN;
    htim20.Instance->CR1 &= ~TIM_CR1_CEN;

    htim8.Instance->CNT  = 0;
    htim20.Instance->CNT = 0;

    __HAL_TIM_CLEAR_IT(&htim8,  TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_IT(&htim20, TIM_IT_UPDATE);

    __HAL_TIM_ENABLE_IT(&htim8,  TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim20, TIM_IT_UPDATE);

    htim8.Instance->CR1  |= TIM_CR1_CEN;
    htim20.Instance->CR1 |= TIM_CR1_CEN;
}


/* ============================================================================
 * MAIN
 * ============================================================================ */
int main(void)
{

	HAL_Init();
    SystemClock_Config();

    /* CubeMX-generated peripheral inits */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_TIM1_Init();
    MX_CORDIC_Init();
    MX_CRC_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM7_Init();
    MX_TIM8_Init();
    MX_TIM20_Init();
    MX_USART1_UART_Init();

    uart_init();
    faultFlags = 0u;

    /* 1. Start all hardware. Both motors in NEUTRAL. */
    startHardwareSync();

    /* 2. Calibrate motor 1. Stays in NEUTRAL on exit. */
    calibrateMotor(MOTOR_1);

#if ENABLE_MOTOR_2 == 1
    /* 3. Calibrate motor 2. M1 stays in NEUTRAL — no bouncing. */
    calibrateMotor(MOTOR_2);
#endif

    /* 4. Now arm both motors together (only if no hard faults / spread fail) */
    const uint8_t arm_ok = !(faultFlags & (FAULT_HARD_MASK | FAULT_ALIGN_NOT_MOVED));
    if (arm_ok)
    {
        for (int m = 0; m < MTR_AMT; m++)
        {
#if ENABLE_MOTOR_2 == 0
            if (m == MOTOR_2) continue;
#endif
#if BENCH_TEST_LOOP == 1
            focState[m]   = FOC_STATE_UART_ENABLED;
#else
            focState[m]   = FOC_STATE_WAIT_UART;
#endif
            focEnabled[m] = 1;
        }
    }

    /* 5. Start TIM7 1 kHz health/UART timer */
    HAL_TIM_Base_Start_IT(&htim7);


    /* ========================================================================
     * APPLICATION LOOP
     * ======================================================================== */
    while (1)
    {
#if BENCH_TEST_LOOP == 1
        targetTorque_Nm[MOTOR_1] = 0.03f;
        HAL_Delay(2000);
        targetTorque_Nm[MOTOR_1] = 0.04f;
        HAL_Delay(2000);
        targetTorque_Nm[MOTOR_1] = 0.05f;
        HAL_Delay(2000);
        targetTorque_Nm[MOTOR_1] = 0.00f;
        HAL_Delay(2000);
        targetTorque_Nm[MOTOR_1] = -0.03f;
        HAL_Delay(2000);
        targetTorque_Nm[MOTOR_1] = -0.04f;
        HAL_Delay(2000);
        targetTorque_Nm[MOTOR_1] = -0.05f;
        HAL_Delay(2000);
        targetTorque_Nm[MOTOR_1] = 0.00f;
        HAL_Delay(2000);
  #if ENABLE_MOTOR_2 == 1
        targetTorque_Nm[MOTOR_2] = 0.03f;
  #endif

  #if ENABLE_MOTOR_2 == 1
        targetTorque_Nm[MOTOR_2] = -0.03f;
  #endif

#else
        HAL_Delay(10);
#endif
    }
}


/* ============================================================================
 * SYSTEM CLOCK CONFIGURATION
 * ============================================================================ */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1
                                     | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}


/* ============================================================================
 * ERROR HANDLER
 * ============================================================================ */
void Error_Handler(void)
{
    emergencyStop(FAULT_ADC_STUCK);
    while (1) { /* halt */ }
}
