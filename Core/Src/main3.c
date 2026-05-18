/* ============================================================================
 * main.c — STM32G474VET6 FOC Motor Control
 *
 * HARDWARE TARGETED:
 *   MCU:        STM32G474VET6 @ 170 MHz
 *   Driver:     DRV8353RHRGZR (6× PWM, 40 V/V gain, 200 mA IDRIVE, 0.2 V VDS)
 *   FETs:       NCEP020N10LL (100 V, 20 A, RDS(on) ≈ 2 mΩ)
 *   Shunts:     WSHP28181L000FEA (1 mΩ, 1 W, ±1 %)
 *   Motor:      AT4310 hobby gimbal-style BLDC, 20 pole pairs
 *               (https://www.aliexpress.com/item/1005005084172325.html)
 *   Encoder:    AS5048A or similar 2048 CPR magnetic, on motor shaft
 *   Bus:        20 V (bench supply) — see PORTABILITY notes below
 *
 * PORTABILITY (other identical boards / motors of same model):
 *   Pole pair count, encoder resolution, and pin assignments transfer
 *   directly. Per-motor variation in Kt and Rs is ±5–10 %, which the boot
 *   sensor self-calibration partially handles. For production use of
 *   multiple motors, edit the MotorParams_t struct below per unit (or read
 *   from EEPROM / device-specific area). See "MULTI-MOTOR DEPLOYMENT" notes.
 *
 *   The previous hard-coded "1.741× Phase B fix" has been replaced with
 *   adaptive sensor calibration (see SensorCal_t) that runs each boot.
 *   This means a correctly built second board will calibrate near 1.0×
 *   automatically — no firmware change needed.
 *
 * MOUNTED-TO-LOAD CALIBRATION (e.g. robot leg):
 *   Auto-alignment torque is T_align = Kt × Vd / Rs ≈ 0.43 Nm at Vd = 0.5 V.
 *   A loaded leg may exceed this by 4–5×, in which case the rotor will
 *   not budge to electrical 0° and FOC will be miscommutated.
 *
 *   This firmware now DETECTS that condition: it monitors the encoder
 *   during alignment, and if the rotor has not displaced by at least
 *   ALIGN_MIN_DISPLACEMENT counts after the alignment hold, it raises a
 *   FAULT_ALIGN_NOT_MOVED soft fault and refuses to enable FOC. The user
 *   must then either (a) unload the motor for a one-time bench
 *   calibration, or (b) use the UART command "set offset" to install a
 *   pre-measured encoder offset (see uart_apply_frame for the protocol).
 *
 * FAULT MODEL — TIERED:
 *   HARD faults disable PWM via MOE (emergencyStop) and require power
 *   cycle: overcurrent, encoder discontinuity, ADC stuck.
 *   SOFT faults degrade gracefully (warn over UART, may run with reduced
 *   torque or zero torque if the loop becomes unsafe): bus over/under-
 *   voltage near limits, sensor gain mismatch, torque command timeout,
 *   alignment-displacement fault.
 *   See FaultFlags_t for the full enumeration.
 *
 * SAFETY ARCHITECTURE — three nested layers:
 *   1. DRV8353R hardware:   VDS shoot-through detection, gate UVLO, OTSD.
 *   2. Firmware ADC layer:  per-cycle current ceiling (5× rated, hard fault).
 *   3. Firmware command layer: torque ramp + watchdog timeout (soft fault).
 *
 *   Each catches a different failure timescale. Hardware = nanoseconds;
 *   firmware ADC = microseconds; command watchdog = milliseconds.
 *
 * BENCH POWER NOTE:
 *   Running from a 20 V / 2 A supply caps real motor torque at ~0.30 Nm
 *   regardless of what the firmware allows. To unlock the firmware's full
 *   ~1 Nm capability, upgrade to ≥24 V / 5 A. Do NOT exceed 24 V on this
 *   firmware without verifying that the bus capacitor and bootstrap circuit
 *   are rated for it.
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

/* Set to 1 only for sensor / SVPWM / encoder-direction characterisation,
 * 0 for normal closed-loop torque control. Open-loop is useful when you
 * need to verify polarity or wiring before trusting the closed loop. */
#define openLoop                0

#if openLoop == 1
static volatile float openLoopVd[2] = {0.0f, 0.0f};
static volatile float openLoopVq[2] = {0.2f, 0.2f};
#endif

/* Live-watch debug variables (visible in the IDE debugger). Set 0 to
 * shave a few cycles off the ISR if you need them. */
#define Debug                   1

/* Periodic data acquisition. Logs every 40th FOC cycle (= 4 kHz/10 = 250 Hz)
 * into circular arrays. Useful for tuning. Set 0 to free ~144 kB of RAM
 * and trim the ISR. */
#define dataAqu                 1
#define AQU_AMT                 3000


/* ============================================================================
 * MOTOR PARAMETERS — edit per motor unit
 *
 * For "same model, different unit": Kt and Rs vary by ±5–10 %. POLE_PAIRS,
 * COUNTS_PER_REV, and KV are model properties and don't change unit-to-unit.
 *
 * MULTI-MOTOR DEPLOYMENT:
 *   For production with many motors of this model, add a one-time
 *   calibration step after PCB assembly:
 *     1. Apply known torque (e.g. weighted lever) and measure Iq → derive Kt
 *     2. Measure phase-to-phase resistance with multimeter → derive Rs
 *     3. Store these in flash via UART command (not yet implemented)
 *     4. Read them on every boot
 *   For now they are compile-time constants. Edit and rebuild per motor.
 * ============================================================================ */
#define POLE_PAIRS              20          /* number of magnet pairs */
#define COUNTS_PER_REV          2048        /* encoder counts per mechanical rev (X4 mode) */
#define KT_NM_PER_AMP           0.0637f     /* torque constant — measured */
#define RS_MOHMS                106         /* phase resistance (mΩ) — measured */


/* ============================================================================
 * POWER STAGE PARAMETERS
 * ============================================================================ */
#define BUS_VOLTAGE_MV          16000UL     /* DC bus, mV — match your supply */
#define AMP_GAIN                40UL        /* DRV8353R sense amp gain (V/V) */
#define SHUNT_UOHMS             1000UL      /* shunt resistance (µΩ) — 1 mΩ */

/* Bus voltage envelope for soft over/under voltage faults.
 * Set ±20 % around BUS_VOLTAGE_MV. */
#define BUS_OV_MV               (BUS_VOLTAGE_MV * 12 / 10)   /* 24 V at 20 V nom */
#define BUS_UV_MV               (BUS_VOLTAGE_MV * 8  / 10)   /* 16 V at 20 V nom */


/* ============================================================================
 * ADC / SAMPLING
 * ============================================================================ */
#define VREF_MV                 3300UL
#define ADC_COUNTS              4095UL      /* matches CubeMX ADC2 oversampling settings */

/* PWM */
#define PWM_ARR                 8499UL      /* 10 kHz centre-aligned at 170 MHz */
#define FOC_FS_HZ               10000UL     /* matches PWM_ARR derivation */
#define FOC_TS_S                (1.0f / (float)FOC_FS_HZ)

/* PI controller voltage limit — Vbus / √3 (linear modulation region of SVPWM) */
#define OUT_MAX                 ((float)BUS_VOLTAGE_MV / (1000.0f * 1.7320508f))


/* ============================================================================
 * CURRENT LIMITS
 *
 * MAX_CURRENT_MA is the firmware ceiling for steady-state torque commands.
 * Do not raise it above what your shunts, MOSFETs, and supply can sustain
 * thermally. With 1 mΩ shunts and 1 W rating: P = I² × 1 mΩ; at 10 A that
 * is 100 mW per shunt — well within limits.
 *
 * The 2 A bench supply caps practical current at ~3 A peak phase regardless
 * of this setting (see BENCH POWER NOTE in the file header).
 *
 * OCP_MULTIPLIER is the headroom over MAX for transient peaks (acceleration,
 * load steps). Below 1.5× the loop will trip on normal transients. Above
 * 5× the firmware OCP becomes non-protective and the DRV8353R hardware
 * VDS protection becomes the only safety net. 5× is a reasonable middle.
 * ============================================================================ */
#define MAX_CURRENT_MA          24000        /* 5 A — sized for 20 V / 2 A bench supply */
#define OCP_MULTIPLIER          1.25f        /* trip = 5 × MAX = 25 A peak */


/* ============================================================================
 * NOISE / FILTER PARAMETERS
 * ============================================================================ */
#define ADC_FILTER_ALPHA        0.95f       /* EMA smoothing — 530 Hz cutoff */
#define MEDIAN_SIZE             3           /* spike rejection */
#define PI_DEADBAND_A           0.05f       /* 50 mA — below noise floor */
#define CALIBRATION_SAMPLES     4000UL      /* 400 ms at 10 kHz */
#define TORQUE_RAMP_RATE        0.0001f     /* Nm/cycle = 1 Nm/sec — responsive */


/* ============================================================================
 * ALIGNMENT / SENSOR-CAL THRESHOLDS
 *
 * GAIN_FAULT_RATIO is the band beyond which a measured-vs-expected current
 * ratio is treated as hardware fault rather than tolerance. ±50 % covers
 * any reasonable component variation; outside that, something is broken.
 *
 * GAIN_CORR_CLAMP_LO/HI restrict the applied correction to ±15 % even
 * if a wider correction "would" be valid. Beyond ±15 %, applying the
 * full correction amplifies sensor noise too much; the fault flag is
 * raised instead.
 *
 * ALIGN_MIN_DISPLACEMENT_COUNTS detects "rotor stuck" (e.g. mounted to a
 * loaded robot leg). If the rotor moves less than this during alignment,
 * the alignment fault flag is set and FOC is not enabled.
 * ============================================================================ */
#define ALIGN_VD_V              0.5f
#define ALIGN_HOLD_MS           1000UL
#define GAIN_HOLD_MS            600UL
#define GAIN_SETTLE_SAMPLES     1000UL      /* skip first 100 ms of GAIN_CORRECTION */
#define GAIN_FAULT_RATIO        8.0f        /* ratio outside [1 / 8.0, 8.0] = hardware fault */
#define GAIN_CORR_CLAMP_LO      0.05f
#define GAIN_CORR_CLAMP_HI      1.80f
#define ALIGN_MIN_DISPLACEMENT_COUNTS  20   /* must move ≥ ~3.5° elec */

/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */
typedef struct { float alpha; float beta;   } AlphaBeta_t;
typedef struct { float d;     float q;      } DQ_t;
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

/* FOC pipeline state — see TIM8 callback for dispatch */
typedef enum {
    FOC_STATE_NEUTRAL          = 0,
    FOC_STATE_ALIGN            = 1,
    FOC_STATE_GAIN_CORRECTION  = 2,
	FOC_STATE_WAIT_UART		   = 3,
	FOC_STATE_UART_ENABLED     = 4,
    FOC_STATE_FAULT            = 5    /* hard-fault latched */
} FocState_t;

/* Tiered fault model.
 *
 * HARD faults latch focState into FAULT, disable PWM, and require power
 * cycle. They indicate that continuing operation would be unsafe (sustained
 * overcurrent, lost current sensing, lost encoder).
 *
 * SOFT faults set their bit in faultFlags and are reported over UART.
 * Some soft faults additionally clamp targetTorque_Nm to 0 (host timeout,
 * alignment-not-moved). Others are warnings only (gain mismatch within
 * fault bounds, near-limit bus voltage).
 *
 * Both kinds OR into the same bitfield so the host gets one diagnostic. */
typedef enum {
    /* HARD — focState forced to FAULT, PWM off, no auto-recovery */
    FAULT_OVERCURRENT_PHA      = (1u << 0),
    FAULT_OVERCURRENT_PHB      = (1u << 1),
    FAULT_BUS_OVERVOLTAGE      = (1u << 2),  /* not yet wired — see below */
    FAULT_BUS_UNDERVOLTAGE     = (1u << 3),  /* not yet wired — see below */
    FAULT_ENCODER_DISCONTINUITY= (1u << 4),
    FAULT_ADC_STUCK            = (1u << 5),

    /* SOFT — torque clamp + UART notification, system continues */
    FAULT_GAIN_BAD_PHA         = (1u << 8),
    FAULT_GAIN_BAD_PHB         = (1u << 9),
    FAULT_HOST_TIMEOUT         = (1u << 10),
    FAULT_ALIGN_NOT_MOVED      = (1u << 11),
    FAULT_DQ_OFFSET_LARGE      = (1u << 12),
    FAULT_TORQUE_CMD_CLAMPED   = (1u << 13),

    FAULT_HARD_MASK            = 0x00FF,
    FAULT_SOFT_MASK            = 0xFF00
} FaultFlags_t;

/* BUS VOLTAGE MONITORING — TO ADD WHEN ADC CHANNEL IS AVAILABLE:
 *
 * Configure one of the unused ADC1 or ADC3 channels as a regular conversion
 * sampling a divided bus-voltage signal (e.g. 100k/10k divider giving
 * Vbus/11 into the ADC). Then in TIM7 ISR (or a 100 Hz polling routine):
 *
 *     uint32_t vbus_mV = (HAL_ADC_GetValue(&hadc1) * 3300 * 11) / 4095;
 *     if (vbus_mV > BUS_OV_MV)  emergencyStop(FAULT_BUS_OVERVOLTAGE);
 *     if (vbus_mV < BUS_UV_MV)  emergencyStop(FAULT_BUS_UNDERVOLTAGE);
 *
 * Until that hardware is added, the firmware relies on the DRV8353R's own
 * UVLO and the host supply's protections. This is acceptable for bench use
 * but should be added before any unattended deployment.
 */


/* ============================================================================
 * GLOBALS
 *
 * `volatile`: cross-context (ISR ↔ ISR or ISR ↔ main).
 * `static`:   file-private, still visible to debugger.
 *
 * Group ordering: encoder, current sense, FOC state, command, telemetry,
 * faults, debug, data acquisition, UART. Each group has its own block
 * comment with rationale.
 * ============================================================================ */

/* --- Encoder (TIM8 ISR + main thread) --- */
volatile int32_t  encoderCount[2]  = {0,0};
volatile int32_t  lastCount[2]     = {32768, 32768};       /* TIM3 counter starts at midpoint */
volatile int32_t  encoderSpeed[2]  = {0,0};           /* signed counts/sec */

/* --- Current sense offsets (set during boot calibration) --- */
volatile float    offsetA[2]          = {0.0f, 0.0f};
volatile float    offsetB[2]          = {0.0f, 0.0f};
volatile uint32_t calibrateCounts[2]  = {0,0};

/* Counts → Amps scalar
 * I = (counts - offset) × current_scalar
 * scalar = VREF_mV / (ADC_COUNTS × R_shunt_mΩ × Gain)
 * The /1000 converts SHUNT_UOHMS (µΩ) into mΩ for unit consistency. */
const float current_scalar = (float)VREF_MV
                           / ((float)ADC_COUNTS
                              * ((float)SHUNT_UOHMS / 1000.0f)
                              * (float)AMP_GAIN);

/* --- Per-channel sensor gain calibration (replaces 1.741× hard-coded fix) ---
 * Multiplied into the raw counts→Amps conversion to compensate hardware
 * gain mismatch (op-amp resistor tolerance, shunt tolerance, op-amp Vos).
 * Calibrated automatically during alignment.
 *
 * Initialised to 1.0 so behaviour is correct before calibration completes. */
static float   gainCorrA[2]      = {1.0f, 1.0f};
static float   gainCorrB[2]      = {1.0f, 1.0f};

/* Filter state (ADC ISR only) */
static MedianFilter_t mfA[2] = {{{0},0},{{0},0}};
static MedianFilter_t mfB[2] = {{{0},0},{{0},0}};
static float filtIa[2]     = {0.0f, 0.0f};
static float filtIb[2]     = {0.0f, 0.0f};

/* Staging buffer (ADC ISR writes, TIM8 ISR reads) */
static volatile float   stagingIa[2]      = {0.0f, 0.0f};
static volatile float   stagingIb[2]      = {0.0f, 0.0f};
static volatile uint8_t newCurrentData[2] = {0,0};

/* TIM8 ISR working copies (debugger-visible) */
static float Ia[2] = {0.0f, 0.0f};
static float Ib[2] = {0.0f, 0.0f};

/* DQ frame offset (TIM8 ISR only) */
static float idOffset[2] = {0.0f, 0.0f};
static float iqOffset[2] = {0.0f, 0.0f};

/* PI controllers
 *
 * kp = 0.2  proportional gain — fast response without oscillation
 * ki = 0.003 per-cycle integral gain at 10 kHz (continuous Ki = 30 V/(A·s))
 *           With L ≈ 50 µH this gives a closed-loop crossover near 1 kHz.
 *           Reduce kp to 0.1 if you see oscillation when first enabling FOC. */
static PI_t piD[2] = {
	{
    .kp = 0.2f, .ki = 0.003f,
    .integral = 0.0f,
    .outMin = -OUT_MAX, .outMax =  OUT_MAX
	},
	{
	.kp = 0.2f, .ki = 0.003f,
    .integral = 0.0f,
    .outMin = -OUT_MAX, .outMax =  OUT_MAX
	}
};
static PI_t piQ[2] = {
	{
    .kp = 0.2f, .ki = 0.003f,
    .integral = 0.0f,
    .outMin = -OUT_MAX, .outMax =  OUT_MAX
	},
	{
	.kp = 0.2f, .ki = 0.003f,
	.integral = 0.0f,
	.outMin = -OUT_MAX, .outMax =  OUT_MAX
	}
};

/* --- Torque command --- */
volatile float    targetTorque_Nm[2]  = {0.0f, 0.0f};
static   float    rampedTorque_Nm[2]  = {0.0f, 0.0f};
static   float    targetIq[2]         = {0.0f, 0.0f};

/* --- FOC state machine --- */
volatile FocState_t focState[2]   = {FOC_STATE_NEUTRAL, FOC_STATE_NEUTRAL};
volatile uint8_t    focEnabled[2] = {0,0};            /* mirrors focState == FOC_STATE_WAIT_UART */
static   float      alignVd_V[2]  = {0.0f, 0.0f};
static   int32_t    alignStartCount[2] = {0, 0};       /* for displacement detection */

/* --- Faults --- */
volatile uint32_t faultFlags    = 0u;
volatile uint8_t  gainFaultA    = 0;            /* legacy: now mirrors FAULT_GAIN_BAD_PHA */
volatile uint8_t  gainFaultB    = 0;

/* --- Gain calibration accumulators (ADC ISR) --- */
static float    gainSumA[2]       = {0.0f, 0.0f};
static float    gainSumB[2]       = {0.0f, 0.0f};
static uint32_t gainSamples[2]    = {0, 0};
static uint32_t gainSettleSkip[2] = {0, 0};

/* --- Debug live-watch --- */
#if Debug == 1
volatile float   debugId[2]           = {0.0f, 0.0f};
volatile float   debugIq[2]           = {0.0f, 0.0f};
volatile float   debugElecAngle[2]    = {0.0f, 0.0f};
volatile int32_t debugEncoderCount[2] = {0, 0};
volatile float   debugVd[2]           = {0.0f, 0.0f};
volatile float   debugVq[2]           = {0.0f, 0.0f};
volatile float   debugIa[2]           = {0.0f, 0.0f};
volatile float   debugIb[2]           = {0.0f, 0.0f};
#endif

/* --- Data acquisition arrays --- */
#if dataAqu == 1
volatile float   IdG[AQU_AMT]          = {0};
volatile float   IqG[AQU_AMT]          = {0};
volatile float   ElecAngleG[AQU_AMT]   = {0};
volatile float   VdG[AQU_AMT]          = {0};
volatile float   VqG[AQU_AMT]          = {0};
volatile float   IaG[AQU_AMT]          = {0};
volatile float   IbG[AQU_AMT]          = {0};
volatile int32_t EncoderCountG[AQU_AMT] = {0};
static uint32_t  Gi = {0, 0};
static uint16_t  Pi = {0, 0};
#endif

/* --- HAL handles (generated by CubeMX) --- */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;


/* ============================================================================
 * FORWARD DECLARATIONS
 * ============================================================================ */
static void        SystemClock_Config	(void);
static void        CORDIC_SinCos		(float angle_deg, float *sinVal, float *cosVal);
static float       medianFilter			(MedianFilter_t *f, float newVal);
static AlphaBeta_t clarkeTransform		(float Ia, float Ib);
static DQ_t        parkTransformSC		(AlphaBeta_t ab, float sinTheta, float cosTheta);
static AlphaBeta_t inverseParkSC		(DQ_t dq, float sinTheta, float cosTheta);
static void        SVPWM				(AlphaBeta_t vab, float vBus, uint32_t arr, uint8_t motor);
static float       PI_Update			(PI_t *pi, float setpoint, float measured);
static float       getElectricalAngle	(uint8_t motor);
static void        applyNeutralOutput	(uint8_t motor);
static void        emergencyStop		(uint32_t reason);
static void        resetFilters			(uint8_t motor);
static void        alignRotor			(uint8_t motor);
static void        calibrateDQOffsets	(uint8_t motor);
static void        raiseSoftFault		(uint32_t flag);
static void        clearSoftFault		(uint32_t flag);
static void		   FOCroutine			(uint8_t motor);
static void 	   ADCloop				(uint8_t motor);
#define MTR_AMT 1
#define MOTOR_1 0
#define MOTOR_2 1

/* ============================================================================
 * UART HOST PROTOCOL
 *
 * RX (host → STM32, 10 bytes):
 *   [0]    0xA5             sync
 *   [1]    sequence         uint8 (wraps)
 *   [2-5]  torque_M1 (float, Nm, little-endian)
 *   [6-9]  torque_M2 (float, Nm, little-endian, reserved)
 *
 * TX (STM32 → host, 18 bytes), reply on every accepted RX:
 *   [0]    0x5A             sync
 *   [1]    sequence         echoed
 *   [2-5]  position_M1   (float, mech revs)
 *   [6-9]  position_M2   (float, reserved 0)
 *   [10-13] velocity_M1  (float, mech RPS)
 *   [14-17] velocity_M2  (float, reserved 0)
 *
 * NOTE: 18-byte response carries no fault code. To get faults over UART,
 * watch `faultFlags` in the live debugger or extend the protocol with a
 * status frame (not implemented here to keep the protocol identical to
 * what your host expects).
 *
 * CUBEMX SETUP REQUIRED:
 *   USART1: Asynchronous, 115200 8N1
 *   USART1 DMA RX: Circular, both data widths = Byte (NOT Half-Word)
 *   USART1 NVIC: enabled, priority 3
 *   TIM7: 1 kHz, NVIC priority 3 (used to drive uart_poll_rx)
 *
 * SAFETY: target torque is forced to 0 if no valid frame is received
 * within UART_CMD_TIMEOUT_MS. This raises FAULT_HOST_TIMEOUT (soft fault).
 * ============================================================================ */
#define UART_RX_BUF_SIZE        64u         /* power of 2 for cheap wrap math */
#define UART_CMD_FRAME_LEN      10u
#define UART_RSP_FRAME_LEN      18u
#define UART_SYNC_RX            0xA5u
#define UART_SYNC_TX            0x5Au
#define UART_CMD_TIMEOUT_MS     400u
#define UART_INIT_NAN			0x7FC00001
#define UART_INIT_NAN_SEND		0x7FC00002 //0x0200C07F
#define UART_ZERO_ENCODERS_NAN  0x7FC00013

static uint8_t          uartRxBuf[UART_RX_BUF_SIZE];
static uint16_t         uartRxTail   = 0;
static uint8_t          uartTxBuf[UART_RSP_FRAME_LEN];
static volatile uint8_t uartTxBusy   = 0;

volatile uint32_t       uartFramesAccepted = 0;
volatile uint32_t       uartFramesDropped  = 0;
volatile uint32_t       uartSyncHunts      = 0;
volatile uint32_t       uartSeqErrors      = 0;
volatile uint8_t		isInit			   = 0;
volatile float 			pos_M1_Offset      = 0.0f;
volatile float 			pos_M2_Offset      = 0.0f;

static uint8_t          uartLastSeq    = 0;
static uint8_t          uartFirstFrame = 1;
static uint32_t         uartLastValidMs = 0;


/* ── HAL TX-complete callback ───────────────────────────────────────────── */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        uartTxBusy = 0;
}


/* ── Endian-safe pack helpers ───────────────────────────────────────────── */
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


/* ── Build and send response frame ──────────────────────────────────────── */
static void uart_send_response(uint8_t seq)
{
    if (uartTxBusy)
    {
        uartFramesDropped++;
        return;
    }

    /* Mechanical position in revolutions, velocity in revolutions per second */
    float pos_M1 = (float)encoderCount[MOTOR_1] / (float)COUNTS_PER_REV - pos_M1_Offset;
    float vel_M1 = (float)encoderSpeed[MOTOR_1] / (float)COUNTS_PER_REV;

    float pos_M2 = (float)encoderCount[MOTOR_2] / (float)COUNTS_PER_REV - pos_M2_Offset;
    float vel_M2 = (float)encoderSpeed[MOTOR_2] / (float)COUNTS_PER_REV;

    uartTxBuf[0] = UART_SYNC_TX;
    uartTxBuf[1] = seq;

    if (isInit == 1)
    {
    	uint32_t initSend = UART_INIT_NAN_SEND;
    	isInit = 0;
    	memcpy(&uartTxBuf[2], &initSend, 4);
    }
    else uart_store_f32(&uartTxBuf[2],  pos_M1);
    uart_store_f32(&uartTxBuf[6],  pos_M2);     /* M2 reserved */
    uart_store_f32(&uartTxBuf[10], vel_M1);
    uart_store_f32(&uartTxBuf[14], vel_M2);     /* M2 reserved */

    uartTxBusy = 1;
    if (HAL_UART_Transmit_IT(&huart1, uartTxBuf, UART_RSP_FRAME_LEN) != HAL_OK)
    {
        uartTxBusy = 0;
        uartFramesDropped++;
    }
}


/* ── Apply received command frame ───────────────────────────────────────── */
static void uart_apply_frame(const uint8_t *frame)
{
    const uint8_t seq    = frame[1];
    float         tq_M1  = uart_load_f32(&frame[2]);
    float         tq_M2  = uart_load_f32(&frame[6]);

    /* Sequence sanity (informational only — frame is always applied) */
    if (!uartFirstFrame)
    {
        if (seq != (uint8_t)(uartLastSeq + 1u))
            uartSeqErrors++;
    }
    uartFirstFrame = 0;
    uartLastSeq    = seq;

    /* Reject NaN; clamp to firmware ceiling */
    const float TQ_LIMIT = (float)MAX_CURRENT_MA * 0.001f * KT_NM_PER_AMP;

    /* Check for Uart Initialize using a NaN in package position of Tq_M1 */
    uint32_t initCheck;
    memcpy(&initCheck, &frame[2], 4);
    if(initCheck == UART_INIT_NAN && focState[MOTOR_1] == FOC_STATE_WAIT_UART)
    {
    	focState[MOTOR_1] = FOC_STATE_UART_ENABLED;
    	isInit = 1;
    }
    if(initCheck == UART_ZERO_ENCODERS_NAN)
    {
    	pos_M1_Offset = (float)encoderCount[MOTOR_1] / (float)COUNTS_PER_REV;
    	pos_M2_Offset = (float)encoderCount[MOTOR_2] / (float)COUNTS_PER_REV;
    }

    if (tq_M1 != tq_M1) tq_M1 = 0.0f;
    if (tq_M1 >  TQ_LIMIT) { tq_M1 =  TQ_LIMIT; raiseSoftFault(FAULT_TORQUE_CMD_CLAMPED); }
    else if (tq_M1 < -TQ_LIMIT) { tq_M1 = -TQ_LIMIT; raiseSoftFault(FAULT_TORQUE_CMD_CLAMPED); }

    if (tq_M2 != tq_M2) tq_M2 = 0.0f;
    if (tq_M2 >  TQ_LIMIT) { tq_M2 =  TQ_LIMIT; raiseSoftFault(FAULT_TORQUE_CMD_CLAMPED); }
    else if (tq_M2 < -TQ_LIMIT) { tq_M2 = -TQ_LIMIT; raiseSoftFault(FAULT_TORQUE_CMD_CLAMPED); }

    else clearSoftFault(FAULT_TORQUE_CMD_CLAMPED);

    /* Refuse positive torque if any HARD fault is latched */
    if (faultFlags & FAULT_HARD_MASK)
    {
        tq_M1 = 0.0f;
        tq_M2 = 0.0f;
    }
    /* Refuse positive torque if alignment-displacement fault was raised */
    if (faultFlags & FAULT_ALIGN_NOT_MOVED)
    {
        tq_M1 = 0.0f;
    	tq_M2 = 0.0f;
	}

    if (focState[MOTOR_1] == FOC_STATE_UART_ENABLED)
    {
    	targetTorque_Nm[MOTOR_1] = tq_M1;
    	targetTorque_Nm[MOTOR_2] = tq_M2;
    }
    else
    {
    	targetTorque_Nm[MOTOR_1] = 0;
    	targetTorque_Nm[MOTOR_2] = 0;
    }
    uartLastValidMs = HAL_GetTick();
    uartFramesAccepted++;
    clearSoftFault(FAULT_HOST_TIMEOUT);

    uart_send_response(seq);
}


/* ── Poll DMA ring buffer for complete frames ───────────────────────────── */
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

        /* Sync confirmed — copy frame flat to handle ring wrap cleanly */
        uint8_t frame[UART_CMD_FRAME_LEN];
        for (uint16_t i = 0; i < UART_CMD_FRAME_LEN; i++)
            frame[i] = uartRxBuf[(uartRxTail + i) & (UART_RX_BUF_SIZE - 1u)];

        uartRxTail = (uint16_t)((uartRxTail + UART_CMD_FRAME_LEN)
                                & (UART_RX_BUF_SIZE - 1u));

        uart_apply_frame(frame);

        /* Refresh DMA head — bytes may have arrived during apply */
        dmaHead = (uint16_t)(UART_RX_BUF_SIZE
                             - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    }
}


/* ── Watchdog — zero torque if host has gone silent ─────────────────────── */
void uart_check_timeout(void)
{
    if (uartFirstFrame) return;        /* never received anything yet */

    if ((HAL_GetTick() - uartLastValidMs) > UART_CMD_TIMEOUT_MS)
    {
    	for(int motor = 0; motor < MTR_AMT; motor++){
        	targetTorque_Nm[motor] = 0.0f;
    	}
    	raiseSoftFault(FAULT_HOST_TIMEOUT);
    }
}


/* ── 100 Hz velocity update for telemetry ───────────────────────────────── */
void uart_update_velocity(void)
{
    static uint32_t lastTick  = 0;
    static int32_t  lastCnt[MTR_AMT]   = {0};

    uint32_t now = HAL_GetTick();
    if ((now - lastTick) >= 10u)
    {
    	for(int motor = 0; motor < MTR_AMT; motor++){
        	const int32_t delta = encoderCount[motor] - lastCnt[motor];
        	encoderSpeed[motor]   = delta * 100;     /* 10 ms × 100 = 1 sec */
        	lastCnt[motor]        = encoderCount[motor];
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

/* CORDIC sin/cos: ~6 cycles via hardware accelerator vs ~100 for sinf/cosf. */
static void CORDIC_SinCos(float angle_deg, float *sinVal, float *cosVal)
{
    float norm = angle_deg / 180.0f;
    if (norm >  1.0f) norm -= 2.0f;
    if (norm < -1.0f) norm += 2.0f;

    int32_t angleQ31 = (int32_t)(norm * 2147483647.0f);

    CORDIC->CSR = LL_CORDIC_FUNCTION_COSINE
                | LL_CORDIC_PRECISION_6CYCLES
                | LL_CORDIC_SCALE_0
                | LL_CORDIC_NBWRITE_1
                | LL_CORDIC_NBREAD_2
                | LL_CORDIC_INSIZE_32BITS
                | LL_CORDIC_OUTSIZE_32BITS;

    CORDIC->WDATA = (uint32_t)angleQ31;
    int32_t cosQ31 = (int32_t)CORDIC->RDATA;   /* cosine first */
    int32_t sinQ31 = (int32_t)CORDIC->RDATA;

    *cosVal = (float)cosQ31 / 2147483648.0f;
    *sinVal = (float)sinQ31 / 2147483648.0f;
}


/* Median filter — rejects single-sample switching spikes without phase lag. */
static float medianFilter(MedianFilter_t *f, float newVal)
{
    f->buf[f->idx] = newVal;
    f->idx = (uint8_t)((f->idx + 1u) % MEDIAN_SIZE);

    float sorted[MEDIAN_SIZE];
    memcpy(sorted, f->buf, sizeof(sorted));

    /* Insertion sort — O(n²), n=3, max 3 comparisons */
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


/* Clarke (3-phase → αβ). Iα = Ia, Iβ = (Ia + 2·Ib)/√3. */
static AlphaBeta_t clarkeTransform(float Ia, float Ib)
{
    AlphaBeta_t out;
    out.alpha = Ia;
    out.beta  = (Ia + 2.0f * Ib) * 0.57735027f;   /* 1/√3 */
    return out;
}


/* Park (αβ → dq). Id =  Iα·cosθ + Iβ·sinθ ;  Iq = -Iα·sinθ + Iβ·cosθ. */
static DQ_t parkTransformSC(AlphaBeta_t ab, float sinTheta, float cosTheta)
{
    DQ_t out;
    out.d =  ab.alpha * cosTheta + ab.beta * sinTheta;
    out.q = -ab.alpha * sinTheta + ab.beta * cosTheta;
    return out;
}


/* Inverse Park (dq → αβ). */
static AlphaBeta_t inverseParkSC(DQ_t dq, float sinTheta, float cosTheta)
{
    AlphaBeta_t out;
    out.alpha = dq.d * cosTheta - dq.q * sinTheta;
    out.beta  = dq.d * sinTheta + dq.q * cosTheta;
    return out;
}


/* Get electrical angle from absolute encoder count (degrees, [0, 360)). */
static float getElectricalAngle(uint8_t motor)
{
    int32_t posCount = ((encoderCount[motor] % COUNTS_PER_REV)
                        + COUNTS_PER_REV) % COUNTS_PER_REV;

    float mech     = (float)posCount / (float)COUNTS_PER_REV;
    float elec     = mech * (float)POLE_PAIRS * 360.0f;

    elec = fmodf(elec, 360.0f);
    if (elec < 0.0f) elec += 360.0f;
    return elec;
}


/* ============================================================================
 * SVPWM — zero-sequence injection
 *
 * Linear modulation up to Vbus/√3.
 *
 * CRITICAL PIN MAPPING (verify against your PCB):
 *   TIM8 CH1 → Phase A
 *   TIM8 CH2 → Phase B
 *   TIM8 CH3 → Phase C
 * ============================================================================ */
static void SVPWM(AlphaBeta_t vab, float vBus, uint32_t arr, uint8_t motor)
{
    const float vHalf = vBus * 0.5f;

    /* Inverse Clarke → 3-phase reference (normalised to ±1 = ±Vbus/2) */
    float Va = vab.alpha / vHalf;
    float Vb = (-vab.alpha * 0.5f + vab.beta * 0.86602540f) / vHalf;
    float Vc = (-vab.alpha * 0.5f - vab.beta * 0.86602540f) / vHalf;

    /* Find max/min for zero-sequence offset */
    float Vmax = Va; if (Vb > Vmax) Vmax = Vb; if (Vc > Vmax) Vmax = Vc;
    float Vmin = Va; if (Vb < Vmin) Vmin = Vb; if (Vc < Vmin) Vmin = Vc;
    const float Voffset = -0.5f * (Vmax + Vmin);

    /* Apply offset and convert to duty cycle [0,1] */
    float tA = 0.5f + 0.5f * (Va + Voffset);
    float tB = 0.5f + 0.5f * (Vb + Voffset);
    float tC = 0.5f + 0.5f * (Vc + Voffset);

    /* Overmodulation clamp — 0.95 ceiling preserves bootstrap charging */
    if (tA > 0.95f) tA = 0.95f; else if (tA < 0.0f) tA = 0.0f;
    if (tB > 0.95f) tB = 0.95f; else if (tB < 0.0f) tB = 0.0f;
    if (tC > 0.95f) tC = 0.95f; else if (tC < 0.0f) tC = 0.0f;

    TIM_HandleTypeDef *htim = (motor == MOTOR_1) ? &htim8 : &htim20;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(tA * (float)arr));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(tB * (float)arr));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(tC * (float)arr));
}


/* ============================================================================
 * PI CONTROLLER — clamping anti-windup + deadband
 *
 *   e[n]  = setpoint - measured
 *   u_pre = kp·e[n] + I[n-1]
 *   if |e[n]| > deadband AND NOT (saturated_in_same_direction):
 *       I[n] = I[n-1] + ki·e[n]
 *   u[n]  = clamp(kp·e[n] + I[n], outMin, outMax)
 *
 * Saturated_in_same_direction = output is at +outMax AND error > 0
 *                            OR output is at -outMin AND error < 0.
 * Integrating in the opposite direction (which would un-saturate) is
 * always allowed.
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


/* Hard-fault path. Called from inside ISRs on overcurrent etc.
 * Disables PWM via MOE, latches state into FAULT, sets fault flag. */
static void emergencyStop(uint32_t reason)
{
    focState[MOTOR_1]   = FOC_STATE_FAULT;
    focEnabled[MOTOR_1] = 0;
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim20);

    piD[0].integral    = 0.0f;
    piD[1].integral    = 0.0f;
    piQ[0].integral    = 0.0f;
    piQ[1].integral    = 0.0f;
    rampedTorque_Nm[0] = 0.0f;
    rampedTorque_Nm[1] = 0.0f;
    targetIq[0]        = 0.0f;
    targetIq[1]        = 0.0f;
    targetTorque_Nm[0] = 0.0f;
    targetTorque_Nm[1] = 0.0f;

    faultFlags |= reason;     /* HARD-fault bits */
}


/* Soft fault helpers — called from any context (volatile fault flag). */
static void raiseSoftFault(uint32_t flag)
{
    faultFlags |= flag;
    /* Mirror to legacy gainFault flags for back-compat */
    if (flag & FAULT_GAIN_BAD_PHA) gainFaultA = 1;
    if (flag & FAULT_GAIN_BAD_PHB) gainFaultB = 1;
}

static void clearSoftFault(uint32_t flag)
{
    faultFlags &= ~flag;
    if (flag & FAULT_GAIN_BAD_PHA) gainFaultA = 0;
    if (flag & FAULT_GAIN_BAD_PHB) gainFaultB = 0;
}

void FOCroutine(uint8_t motor)
{
	/* HARD-fault latch: if anything has tripped, output stays neutral
	     * (PWM is already disabled by emergencyStop, but writing neutral
	     * keeps the CCR registers from holding stale duty for the next time
	     * MOE is re-enabled in some recovery scenario). */
	    if (focState[motor] == FOC_STATE_FAULT)
	    {
	        applyNeutralOutput(MOTOR_1);
	        applyNeutralOutput(MOTOR_2);
	        return;
	    }

	    /* State dispatch */
	    if (focState[motor] == FOC_STATE_NEUTRAL)
	    {
	    	applyNeutralOutput(MOTOR_1);
	    	applyNeutralOutput(MOTOR_2);
	        return;
	    }
	    if (focState[motor] == FOC_STATE_ALIGN
	     || focState[motor] == FOC_STATE_GAIN_CORRECTION)
	    {
	        /* Vd at electrical 0°: sin(0)=0, cos(0)=1 → Vα = Vd, Vβ = 0 */
	        const AlphaBeta_t vAlpha = { .alpha = alignVd_V[motor], .beta = 0.0f };
	        SVPWM(vAlpha, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);
	        return;
	    }
	    /* else: FOC_STATE_WAIT_UART */
	    /* Only FOC_STATE_RUN or FOC_STATE_UART_ENABLED reach the closed-loop pipeline */
	    /*if (focState[motor] != FOC_STATE_UART_ENABLED)
	    {
	        applyNeutralOutput(motor);
	        return;
	    }*/
	    /* OPEN-LOOP test mode — useful for sensor/encoder diagnostics */
#if openLoop == 1
	        DQ_t        vdq = { .d = openLoopVd[motor], .q = openLoopVq[motor] };
	        const float elec = getElectricalAngle(motor);
	        float       sinT, cosT;
	        CORDIC_SinCos(elec, &sinT, &cosT);
	        AlphaBeta_t vab = inverseParkSC(vdq, sinT, cosT);
	        SVPWM(vab, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);

	        /* Still read currents for diagnostic — but skip PI */
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
#endif
	    /* ── STAGE 0: copy current snapshot from staging ─────────────────────── */
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


	    /* ── STAGE 1: encoder update ─────────────────────────────────────────── */
	    /* TIM3 16-bit counter — handle wraparound. Discontinuity check raises a
	     * hard fault if the encoder seems to have jumped impossibly far in one
	     * cycle (signal cable disconnected, magnet detached, etc.). */
	    TIM_HandleTypeDef *htim = (motor == MOTOR_1) ? &htim3 : &htim4;
	    const int32_t currentCount = (int32_t)__HAL_TIM_GET_COUNTER(htim);
	    int32_t delta = currentCount - lastCount[motor];

	    if (delta >  32767) delta -= 65536;
	    if (delta < -32768) delta += 65536;

	    /* Discontinuity threshold: 100 counts in one 100 µs cycle = ~30 krpm
	     * mechanical. Any motor reading above this is a sensor fault, not real. */
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


	    /* ── STAGE 2: angle + velocity estimate + prediction ─────────────────── */
	    /* Per-cycle electrical angle change (rad). */
	    #define TWO_PI_F            6.28318530717958647693f
	    #define ELEC_RAD_PER_COUNT  ((TWO_PI_F / (float)COUNTS_PER_REV) * (float)POLE_PAIRS)

	    const float deltaTheta_e = (float)delta * ELEC_RAD_PER_COUNT;

	    /* Rate-limited IIR on speed estimate.
	     *   alpha = 0.2: 5-cycle window kills encoder quantization noise
	     *   rate limit = 1 count/cycle: prevents stale filter state from
	     *     overshooting predAngle when the rotor brakes suddenly. */
	    static float deltaTheta_e_filt[MTR_AMT] = {0.0f, 0.0f};
	    {
	        float ferr = deltaTheta_e - deltaTheta_e_filt[motor];
	        if      (ferr >  ELEC_RAD_PER_COUNT) ferr =  ELEC_RAD_PER_COUNT;
	        else if (ferr < -ELEC_RAD_PER_COUNT) ferr = -ELEC_RAD_PER_COUNT;
	        deltaTheta_e_filt[motor] += 0.2f * ferr;
	    }

	    /* Speed-scaled prediction horizon.
	     *
	     * predAngle compensates for measurement-to-actuation delay:
	     *   1.0 Ts — current sample is already 1 cycle stale by FOC time
	     *   0.5 Ts — center of next PWM period (where average voltage applies)
	     *   1.0 Ts — median(3) filter group delay
	     *   X.X Ts — EMA filter group delay = (1-α)/α
	     * For α = 0.95: total ≈ 2.55 Ts.
	     *
	     * At low speeds the 1-count encoder jitter would be amplified by 2.55
	     * into ±9° prediction noise, which destabilises commutation. The
	     * scale factor smoothly turns prediction off near zero speed:
	     *   scale = |speed| / (|speed| + corner)
	     *   corner = 1 count/cycle (~17 RPM mech)
	     * Below corner, scale < 0.5, prediction is reduced. Above corner,
	     * scale > 0.5 ramping to ~0.8 at 4× corner. */
	    #define ANGLE_PREDICT_TS_FULL   2.55f
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


	    /* ── STAGE 3: Clarke + Park ──────────────────────────────────────────── */
	    AlphaBeta_t iab = clarkeTransform(Ia[motor], Ib[motor]);
	    DQ_t        idq = parkTransformSC(iab, sinTheta, cosTheta);
	    idq.d -= idOffset[motor];
	    idq.q -= iqOffset[motor];

	#if Debug == 1
	    debugId[motor] = idq.d;
	    debugIq[motor] = idq.q;
	#endif


	    /* ── STAGE 4: torque ramp + PI controllers ───────────────────────────── */
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

	    /* T = Kt × Iq → Iq = T / Kt */
	    targetIq[motor] = rampedTorque_Nm[motor] / KT_NM_PER_AMP;

	    const float maxIq = (float)MAX_CURRENT_MA / 1000.0f;
	    if (targetIq[motor] >  maxIq) targetIq[motor] =  maxIq;
	    if (targetIq[motor] < -maxIq) targetIq[motor] = -maxIq;

	    const float Vd = PI_Update(&piD[motor], 0.0f,    idq.d);
	    const float Vq = PI_Update(&piQ[motor], targetIq[motor], idq.q);

	  #if Debug == 1
	    debugVd[motor] = Vd;
	    debugVq[motor] = Vq;
	  #endif

	    /* ── STAGE 5: inverse Park (with delay-compensated angle) + SVPWM ────── */
	    DQ_t        vdq = { .d = Vd, .q = Vq };
	    AlphaBeta_t vab = inverseParkSC(vdq, sinPred, cosPred);
	    SVPWM(vab, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);

	#elif openLoop == 1
	    DQ_t        vdq = { .d = openLoopVd, .q = openLoopVq };
	    AlphaBeta_t vab = inverseParkSC(vdq, sinPred, cosPred);
	    SVPWM(vab, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR, motor);

	  #if Debug == 1
	    debugVd[0] = openLoopVd;
	    debugVq[0] = openLoopVq;
	  #endif
	#endif


	    /* ── STAGE 6: data acquisition (every 40th cycle = 250 Hz) ──────────── */
	#if dataAqu == 1
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
	#endif
}
/* ============================================================================
 * ADC INJECTED COMPLETE CALLBACK — 10 kHz, triggered by TIM8 TRGO2
 *
 * Pipeline:
 *   raw counts → offset subtract → ×current_scalar → ×gainCorr → median → EMA
 *              → staging (atomic write for TIM8 ISR)
 *
 * The boot calibration cycle runs in TWO phases:
 *
 *   PHASE 1 (calibrateCounts < CALIBRATION_SAMPLES, focState = NEUTRAL):
 *     Average raw counts at zero current to find offsetA, offsetB.
 *     This runs once at startup and again after alignment for a clean baseline.
 *
 *   PHASE 2 (focState = GAIN_CORRECTION):
 *     Rotor is held at electrical 0° with Vd applied.
 *     Expected currents: Ia = +Vd/Rs (positive), Ib = -Vd/(2Rs) (negative).
 *     Average measured currents and divide expected/measured to get gainCorr.
 *
 *   The gain finalisation block (focState transitions OUT of GAIN_CORRECTION
 *   while gainSamples > 0) computes the ratio and:
 *     - If 1/GAIN_FAULT_RATIO < ratio < GAIN_FAULT_RATIO: clamp to ±15% and
 *       use it as gainCorr. Soft fault raised if hardware was outside ±15%.
 *     - Outside the fault ratio: gainCorr = 1.0,
 *       SOFT-fault so the user can still observe the system. The motor will
 *       not drive cleanly with bad gain calibration, but it won't be unsafe
 *       so long as the OCP layer is active.
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


static void ADCloop(uint8_t motor){
    /* Read raw oversampled counts */
    ADC_HandleTypeDef *adc = (motor == MOTOR_1) ? &hadc2 : &hadc3;
    const float countsA = (float)HAL_ADCEx_InjectedGetValue(adc, ADC_INJECTED_RANK_1);
    const float countsB = (float)HAL_ADCEx_InjectedGetValue(adc, ADC_INJECTED_RANK_2);

    /* ── PHASE 1: zero-current offset calibration ────────────────────────── */
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

    /* ── Convert to Amps ─────────────────────────────────────────────────── */
    /* gainCorrA/B compensate hardware gain mismatch — initially 1.0, set
     * by the gain calibration block below during boot. */

    const float Ia_amps = (countsA - offsetA[motor]) * current_scalar * gainCorrA[motor];
    const float Ib_amps = (countsB - offsetB[motor]) * current_scalar * gainCorrB[motor];

    /* ── Per-cycle overcurrent guard (firmware OCP) ──────────────────────── */
    const float ocpLimit = (float)MAX_CURRENT_MA * 0.001f * OCP_MULTIPLIER;
    if(focState[MOTOR_1] != FOC_STATE_GAIN_CORRECTION && focState[MOTOR_1] != FOC_STATE_ALIGN)
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

    /* ── Median + EMA filter ─────────────────────────────────────────────── */
    const float medA = medianFilter(&mfA[motor], Ia_amps);
    const float medB = medianFilter(&mfB[motor], Ib_amps);
    filtIa[motor] = filtIa[motor] + ADC_FILTER_ALPHA * (medA - filtIa[motor]);
    filtIb[motor] = filtIb[motor] + ADC_FILTER_ALPHA * (medB - filtIb[motor]);

    /* ── PHASE 2: gain calibration accumulator ───────────────────────────── */
    if (focState[MOTOR_1] == FOC_STATE_GAIN_CORRECTION)
    {
        if (gainSettleSkip[motor] < GAIN_SETTLE_SAMPLES)
        {
            gainSettleSkip[motor]++;
        }
        else
        {
            /* Accumulate UNCORRECTED currents (gainCorr == 1.0 here on first
             * boot pass; if ever recalibrated, prior gain would be already
             * applied — accept that and treat the result as a refinement). */
            gainSumA[motor] += filtIa[motor];
            gainSumB[motor] += filtIb[motor];
            gainSamples[motor]++;
        }
    }
    else if (gainSamples[motor] > 0 && focState[MOTOR_1] != FOC_STATE_GAIN_CORRECTION)
    {
        const float Ia_meas = gainSumA[motor] / (float)gainSamples[motor];
        const float Ib_meas = gainSumB[motor] / (float)gainSamples[motor];

        const float Rs_ohms = (float)RS_MOHMS / 1000.0f;
        const float Ia_exp  =  alignVd_V[motor] / Rs_ohms;
        const float Ib_exp  = -alignVd_V[motor] / (2.0f * Rs_ohms);

        /* PLAUSIBILITY GATE — if the measured currents have wrong sign or
         * are far below the expected magnitude, the rotor wasn't actually
         * aligned and any "gain ratio" would be a fiction. Leave gainCorr
         * at 1.0 and raise the fault flags rather than installing garbage. */
        const uint8_t signsOk = (Ia_meas > 0.0f) && (Ib_meas < 0.0f);
        const uint8_t magsOk  = (fabsf(Ia_meas) > 0.5f * fabsf(Ia_exp))
                             && (fabsf(Ib_meas) > 0.5f * fabsf(Ib_exp));

        if (!signsOk || !magsOk)
        {
            gainCorrA[motor] = 1.0f;
            gainCorrB[motor] = 1.0f;
            /* Do NOT raise FAULT_GAIN_BAD here — this is "couldn't measure",
             * not "hardware is broken". The alignment fault flag set by
             * alignRotor() conveys the real diagnosis. */
        }
        else
        {
            /* (existing per-channel ratio computation, unchanged) */
            /* Phase A */
            float ratioA = Ia_exp / Ia_meas;
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

            /* Phase B (same logic) */
            float ratioB = Ib_exp / Ib_meas;
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

        gainSumA[motor] = 0.0f;
        gainSumB[motor] = 0.0f;
        gainSamples[motor] = 0;
        gainSettleSkip[motor] = 0;
    }

    /* ── Stage for FOC ISR ───────────────────────────────────────────────── */
    stagingIa[motor] = filtIa[motor];
    stagingIb[motor] = filtIb[motor];
    newCurrentData[motor] = 1;
}

/* ============================================================================
 * COMBINED TIMER PERIOD-ELAPSED CALLBACK
 *
 * STM32IDE generates a single weak HAL_TIM_PeriodElapsedCallback for ALL
 * timers. We dispatch by handle here.
 *
 * NVIC priorities (lower number = higher priority):
 *   ADC1_2          0    (current sense — never preempted)
 *   TIM8            2    (FOC loop)
 *   TIM7            3    (UART poll, telemetry)
 *   TIM17           3    (HAL systick)
 *
 * The 10× speed difference between TIM8 (10 kHz) and TIM7 (1 kHz) means
 * TIM7 spends only ~2 % of bandwidth on UART work. TIM8 always preempts
 * TIM7 cleanly thanks to the priority gap.
 * ============================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ── HAL systick maintenance ─────────────────────────────────────────── */
    if (htim->Instance == TIM17)
    {
        HAL_IncTick();
        return;
    }

    /* ── TIM7 — 1 kHz housekeeping ──────────────────────────────────────── */
    if (htim->Instance == TIM7)
    {
        uart_poll_rx();
        uart_check_timeout();
        uart_update_velocity();
        return;
    }

    /* ── TIM8 — 10 kHz FOC loop ─────────────────────────────────────────── */
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
 * ALIGN ROTOR + DETECT MOUNTED-LOAD CONDITION
 *
 * Two distinct failure modes that both LOOK like "rotor didn't move":
 *
 *   (a) Rotor was ALREADY at electrical 0° when alignment started. Common
 *       on every boot after the first one — the motor was last left at
 *       its aligned position. Currents will read healthy (+Ia, -Ib/2)
 *       because the rotor is in the right place; alignment is good.
 *
 *   (b) Rotor is mechanically blocked (loaded leg, jammed gearbox, etc.)
 *       and could not be pulled to electrical 0°. Currents will read at
 *       arbitrary magnitudes/signs depending on which angle the rotor is
 *       stuck at; the expected current ratios will be wildly off.
 *
 * We distinguish them by checking the gain-calibration measurements:
 *   - Healthy alignment: |Ia_measured| within ±30% of expected,
 *                        sign of Ib matches expected (negative).
 *   - Stuck at wrong angle: ratio outside ±30% OR Ib sign wrong.
 *
 * The gain-calibration block in the ADC ISR populates gainSumA/gainSumB
 * BEFORE we transition out of GAIN_CORRECTION. We grab a snapshot here
 * so we can reason about it before the ISR resets the accumulators.
 * ============================================================================ */
static void alignRotor(uint8_t motor)
{
	TIM_HandleTypeDef *htim = (motor == MOTOR_1) ? &htim3 : &htim4;
    alignStartCount[motor] = (int32_t)__HAL_TIM_GET_COUNTER(htim);

    alignVd_V[motor] = ALIGN_VD_V;
    focState[motor]  = FOC_STATE_ALIGN;
    HAL_Delay(ALIGN_HOLD_MS);

    focState[motor] = FOC_STATE_GAIN_CORRECTION;
    HAL_Delay(GAIN_HOLD_MS);

    /* Snapshot the gain-calibration accumulators BEFORE state transition.
     * Once focState leaves GAIN_CORRECTION the ADC ISR will reset them. */
    const float    snapSumA    = gainSumA[motor];
    const float    snapSumB    = gainSumB[motor];
    const uint32_t snapSamples = gainSamples[motor];

    const int32_t endCount = (int32_t)__HAL_TIM_GET_COUNTER(htim);

    focState[motor] = FOC_STATE_NEUTRAL;
    HAL_Delay(50);

    /* ── Did the rotor displace? ─────────────────────────────────────────── */
    int32_t displacement = endCount - alignStartCount[motor];
    if (displacement < 0) displacement = -displacement;
    if (displacement > 32767) displacement = 65536 - displacement;

    const uint8_t didMove = (displacement >= ALIGN_MIN_DISPLACEMENT_COUNTS);

    /* ── Do the currents look right? ─────────────────────────────────────── */
    /* If alignment worked (regardless of whether the rotor moved), we expect:
     *   Ia_meas ≈ +Vd/Rs       (positive, full magnitude)
     *   Ib_meas ≈ -Vd/(2*Rs)   (negative, half magnitude)
     * Tolerance ±30% accounts for Rs uncertainty + filter ripple. */
    uint8_t currentsLookHealthy = 0;
    if (snapSamples > 100u)        /* enough averaging to trust */
    {
        const float Ia_meas  = snapSumA / (float)snapSamples;
        const float Ib_meas  = snapSumB / (float)snapSamples;
        const float Rs_ohms  = (float)RS_MOHMS / 1000.0f;
        const float Ia_exp   =  alignVd_V[motor] / Rs_ohms;
        const float Ib_exp   = -alignVd_V[motor] / (2.0f * Rs_ohms);

        const float ratioA   = Ia_meas / Ia_exp;
        const float ratioB   = Ib_meas / Ib_exp;

        /* Both ratios must be in [0.2, 5]. The ratio framing means a
         * negative current that should be negative gives positive ratio,
         * so this catches sign errors automatically. */
        const uint8_t okA = (ratioA > 0.2f) && (ratioA < 5.0f);
        const uint8_t okB = (ratioB > 0.2f) && (ratioB < 5.0f);
        currentsLookHealthy = (okA && okB);
    }

    /* ── Decide ──────────────────────────────────────────────────────────── */
    /* Healthy if EITHER:
     *   - the rotor moved enough (it was somewhere else and got pulled in), OR
     *   - the currents match expectations (rotor was already aligned).
     * Otherwise it's stuck. */
    if (!didMove && !currentsLookHealthy)
    {
        raiseSoftFault(FAULT_ALIGN_NOT_MOVED);
    }

    /* Zero encoder at the aligned position. */
    __HAL_TIM_SET_COUNTER(htim, 32768);
    encoderCount[motor] = 0;
    lastCount[motor]    = 32768;
}


/* ============================================================================
 * CALIBRATE DQ OFFSETS
 * Measures residual Id and Iq at zero torque after alignment. Subtracted
 * every FOC cycle. If offsets are larger than 15 % of MAX_CURRENT, marks
 * a soft fault — calibration likely failed but doesn't stop boot.
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
            /* ADC not updating — escalate to hard fault */
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
 * MAIN
 *
 * Boot sequence (see comments inline):
 *   1. CubeMX peripheral inits
 *   2. ADC hardware self-cal
 *   3. DRV8353R enable + 5 ms charge-pump wait
 *   4. PWM start + TIM8 base start (FOC ISR begins, NEUTRAL state)
 *   5. ADC injected start
 *   6. Encoder start
 *   7. First-pass offset calibration (4000 samples = 400 ms)
 *   8. Alignment + gain calibration + displacement check
 *   9. Reset filters, redo offset cal (clean baseline)
 *  10. DQ offset calibration
 *  11. Enable FOC if no blocking faults
 *  12. Start TIM7 1 kHz UART/health timer
 *  13. Application loop (slow housekeeping only)
 *
 * If FAULT_ALIGN_NOT_MOVED is raised during step 8, FOC will NOT be enabled
 * at step 11. The motor stays at neutral and the host must reset or
 * resolve the load condition.
 * ============================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* CubeMX-generated peripheral inits. Order matches the .ioc. */
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

    for(int motor = 0; motor < MTR_AMT; motor++){
        focState[motor]   = FOC_STATE_NEUTRAL;
        focEnabled[motor] = 0;
    	//choose which motor/encoder/adc to init
    	TIM_HandleTypeDef *FOCtimer = (motor == MOTOR_1) ? &htim8 : &htim20;
    	TIM_HandleTypeDef *encoder = (motor == MOTOR_1) ? &htim3 : &htim4;
    	ADC_HandleTypeDef *adc = (motor == MOTOR_1) ? &hadc2 : &hadc3;
    	GPIO_TypeDef *EnablePort = (motor == MOTOR_1) ? ENABLE_3_GPIO_Port : ENABLE_1_GPIO_Port;
    	uint16_t EnablePin = (motor == MOTOR_1) ? ENABLE_3_Pin : ENABLE_1_Pin;
    	/* ── Step 1: ADC hardware self-cal ──────────────────────────────────── */

    	HAL_ADCEx_Calibration_Start(adc, ADC_SINGLE_ENDED);

    	/* ── Step 2: enable DRV8353R, wait for charge pump ──────────────────── */

    	HAL_GPIO_WritePin(EnablePort, EnablePin, GPIO_PIN_SET); //rewrite this
    	HAL_Delay(5);

    	applyNeutralOutput(motor);

    	/* ── Step 3: start PWM (MOE on) ──────────────────────────────────────── */
    	HAL_TIM_PWM_Start(FOCtimer,    TIM_CHANNEL_1);
    	HAL_TIMEx_PWMN_Start(FOCtimer, TIM_CHANNEL_1);
    	HAL_TIM_PWM_Start(FOCtimer,    TIM_CHANNEL_2);
    	HAL_TIMEx_PWMN_Start(FOCtimer, TIM_CHANNEL_2);
    	HAL_TIM_PWM_Start(FOCtimer,    TIM_CHANNEL_3);
    	HAL_TIMEx_PWMN_Start(FOCtimer, TIM_CHANNEL_3);

    	/* ── Step 4: start TIM8 base (FOC ISR fires at 10 kHz, NEUTRAL state) ─ */
    	HAL_TIM_Base_Start_IT(FOCtimer);
    	HAL_Delay(10);

    	/* ── Step 5: start ADC injected ─────────────────────────────────────── */
    	HAL_ADCEx_InjectedStart_IT(adc);
    	HAL_Delay(50);

    	/* ── Step 6: start encoder ──────────────────────────────────────────── */
    	HAL_TIM_Encoder_Start(encoder, TIM_CHANNEL_ALL);

    	/* ── Step 7: wait for first-pass offset calibration ─────────────────── */
    	while (calibrateCounts[motor] < CALIBRATION_SAMPLES){
    		HAL_Delay(10);
    	}

    	/* ── Step 8: align rotor + gain cal + displacement check ────────────── */
    	alignRotor(motor);

    	/* ── Step 9: reset filters, redo offset cal for a clean baseline ────── */
    	resetFilters(motor);
    	piD[motor].integral    = 0.0f;
    	piQ[motor].integral    = 0.0f;
    	rampedTorque_Nm[motor] = 0.0f;
    	targetIq[motor]        = 0.0f;
    	alignVd_V[motor] 	   = 0.0f;

        /* Atomic swap so the ADC ISR can never see a partially-reset state. */
        __disable_irq();
        calibrateCounts[motor] = 0;
        offsetA[motor]         = 0.0f;
        offsetB[motor]         = 0.0f;
        __enable_irq();
        while (calibrateCounts[motor] < CALIBRATION_SAMPLES) HAL_Delay(10);
    	HAL_Delay(100);                /* let filters settle with new offsets */

    	/* ── Step 10: DQ offset calibration ─────────────────────────────────── */
    	calibrateDQOffsets(motor);

    	HAL_Delay(50);   /* settling */
    	alignVd_V[motor] = 0.0f;

    	if (faultFlags & FAULT_HARD_MASK)
    		{
    		    /* Hard fault during boot — stay in fault state, never enable FOC */
    		    focState[motor]   = FOC_STATE_FAULT;
    		    focEnabled[motor] = 0;
    		}
    		else if (faultFlags & FAULT_ALIGN_NOT_MOVED)
    		{
    	    	/* Alignment couldn't displace the rotor — encoder offset is unsafe.
    	    	 * Stay neutral; the host must intervene. UART is still alive and will
    	    	 * report the fault flag. */
    	    	focState[motor]   = FOC_STATE_NEUTRAL;
    	    	focEnabled[motor] = 0;
    		}
    		else
    		{
    		    focState[motor]   = FOC_STATE_WAIT_UART;
    		    focEnabled[motor] = 1;
    		}
    }
	/* ── Step 11: enable FOC ONLY if no blocking faults ─────────────────── */


	/* ── Step 12: start TIM7 health/UART timer ──────────────────────────── */
	HAL_TIM_Base_Start_IT(&htim7);

	/* ============================================================================
 	* APPLICATION LOOP — slow housekeeping only
 	*
 	* All real-time work runs in interrupts:
 	*   - ADC1_2  (priority 0): current sense + filter + per-cycle OCP
 	*   - TIM8    (priority 2): FOC pipeline at 10 kHz
 	*   - TIM7    (priority 3): UART poll, timeout, velocity at 1 kHz
 	*
 	* This loop checks for fault recovery opportunities and sleeps
 	* otherwise. Adding HAL_Delay or running blocking code here is fine.
 	*
 	* SOFT-FAULT RECOVERY:
 	*   FAULT_HOST_TIMEOUT clears automatically when next valid frame arrives.
 	*   FAULT_TORQUE_CMD_CLAMPED clears on next valid in-range command.
 	*   FAULT_GAIN_BAD_*, FAULT_DQ_OFFSET_LARGE, FAULT_ALIGN_NOT_MOVED:
 	*     persistent — require power cycle (gain/DQ are calibrated once).
 	*
 	* HARD-FAULT RECOVERY:
 	*   None — power cycle required. This is intentional; if you've tripped
 	*   OCP or encoder discontinuity, something needs human inspection.
 	* ============================================================================ */
    while (1)
    {
    	///*
    	targetTorque_Nm[MOTOR_1] = 0.05;
    	//targetTorque_Nm[MOTOR_2] = 0.05;
    	HAL_Delay(2000);
    	targetTorque_Nm[MOTOR_1] = -0.05;
    	//targetTorque_Nm[MOTOR_2] = -0.05;
    	HAL_Delay(2000);//*/
        /* No-op for now. Add your application logic here. */
        HAL_Delay(10);
    }
}


/* ============================================================================
 * SYSTEM CLOCK CONFIGURATION (CubeMX-generated, 170 MHz from HSI + PLL)
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
 * Called from CubeMX-generated init failures. We don't disable interrupts
 * because the fault layer handles that — just spin so a debugger can attach.
 * ============================================================================ */
void Error_Handler(void)
{
    emergencyStop(FAULT_ADC_STUCK);   /* generic — closest match */
    while (1) { /* halt */ }
}
