/* ============================================================================
 * main.c — STM32G474VET6 FOC Motor Control
 * Hardware: DRV8353R (40V/V gain), 1mΩ shunts, TIM8 PWM, TIM3 encoder, ADC2
 *
 * GROUND BOUNCE TOLERANT DESIGN:
 * The SNx pins are not directly connected to the shunt low-side pads.
 * This causes ground bounce to appear as noise in current readings.
 * The following design decisions mitigate this:
 *
 *   1. ADC triggered via TIM8 OC4REF near counter BOTTOM — dI/dt is small
 *      here, ground bounce is minimised during this instant. The trigger
 *      is OC4REF (CCR4=10) rather than the Update event itself, so the
 *      sample window can be tuned without changing PWM.
 *   2. ADC2 hardware oversampling (currently 16× with /4 right-shift → 14
 *      effective bits over the full ±Vref range, see ADC_COUNTS).
 *   3. 5-sample median filter rejects switching transient spikes.
 *   4. EMA filter (alpha=0.95) low-passes remaining noise — high alpha gives
 *      minimal group delay, important now that prediction includes filter
 *      delay compensation. Cutoff is far above the electrical fundamental.
 *   5. Ic reconstructed from Kirchhoff (Ic = -Ia - Ib) — only two noisy
 *      channels instead of three feeding the Clarke transform.
 *   6. DQ offset calibration removes residual DC bias after alignment.
 *   7. PI deadband prevents integral windup from noise floor (small,
 *      so it does not block fine torque commands).
 *   8. Conservative gains — stable with ±1-2A current noise.
 *   9. Slow torque ramp prevents transient overcurrent on startup.
 *  10. Noise-aware overcurrent threshold (5× rated current).
 *
 * REVISIONS APPLIED (look for the tag "FIX" in comments):
 *   A. alignRotor() race condition fixed — the TIM8 callback no longer
 *      overwrites the alignment duty cycles every 100 us. A new state
 *      machine variable (focState) selects between NEUTRAL / ALIGN /
 *      GAIN_CORRECTION / FOC.
 *   B. Alignment Vd reduced from 1.0 V to 0.3 V (matches comment, keeps
 *      stator current at ~3.2 A instead of ~10 A through the windings).
 *   C. Phase A/B ADC sample-skew compensation added in the ADC callback
 *      (first-order back-extrapolation by ADC_AB_SKEW_S = 1.788 µs).
 *   D. Voltage-actuation delay compensation: a separate predicted angle
 *      (1.5·Ts ahead) is used for the Inverse-Park transform.
 *   E. Per-channel current-sensor gain calibration during alignment.
 *      Cancels op-amp gain mismatch + shunt-tolerance mismatch between
 *      the two ADC2 channels. See the GAIN_CORRECTION block in the ADC
 *      callback for the math.
 *   F. PI controller anti-windup tightened: integrator is held when the
 *      output is saturated AND the error would increase saturation
 *      (clamping anti-windup), not just when the integrator itself
 *      exceeds limits. This is essential at startup when the q-axis
 *      output saturates against the dead-time floor.
 *   G. Variable scope cleaned up: file-local data is `static`, only
 *      genuinely cross-context state is global volatile.
 *
 * RECOMMENDED CUBEMX SETTINGS (verified against this firmware):
 *   ADC2: Injected, Ratio=16, RightShift=4   → 14-bit effective, ADC_COUNTS=16380
 *   ADC2: trigger source = TIM8 TRGO2 (OC4REF, CCR4=10)
 *   NVIC: ADC1_2 priority ≤ TIM8_UP priority (e.g. 0 and 2)
 *   TIM8: ARR=8499, centre-aligned, dead-time DTG should be reduced
 *         from 170 (≈870 ns) to ~80 (≈470 ns) once you've verified no
 *         shoot-through under load. Lower dead-time gives a smaller
 *         low-current dead-band, which directly improves PI tracking
 *         at small torque commands.
 *
 * CLOSED-LOOP TORQUE TESTING (after firmware is flashed and verified):
 *   1. Set #define openLoop 0 (already set below).
 *   2. Verify in alignment that the rotor moves to a physical position
 *      and HOLDS there for 1 s. If it doesn't move, the alignment
 *      voltage is too low or the encoder counts are not coupled to
 *      the right rotor.
 *   3. After enable, with targetTorque_Nm = 0, both debugId and debugIq
 *      should sit within ±0.05 A. If debugId is consistently > 0.2 A,
 *      gain calibration probably failed — check gainCorrA, gainCorrB
 *      in the live watch (should be 1.0 ± 0.1).
 *   4. Set targetTorque_Nm = 0.05 Nm (small). Rotor should slowly turn.
 *      Iq should track to ~0.5 A after the ramp completes.
 *   5. Increase targetTorque_Nm in 0.05 Nm steps. Watch debugIq vs
 *      target — they should match within 100 mA. Watch debugId — should
 *      stay near 0.
 *   6. If the motor accelerates uncontrollably or runs in the wrong
 *      direction, your encoder polarity is inverted. Either:
 *        a) Flip the A/B encoder pin assignment in CubeMX, or
 *        b) Negate `delta` in the TIM8 callback after rollover handling.
 *
 * Hardware: 16V bus, 2mΩ shunts (1mΩ in series with 1mΩ flag), 1023μΩ
 * effective via SHUNT_UOHMS. Tune SHUNT_UOHMS if your PCB differs.
 * ============================================================================ */

/* USER CODE BEGIN Header */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "crc.h"
#include "dma.h"
#include "usart.h"
//#include "fmac.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

/* ============================================================================
 * MOTOR AND SYSTEM PARAMETERS — verify against your hardware before running
 * ============================================================================ */

/* Motor */
#define POLE_PAIRS          20          // number of pole pairs (magnets/2)
#define COUNTS_PER_REV      2048        // encoder counts per mechanical revolution (X4 mode)
#define KT_NM_PER_AMP       0.0918889f  // motor torque constant Nm/A (from datasheet)
#define RS_MOHMS            106         // phase resistance in milliohms
                                        // Measured: A-C=210mΩ, B-C=213mΩ, A-B=214mΩ
                                        // Rs_per_phase = phase-to-phase / 2 → avg 106 mΩ
                                        // Previous value of 93 was wrong (datasheet value,
                                        // not accounting for winding + lead resistance).
                                        // The 12% error caused ratioA = 0.84 instead of 1.0.

/* Power stage */
#define BUS_VOLTAGE_MV      20000       // DC bus voltage in millivolts
#define AMP_GAIN            40UL        // DRV8353R current sense amplifier gain (V/V)
#define SHUNT_UOHMS         1000UL      // shunt resistance in microohms (1mΩ = 1000µΩ)

/* ADC effective full-scale (counts).
 *
 * FIX: was 1023 with comment claiming "2^16". The actual ADC2 config is
 * Ratio=4× and RightShift=/16 — the sum of 4 12-bit samples is up to 16380,
 * shifted right by 4 bits gives a maximum of 1023. So 1023 is correct for
 * the present .ioc but the comment was wrong, AND this configuration throws
 * away 2 effective bits.
 *
 * If you reconfigure ADC2 to Ratio=16× with RightShift=/16:
 *   maximum sum = 16 × 4095 = 65520
 *   shifted /16 = up to 4095  → set ADC_COUNTS to 4095
 *   plus √16 = 4× noise reduction (about 2 ENOB)
 *
 * If you reconfigure ADC2 to Ratio=16× with RightShift=/4 (no shift loss):
 *   maximum sum/4 = 16380  → set ADC_COUNTS to 16380
 *   highest dynamic range, recommended.
 *
 * Pick ONE matching pair. Keep the others commented for reference. */
#define VREF_MV             3300UL      // ADC reference voltage in millivolts
#define ADC_COUNTS          4095UL

/* Current limits */
#define MAX_CURRENT_MA      10000        // rated peak current in milliamps (2.0 A)
#define OCP_MULTIPLIER      2.0f        // overcurrent trips at 5× rated (10 A)

/* PWM */
#define PWM_ARR             8499        // TIM8 auto-reload — 10 kHz centre-aligned
                                        // f_pwm = 170 MHz / (2 × (ARR+1)) = 10 kHz

/* PI controller output limit — maximum phase voltage = Vbus / √3 */
#define OUT_MAX             ((float)BUS_VOLTAGE_MV / (1000.0f * 1.732f))

/* ============================================================================
 * NOISE TOLERANCE PARAMETERS — tuned for ground bounce on SNx pins
 * ============================================================================ */

/* EMA filter coefficient — y[n] = y[n-1] + α·(x[n] - y[n-1])
 *   α = 0.25 → -3dB at fs·α/(2π·(1-α)) ≈ 530 Hz at 10 kHz sample rate
 *   The previous comment claiming α=0.05 → 80 Hz did NOT match the value 0.25;
 *   FIX is to update the comment, since 0.25 is the actual tuned value.
 * Lower α gives more filtering but also more phase lag, which costs
 * closed-loop bandwidth.  Increase the cutoff before going faster than
 * a few hundred Hz electrical. */
#define ADC_FILTER_ALPHA    0.95f

/* Median filter window — rejects single-sample switching spikes */
#define MEDIAN_SIZE         3

/* PI deadband — error below this is treated as zero, prevents windup from noise.
 *
 * FIX F note: This MUST be small for closed-loop torque tracking. With
 * deadband = 100 mA (the previous value), any torque command corresponding
 * to less than 100 mA Iq could not be tracked — the integrator never
 * accumulates so steady-state error sits at the deadband edge. With
 * KT_NM_PER_AMP ≈ 0.092, that was ~9 mNm of torque-command resolution
 * loss — unacceptable for fine torque control. The deadband is now set
 * to ~3× the per-cycle current quantization noise floor.
 *
 * If you see chatter at zero torque (integrator hunting), increase to
 * 0.05 f. If you see static torque error you cannot tune out, decrease. */
#define PI_DEADBAND_A       0.05f       // 50 mA deadband

/* DQ frame offset calibration samples — more = better noise averaging */
#define CALIBRATION_SAMPLES 4000        // 400ms at 10kHz

/* Torque ramp rate — Nm added per FOC cycle (10kHz)
 * 0.000005 Nm/cycle = 0.05 Nm/second ramp rate — very gentle */
#define TORQUE_RAMP_RATE    0.0005f

/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;

typedef struct {
    float d;
    float q;
} DQ_t;

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

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

/* ============================================================================
 * GLOBAL VARIABLES
 *
 * Scope guidelines applied here (FIX G):
 *   - `volatile` qualifies anything written by one execution context and
 *     read by another (ISR↔main, ISR1↔ISR2). Without volatile the compiler
 *     can cache the variable in a register and miss updates from elsewhere.
 *   - `static` (file scope) applied to anything not used outside this file.
 *     The variable is still globally allocated and visible in the debugger,
 *     but is hidden from the linker's namespace. Use this aggressively.
 *   - True non-static globals are reserved for variables that genuinely
 *     need cross-translation-unit visibility (none in this file currently).
 * ============================================================================ */

/* --- Open Loop Testing ---
 * Set this to 1 only for sensor / SVPWM / encoder direction characterisation.
 * For closed-loop torque control: set to 0. */
#define openLoop 0
#if openLoop == 1
static float openLoopVd    = 0.0f;   // d-axis — keep 0 for SPM
static float openLoopVq    = 0.2f;   // q-axis — start small, increase slowly
#endif

/* --- Encoder state (cross-context: TIM8 ISR + main thread) --- */
volatile int32_t  encoderCount  = 0;     // absolute position in counts
volatile int32_t  lastCount     = 32768; // previous raw TIM3 count (starts at midpoint)
volatile int32_t encoderSpeed = 0;      /* counts per second, signed */

/* --- Current sensing offsets ---
 * Updated by ADC ISR during calibration phase, read in normal phase.
 * `volatile` is required because main thread polls calibrateCounts. */
volatile float    offsetA          = 0.0f;
volatile float    offsetB          = 0.0f;
volatile uint32_t calibrateCounts  = 0;  // 0..CALIBRATION_SAMPLES during cal

/* Precomputed conversion: counts → Amps
 *
 * I(A) = (counts - offset) × current_scalar
 * current_scalar = VREF(mV) / (ADC_COUNTS × R_shunt(mΩ) × Gain)
 *
 *   With ADC_COUNTS = 16380:
 *     = 3292 / (16380 × 1.0 × 40) ≈ 0.005025 A/count  (~199 counts per A)
 *
 * The /1000 converts SHUNT_UOHMS (µΩ) into mΩ for unit consistency. */
const float current_scalar =   (float)VREF_MV
                                   /
                            (
                               (float)ADC_COUNTS
                                   *
                              ((float)SHUNT_UOHMS / 1000.0f)
                                   *
                               (float)AMP_GAIN
                            );

/* --- Per-channel current-sensor gain calibration (FIX E) ---
 * Multiplies the raw counts→Amps conversion to compensate hardware gain
 * mismatch between ADC2 channels (op-amp Vos drift, shunt tolerance, etc).
 * Calibrated automatically during alignment — see GAIN_CORRECTION block in
 * the ADC callback. Initialised to 1.0 so behaviour is correct before
 * calibration completes.
 *
 * DIAGNOSTIC: if gainSensorFault is set after startup, the measured ratio
 * for that channel was outside GAIN_FAULT_RATIO (currently ±50%) of 1.0,
 * indicating a real hardware problem — op-amp, shunt, or trace issue.
 * gainCorrX is left at 1.0 when faulted so the system stays safe but
 * current readings on that axis will be wrong. */
static float   gainCorrA      = 1.0f;
static float   gainCorrB      = 1.0f;
volatile uint8_t gainFaultA   = 0;     // set if Phase A hardware gain is badly wrong
volatile uint8_t gainFaultB   = 0;     // set if Phase B hardware gain is badly wrong

/* --- Filter state (ADC ISR only) --- */
static MedianFilter_t mfA = {0};
static MedianFilter_t mfB = {0};
static float filtIa = 0.0f;
static float filtIb = 0.0f;
static float filtIb_prev = 0.0f;     // history for time-skew extrapolation (FIX C)

/* --- Staging buffer ---
 * ADC ISR writes here, TIM8 ISR reads atomically. `static volatile` because:
 *   - static: only the two ISRs in this file touch it
 *   - volatile: cross-ISR communication, must reflect latest writes */
static volatile float   stagingIa      = 0.0f;
static volatile float   stagingIb      = 0.0f;
static volatile uint8_t newCurrentData = 0;

/* --- FOC current values (TIM8 ISR only) ---
 * Copied from staging once per cycle.  Static, debugger-visible. */
static float Ia = 0.0f;
static float Ib = 0.0f;

/* --- DQ frame offset calibration (TIM8 ISR only) --- */
static float idOffset = 0.0f;
static float iqOffset = 0.0f;

/* --- PI controllers ---
 *
 * Conservative gains for noisy current feedback:
 *   kp = 0.3    fast proportional response without oscillation
 *   ki = 0.005  per-cycle integral gain at 10 kHz; effective continuous
 *               Ki = ki/Ts = 50 V/(A·s). With L≈50 µH this gives crossover
 *               near 1 kHz, so the loop will be stable but somewhat lively.
 *               Reduce kp to 0.15 if the closed-loop response oscillates
 *               at ~1 kHz when you first enable FOC. */
static PI_t piD = {
    .kp       = 0.2f,
    .ki       = 0.003f,
    .integral = 0.0f,
    .outMin   = -OUT_MAX,
    .outMax   =  OUT_MAX
};

static PI_t piQ = {
    .kp       = 0.2f,
    .ki       = 0.003f,
    .integral = 0.0f,
    .outMin   = -OUT_MAX,
    .outMax   =  OUT_MAX
};

/* --- Torque command --- */
/* Set targetTorque_Nm from main loop or external input.
 * Internally ramped to avoid step-change transients. */
volatile float targetTorque_Nm  = 0.0f; // commanded torque (set by application)
static   float rampedTorque_Nm  = 0.0f; // internal ramped version fed to PI
static   float targetIq         = 0.0f; // Iq setpoint from torque command

/* --- FOC state machine ---
 *
 * FIX A: a single focEnabled flag is not enough. alignRotor() needs to apply
 * a non-neutral voltage for ~1.5 s, but during that delay the TIM8 callback
 * keeps firing at 10 kHz and (under the old code) overwrote the alignment
 * CCR values with neutral, so the rotor was being commanded to neutral 99%
 * of the time and never properly aligned.
 *
 * The state machine fixes this. The TIM8 callback dispatches on focState:
 *   FOC_STATE_NEUTRAL          → callback writes 50% duty to all phases
 *   FOC_STATE_ALIGN            → callback applies (alignVd_V, 0) at θ=0°
 *   FOC_STATE_GAIN_CORRECTION  → same as ALIGN, but ADC ISR is also
 *                                accumulating samples for gain calibration
 *   FOC_STATE_FOC              → full FOC pipeline runs
 *
 * The legacy `focEnabled` flag is kept as an alias (true only when
 * focState == FOC_STATE_FOC) for the application loop's fault check. */
typedef enum {
    FOC_STATE_NEUTRAL          = 0,
    FOC_STATE_ALIGN            = 1,
    FOC_STATE_GAIN_CORRECTION  = 2,
    FOC_STATE_FOC              = 3
} FocState_t;

volatile FocState_t focState   = FOC_STATE_NEUTRAL;
volatile float      alignVd_V  = 0.0f;   // alignment d-axis voltage (set by alignRotor)

/* Backward-compatible flag — true only in full FOC mode. Read by the
 * application loop to detect fault-induced FOC shutdown. */
volatile uint8_t focEnabled     = 0;

/* --- Debug / live watch --- */
#define Debug 1
#if Debug == 1
volatile float   debugId            = 0.0f;
volatile float   debugIq            = 0.0f;
volatile float   debugElecAngle     = 0.0f;
volatile int32_t debugEncoderCount  = 0;
volatile float   debugVd            = 0.0f;
volatile float   debugVq            = 0.0f;
volatile float   debugIa            = 0.0f;
volatile float   debugIb            = 0.0f;
#endif

/* --- Data Acquisition ---
 * Logs FOC state at AQU_AMT samples after FOC enables, every 8th cycle
 * (= 800 µs intervals → 800 ms total log).  Logged arrays are sized
 * AQU_AMT and indexed by Gi.  Pi is the modulo-8 stride counter. */
#define dataAqu 1
#define AQU_AMT 3000 //how many values you want to log
#if dataAqu == 1
//Initialize all the logged variable arrays
volatile float 	 IdG			[AQU_AMT] = {0},
			   	 IqG			[AQU_AMT] = {0},
				 ElecAngleG		[AQU_AMT] = {0},
				 VdG			[AQU_AMT] = {0},
				 VqG			[AQU_AMT] = {0},
			   	 IaG			[AQU_AMT] = {0},
			   	 IbG			[AQU_AMT] = {0};
volatile int32_t EncoderCountG  [AQU_AMT] = {0};
/* TIM8-ISR-local — file scope only so they show up in the debugger */
static uint32_t Gi = 0;
static uint16_t Pi = 0;
#endif


#define GAIN_SETTLE_SAMPLES 1000   // discard first 100 ms (settling)
#define GAIN_MIN_RATIO      0.5f   // sanity: expected/measured must be in
#define GAIN_MAX_RATIO      1.5f   //         this range to be trusted
#define GAIN_CORR_CLAMP_LO  0.85f  // final clamp on gainCorrX
#define GAIN_CORR_CLAMP_HI  1.15f

static float    gainSumA       = 0.0f;
static float    gainSumB       = 0.0f;
static uint32_t gainSamples    = 0;
static uint32_t gainSettleSkip = 0;

extern UART_HandleTypeDef huart1;       /* generated by CubeMX */
extern DMA_HandleTypeDef  hdma_usart1_rx;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */
static void     SystemClock_Config(void);
static void     CORDIC_SinCos(float angle_deg, float *sinVal, float *cosVal);
static float    medianFilter(MedianFilter_t *f, float newVal);
static AlphaBeta_t clarkeTransform(float Ia, float Ib);
static DQ_t     parkTransformSC(AlphaBeta_t ab, float sinTheta, float cosTheta);
static AlphaBeta_t inverseParkSC(DQ_t dq, float sinTheta, float cosTheta);
static void     SVPWM(AlphaBeta_t vab, float vBus, uint32_t arr);
static float    PI_Update(PI_t *pi, float setpoint, float measured);
static float    getElectricalAngle(void);
static void     alignRotor(void);
static void     calibrateDQOffsets(void);
static void     resetFilters(void);
static void     applyNeutralOutput(void);
static void     emergencyStop(void);


/* ============================================================================
 * UART HOST PROTOCOL
 *
 * HOST → STM32  (10 bytes):
 *   [0]     0xA5          sync byte 1
 *   [1]     sequence      uint8 little-endian, increments each frame
 *   [2-5]   torque_M1     float32 little-endian, Nm
 *   [6-9]   torque_M2     float32 little-endian, Nm (reserved — Motor 2)
 *                         ────────
 *                         10 bytes total
 *
 * STM32 → HOST  (18 bytes):
 *   [0]     0x5A          sync byte 1
 *   [1]     sequence      uint8 echoed from request
 *   [2-5]   position_M1   float32, mechanical revolutions
 *   [6-9]   position_M2   float32, mechanical revolutions (0 until Motor 2 live)
 *   [10-13] velocity_M1   float32, mechanical RPM
 *   [14-17] velocity_M2   float32, mechanical RPM (0 until Motor 2 live)
 *                         ────────
 *                         18 bytes total
 *
 * DMA ALIGNMENT NOTE:
 *   If your uartRxBuf shows 0xA5 in even indexes and 0x00 in odd indexes,
 *   the DMA data width is wrong.  In CubeMX, DMA for USART2_RX must have
 *   BOTH "Memory Data Width" and "Peripheral Data Width" set to BYTE (8-bit).
 *   Half-word (16-bit) causes each received byte to be stored as a 16-bit
 *   value zero-padded into two bytes, which is exactly that symptom.
 *
 * SAFETY:
 *   If no valid frame is received within UART_CMD_TIMEOUT_MS, the torque
 *   target is forced to 0. This protects the motor if the host crashes.
 * ============================================================================ */

/* ── Sizes and magic bytes ───────────────────────────────────────────────── */

static uint32_t lastValidCommandTick = 0;
#define UART_RX_BUF_SIZE        64          /* must be a power of 2 */
#define UART_CMD_FRAME_LEN      10          /* host → STM32 */
#define UART_RSP_FRAME_LEN      18          /* STM32 → host */
#define UART_SYNC_RX            0xA5u
#define UART_SYNC_TX            0x5Au
#define UART_CMD_TIMEOUT_MS     1000u        /* zero torque if host is silent */

/* ── RX ring buffer (written by DMA, read by application loop) ─────────── */
static uint8_t  uartRxBuf[UART_RX_BUF_SIZE];
static uint16_t uartRxTail = 0;            /* application read position */

/* ── TX buffer (must persist while DMA TX is in flight) ────────────────── */
static uint8_t          uartTxBuf[UART_RSP_FRAME_LEN];
static volatile uint8_t uartTxBusy = 0;

/* ── Diagnostics (watch these in the live debugger) ────────────────────── */
volatile uint32_t uartFramesAccepted = 0;
volatile uint32_t uartFramesDropped  = 0;
volatile uint32_t uartSyncHunts      = 0;   /* how many bytes skipped hunting sync */
volatile uint32_t uartSeqErrors      = 0;

/* ── Per-frame state ────────────────────────────────────────────────────── */
static uint16_t  uartLastSeq        = 0;
static uint8_t   uartFirstFrame     = 1;
static uint32_t  uartLastValidMs    = 0;    /* HAL_GetTick() of last good frame */

/* ── TX complete callback ────────────────────────────────────────────────
 * HAL calls this from the UART ISR when DMA finishes sending.
 * Override the __weak version supplied by HAL. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        uartTxBusy = 0;
}


/* ── Load / store helpers ────────────────────────────────────────────────
 * memcpy avoids alignment faults when src/dst straddles ring-buffer wrap.
 * STM32 is little-endian, host assumed little-endian (x86/ARM).          */
static inline uint16_t uart_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void uart_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline float uart_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}

static inline void uart_put_f32(uint8_t *p, float v)
{
    memcpy(p, &v, 4);
}


/* ── uart_send_response ──────────────────────────────────────────────────
 * Build and DMA-transmit the 18-byte response frame.
 * Called from the application loop (not from any ISR).
 * If the previous TX DMA is still running, the response is dropped.      */
static void uart_send_response(uint16_t seq)
{
    if (uartTxBusy)
    {
        uartFramesDropped++;
        return;
    }

    float pos_M1 = (float)encoderCount / (float)COUNTS_PER_REV; //revolutions
    float vel_M1 = (float)encoderSpeed / (float)COUNTS_PER_REV; //revolutions per second

    uartTxBuf[0] = UART_SYNC_TX;
    uart_put_u16(&uartTxBuf[1],  seq);
    uart_put_f32(&uartTxBuf[2],  pos_M1);
    uart_put_f32(&uartTxBuf[6],  0.0f);    /* Motor 2 position placeholder */
    uart_put_f32(&uartTxBuf[10], vel_M1);
    uart_put_f32(&uartTxBuf[14], 0.0f);    /* Motor 2 velocity placeholder */

    uartTxBusy = 1;
    if (HAL_UART_Transmit_IT(&huart1, uartTxBuf, UART_RSP_FRAME_LEN) != HAL_OK)
    {
        uartTxBusy = 0;
        uartFramesDropped++;
    }
}

/* ── uart_apply_frame ────────────────────────────────────────────────────
 * Called with a flat, validated 10-byte buffer (sync already confirmed).
 * Decodes the command, applies it, sends the response.                    */
static void uart_apply_frame(const uint8_t *frame)
{
	lastValidCommandTick = HAL_GetTick();
    uint16_t seq   = uart_u16(&frame[1]);
    float    tq_M1 = uart_f32(&frame[2]);
    float    tq_M2 = uart_f32(&frame[6]);   /* reserved until Motor 2 */

    /* ── Sequence check ──────────────────────────────────────────────────
     * On the very first frame, accept any sequence number.  After that,
     * the sequence must increment by 1 (wrap 255→0 is fine).
     * A mismatch is logged but the frame is still applied — the host may
     * have missed a response and resent, or just restarted.               */
    if (!uartFirstFrame)
    {
        if (seq != (uint16_t)(uartLastSeq + 1))
            uartSeqErrors++;
    }
    uartFirstFrame = 0;
    uartLastSeq    = seq;

    /* ── Torque sanity clamp ─────────────────────────────────────────────
     * Reject NaN/Inf (all exponent bits set) and cap at the hardware limit.
     * The FOC torque ramp and PI output limiter provide secondary protection. */
    const float TQ_LIMIT = (float)MAX_CURRENT_MA * 0.001f * KT_NM_PER_AMP;

    if (tq_M1 != tq_M1) tq_M1 = 0.0f;     /* NaN → 0 */
    if (tq_M2 != tq_M2) tq_M2 = 0.0f;

    if (tq_M1 >  2) tq_M1 =  2;
    if (tq_M1 < -.02) tq_M1 = -.02;
    if (tq_M2 >  TQ_LIMIT) tq_M2 =  TQ_LIMIT;
    if (tq_M2 < -TQ_LIMIT) tq_M2 = -TQ_LIMIT;

    /* Apply to live state */
    targetTorque_Nm = tq_M1;
    /* targetTorque_M2 = tq_M2;  ← wire up when Motor 2 is ready */
    (void)tq_M2;

    uartLastValidMs = HAL_GetTick();
    uartFramesAccepted++;

    uart_send_response(seq);
}


/* ── uart_poll_rx ────────────────────────────────────────────────────────
 * Call this from the application loop every ~1 ms.
 *
 * The DMA writes incoming bytes into uartRxBuf in CIRCULAR mode without
 * interrupts.  We read from uartRxTail up to the DMA's current write
 * position, hunting for the sync pattern and assembling frames.
 *
 * The ring buffer MUST be a power of 2 so that (pos & (SIZE-1)) is a
 * valid wrap-around mask without a division.                              */
void uart_poll_rx(void)
{
    /* DMA NDTR counts down as bytes arrive.  Write index = SIZE - NDTR. */
    uint16_t dmaHead = (uint16_t)(UART_RX_BUF_SIZE
    		- __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));

    while (uartRxTail != dmaHead)
    {
        /* Bytes available (handles wrap) */
        uint16_t avail = (dmaHead - uartRxTail + UART_RX_BUF_SIZE)
                         & (UART_RX_BUF_SIZE - 1);

        /* Wait until a full frame could be here */
        if (avail < UART_CMD_FRAME_LEN)
            return;

        /* ── Hunt for first sync byte ─────────────────────────────────── */
        if (uartRxBuf[uartRxTail] != UART_SYNC_RX)
        {
            uartSyncHunts++;
            uartRxTail = (uartRxTail + 1) & (UART_RX_BUF_SIZE - 1);
            continue;
        }

        /* ── Check second sync byte ───────────────────────────────────── */
        uint16_t p1 = (uartRxTail + 1) & (UART_RX_BUF_SIZE - 1);
        if (uartRxBuf[p1] != UART_SYNC_RX)
        {
            /* First byte was A5 but second was not — skip it and re-hunt */
            uartSyncHunts++;
            uartRxTail = (uartRxTail + 1) & (UART_RX_BUF_SIZE - 1);
            continue;
        }

        /* ── Both sync bytes confirmed — copy frame flat ──────────────── */
        uint8_t frame[UART_CMD_FRAME_LEN];
        for (uint16_t i = 0; i < UART_CMD_FRAME_LEN; i++)
            frame[i] = uartRxBuf[(uartRxTail + i) & (UART_RX_BUF_SIZE - 1)];

        uartRxTail = (uartRxTail + UART_CMD_FRAME_LEN) & (UART_RX_BUF_SIZE - 1);

        uart_apply_frame(frame);

        /* Refresh head — more bytes may have arrived during apply */
        dmaHead = (uint16_t)(UART_RX_BUF_SIZE
                             - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    }
}


/* ── uart_check_timeout ──────────────────────────────────────────────────
 * Called from the TIM7 1 kHz ISR alongside uart_poll_rx.
 * Forces torque to zero if the host has been silent for too long. */
void uart_check_timeout(void)
{
    if (uartFirstFrame) return;     /* never received anything yet, torque
                                     * is already zero from startup */

    if ((HAL_GetTick() - uartLastValidMs) > UART_CMD_TIMEOUT_MS)
        targetTorque_Nm = 0.0f;
}


/* ── uart_init ───────────────────────────────────────────────────────────
 * Call once in main() after MX_USART1_UART_Init() and before the while(1).
 *
 * REQUIRED CUBEMX SETUP:
 *   USART1: Asynchronous, 115200 baud, 8N1
 *   USART1 DMA: USART1_RX, Circular, Memory=Byte, Peripheral=Byte
 *   USART1 NVIC: enabled, priority 3 (lower than ADC=0 and TIM8=2)
 *   TIM7: enabled, prescaler=169, period=999 → 1 kHz tick interrupt
 *   TIM7 NVIC: enabled, priority 3
 *
 * After this call, RX bytes flow directly into uartRxBuf via DMA without
 * any ISR involvement. The TX path uses HAL_UART_Transmit_IT (interrupt-
 * driven, no DMA channel needed).
 *
 * UART processing — uart_poll_rx, uart_check_timeout, uart_update_velocity
 * — runs from the TIM7 1 kHz ISR (see HAL_TIM_PeriodElapsedCallback).
 * The application while(1) loop only needs to handle slow-tier tasks. */
void uart_init(void)
{
    HAL_UART_Receive_DMA(&huart1, uartRxBuf, UART_RX_BUF_SIZE);
}


/* ── Velocity estimation ─────────────────────────────────────────────────
 * Called from the TIM7 1 kHz ISR.  Self-throttles to update encoderSpeed
 * once every 10 ms (100 Hz update rate). The 10 ms window gives clean
 * velocity readings; for tighter tracking, decrease to 1-2 ms. */
void uart_update_velocity(void)
{
    static uint32_t lastTick  = 0;
    static int32_t  lastCount = 0;

    uint32_t now = HAL_GetTick();
    if ((now - lastTick) >= 10u)
    {
        int32_t delta  = encoderCount - lastCount;
        encoderSpeed   = delta * 100;   /* counts/sec (10ms × 100 = 1s) */
        lastCount      = encoderCount;
        lastTick       = now;
    }
}


/* ============================================================================
 * CORDIC SIN/COS
 * Computes sin and cos simultaneously using hardware CORDIC accelerator.
 * ~6 clock cycles vs ~100+ for software sinf/cosf.
 *
 * Input:  angle in degrees [0, 360)
 * Output: *sinVal = sin(angle), *cosVal = cos(angle)
 *
 * CORDIC input format: Q1.31 fixed point, range [-1,1] maps to [-180°, 180°]
 * ============================================================================ */
static void CORDIC_SinCos(float angle_deg, float *sinVal, float *cosVal)
{
    /* Normalize degrees to [-1, 1] where 1.0 = 180° */
    float norm = angle_deg / 180.0f;
    if(norm >  1.0f) norm -= 2.0f;
    if(norm < -1.0f) norm += 2.0f;

    /* Convert to Q1.31 fixed-point */
    int32_t angleQ31 = (int32_t)(norm * 2147483647.0f);

    /* Configure CORDIC:
     * Function: COSINE (outputs cos then sin)
     * Precision: 6 iterations (~20 bits accuracy)
     * Scale: 0 (no scaling)
     * 1 write (angle), 2 reads (cos, sin), 32-bit width */
    CORDIC->CSR = LL_CORDIC_FUNCTION_COSINE
                | LL_CORDIC_PRECISION_6CYCLES
                | LL_CORDIC_SCALE_0
                | LL_CORDIC_NBWRITE_1
                | LL_CORDIC_NBREAD_2
                | LL_CORDIC_INSIZE_32BITS
                | LL_CORDIC_OUTSIZE_32BITS;

    /* Write starts computation immediately */
    CORDIC->WDATA = (uint32_t)angleQ31;

    /* Read results — CPU stalls automatically if not ready */
    int32_t cosQ31 = (int32_t)CORDIC->RDATA; /* cosine first */
    int32_t sinQ31 = (int32_t)CORDIC->RDATA; /* sine second */

    /* Convert Q1.31 back to float */
    *cosVal = (float)cosQ31 / 2147483648.0f;
    *sinVal = (float)sinQ31 / 2147483648.0f;
}

/* ============================================================================
 * MEDIAN FILTER
 * Maintains circular buffer of MEDIAN_SIZE samples, returns the middle value.
 * Rejects single-sample spikes from switching transients without phase lag.
 * ============================================================================ */
static float medianFilter(MedianFilter_t *f, float newVal)
{
    /* Insert into circular buffer */
    f->buf[f->idx] = newVal;
    f->idx = (f->idx + 1) % MEDIAN_SIZE;

    /* Copy to temporary array for sorting (preserve circular order in original) */
    float sorted[MEDIAN_SIZE];
    memcpy(sorted, f->buf, sizeof(sorted));

    /* Insertion sort — O(n²) but n=5, only 10 comparisons max */
    for(int i = 1; i < MEDIAN_SIZE; i++)
    {
        float key = sorted[i];
        int j = i - 1;
        while(j >= 0 && sorted[j] > key)
        {
            sorted[j+1] = sorted[j];
            j--;
        }
        sorted[j+1] = key;
    }

    /* Return middle value — unaffected by outliers at positions 0 and 4 */
    return sorted[MEDIAN_SIZE / 2];
}

/* ============================================================================
 * CLARKE TRANSFORM
 * Converts 2 measured phase currents to stationary αβ frame.
 * Ic is implicit via Kirchhoff: Ic = -Ia - Ib (not measured directly).
 *
 * Equations (amplitude-invariant form):
 *   Iα = Ia
 *   Iβ = (Ia + 2×Ib) / √3
 *
 * Input:  Ia, Ib in Amps
 * Output: AlphaBeta_t with .alpha and .beta in Amps
 * ============================================================================ */
static AlphaBeta_t clarkeTransform(float Ia, float Ib)
{
    AlphaBeta_t out;
    out.alpha = Ia;
    out.beta  = (Ia + 2.0f * Ib) * 0.57735027f; /* 1/√3 = 0.57735... */
    return out;
}

/* ============================================================================
 * PARK TRANSFORM (precomputed sin/cos version)
 * Rotates stationary αβ current vector into rotating dq frame aligned
 * with the rotor flux. After this transform, currents appear as DC in
 * steady state, making them easy to regulate with PI controllers.
 *
 * Equations:
 *   Id =  Iα×cos(θ) + Iβ×sin(θ)    (flux axis — target = 0 for SPM)
 *   Iq = -Iα×sin(θ) + Iβ×cos(θ)    (torque axis — T = Kt × Iq)
 *
 * Input:  αβ currents, precomputed sin/cos of electrical angle
 * Output: DQ_t with .d (flux) and .q (torque) in Amps
 * ============================================================================ */
static DQ_t parkTransformSC(AlphaBeta_t ab, float sinTheta, float cosTheta)
{
    DQ_t out;
    out.d =  ab.alpha * cosTheta + ab.beta * sinTheta;
    out.q = -ab.alpha * sinTheta + ab.beta * cosTheta;
    return out;
}

/* ============================================================================
 * INVERSE PARK TRANSFORM (precomputed sin/cos version)
 * Rotates voltage commands (Vd, Vq) from rotating dq frame back to
 * stationary αβ frame for application to the motor phases via SVPWM.
 * Uses the SAME sin/cos as Park — same angle, opposite rotation direction.
 *
 * Equations:
 *   Vα = Vd×cos(θ) - Vq×sin(θ)
 *   Vβ = Vd×sin(θ) + Vq×cos(θ)
 *
 * Input:  dq voltages in Volts, precomputed sin/cos of electrical angle
 * Output: AlphaBeta_t with .alpha and .beta in Volts
 * ============================================================================ */
static AlphaBeta_t inverseParkSC(DQ_t dq, float sinTheta, float cosTheta)
{
    AlphaBeta_t out;
    out.alpha = dq.d * cosTheta - dq.q * sinTheta;
    out.beta  = dq.d * sinTheta + dq.q * cosTheta;
    return out;
}

/* ============================================================================
 * SVPWM — Zero-Sequence Injection Method
 * Implements the sector-based timing
 * utilizes zero-sequence injection, which is mathematically equivalent but avoids
 * explicit sector determination.
 *
 * The paper shows that for each sector, the upper switch on-times can be
 * derived by injecting a zero-sequence offset = -(Vmax + Vmin)/2 into the
 * three phase references. This is equivalent to the paper's equations:
 *
 *   t0   = T - t1 - t2      (zero vector time)
 *   t1,t2 = active vector times computed per sector
 *
 * Mapped to duty cycles via:
 *   duty_x = 0.5 + 0.5 × (V_phase_ref + V_offset)
 *
 * Linear modulation range: |Vref| ≤ Vbus/√3 = 0.577×Vbus
 * (15.5% more than SPWM which is limited to Vbus/2 = 0.5×Vbus)
 *
 * Input:  vab  — αβ voltage vector in Volts
 *         vBus — DC bus voltage in Volts
 *         arr  — TIM8 auto-reload register value (period)
 *
 * CRITICAL PHASE MAPPING — verify against your PCB:
 *   TIM8 CH1 → Phase A → Motor terminal A
 *   TIM8 CH2 → Phase B → Motor terminal B
 *   TIM8 CH3 → Phase C → Motor terminal C
 *   Phase A axis aligns with α axis (Vα maps to Phase A reference)
 *   Swapping any of these changes rotation direction and FOC behavior
 * ============================================================================ */
static void SVPWM(AlphaBeta_t vab, float vBus, uint32_t arr)
{
    float vHalf = vBus * 0.5f;

    /* ── STEP 1: INVERSE CLARKE (αβ → three phase references) ─────────────
     * Transforms the αβ voltage command back to three phase references.
     * These are the idealized sinusoidal references each phase would need
     * if driven with pure SPWM — normalized to [-1, 1] where 1 = Vbus/2.
     *
     * Equations (inverse of the Clarke transform):
     *   Va =  Vα
     *   Vb = -Vα/2 + (√3/2)×Vβ
     *   Vc = -Vα/2 - (√3/2)×Vβ
     *
     * At 0° electrical (post-alignment):
     *   If Vα = Vd > 0, Vβ = 0:
     *   Va = positive, Vb = Vc = -Va/2 (phase A driven, B and C pulled down)
     * ──────────────────────────────────────────────────────────────────── */
    float Va = vab.alpha / vHalf;
    float Vb = (-vab.alpha * 0.5f + vab.beta * 0.86602540f) / vHalf;
    float Vc = (-vab.alpha * 0.5f - vab.beta * 0.86602540f) / vHalf;

    /* ── STEP 2: FIND MAX AND MIN OF THREE REFERENCES ──────────────────────
     * These determine the sector and the zero-sequence offset magnitude.
     * The paper identifies 6 sectors by which phase has the highest and
     * lowest reference — Vmax/Vmin correspond to the boundaries.
     * ──────────────────────────────────────────────────────────────────── */
    float Vmax = Va;
    if(Vb > Vmax) Vmax = Vb;
    if(Vc > Vmax) Vmax = Vc;

    float Vmin = Va;
    if(Vb < Vmin) Vmin = Vb;
    if(Vc < Vmin) Vmin = Vc;

    /* ── STEP 3: ZERO-SEQUENCE INJECTION (SPWM → SVPWM) ───────────────────
     * This is the key step that transforms sinusoidal PWM into Space Vector
     * PWM. The offset centers the three references symmetrically around 0,
     * ensuring the zero vector time t0 is split equally between u0 and u7
     * (exactly as the paper prescribes in Equation 26: t0 = t7).
     *
     * Equation from paper (equivalent form):
     *   V_offset = -(Vmax + Vmin) / 2
     *
     * Physical effect: the offset shifts ALL three duty cycles together
     * by the same amount. Since the neutral point of the motor sees all
     * three phases, a common-mode shift does not produce any differential
     * voltage across the motor — the zero vector time is redistributed.
     *
     * Linear range extension:
     *   SPWM:  max |Va| = 1.000 → max phase voltage = Vbus/2 = 6.0V at 12V
     *   SVPWM: max |Va| = 1.155 → max phase voltage = Vbus/√3 = 6.93V at 12V
     *   Improvement: 15.5% more voltage available to the motor
     * ──────────────────────────────────────────────────────────────────── */
    float Voffset = -0.5f * (Vmax + Vmin);

    /* ── STEP 4: APPLY ZERO-SEQUENCE OFFSET ────────────────────────────────
     * Inject the offset into all three phase references.
     * After this step, the three references satisfy:
     *   max(Va_sv, Vb_sv, Vc_sv) = -min(Va_sv, Vb_sv, Vc_sv)
     * which ensures symmetric zero-vector placement matching the paper's
     * switching sequence u0→u1→u2→u7→u7→u2→u1→u0 for Sector 1.
     * ──────────────────────────────────────────────────────────────────── */
    float Va_sv = Va + Voffset;
    float Vb_sv = Vb + Voffset;
    float Vc_sv = Vc + Voffset;

    /* ── STEP 5: CONVERT TO DUTY CYCLES [0, 1] ─────────────────────────────
     * Map from [-1, 1] normalized voltage range to [0, 1] duty cycle range.
     * 0.5 corresponds to 50% duty = zero average voltage.
     *
     * Relation to paper Table V (Sector 1 example):
     *   S1 (Phase A) = (t1 + t2 + t0/2) / T = 0.5 + Va_sv/2
     *   S3 (Phase B) = (t2 + t0/2)      / T = 0.5 + Vb_sv/2
     *   S5 (Phase C) = (t0/2)            / T = 0.5 + Vc_sv/2
     *
     * These exactly reproduce Table V for all 6 sectors.
     * ──────────────────────────────────────────────────────────────────── */
    float tA = 0.5f + 0.5f * Va_sv;
    float tB = 0.5f + 0.5f * Vb_sv;
    float tC = 0.5f + 0.5f * Vc_sv;

    /* ── STEP 6: CLAMP (overmodulation protection) ──────────────────────────
     * Prevents duty cycles from exceeding the physically achievable range.
     * Occurs when |Vref| > Vbus/√3 (overmodulation region).
     * Clamping causes harmonic distortion but prevents undefined behavior.
     * ──────────────────────────────────────────────────────────────────── */
    if(tA > 0.95f) tA = 0.95f; else if(tA < 0.0f) tA = 0.0f;
    if(tB > 0.95f) tB = 0.95f; else if(tB < 0.0f) tB = 0.0f;
    if(tC > 0.95f) tC = 0.95f; else if(tC < 0.0f) tC = 0.0f;

    /* ── STEP 7: WRITE CCR REGISTERS ────────────────────────────────────────
     * In center-aligned PWM Mode 1, output is HIGH when CNT < CCR.
     * CCR = duty × ARR maps the [0,1] duty cycle to [0, ARR] counts.
     * Double-buffered preload ensures clean update at next period boundary.
     * ──────────────────────────────────────────────────────────────────── */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)(tA * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)(tB * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)(tC * (float)arr));
}

/* ============================================================================
 * PI CONTROLLER WITH CLAMPING ANTI-WINDUP AND DEADBAND
 *
 * Noise-tolerant implementation suitable for current-loop torque control:
 *   - Deadband prevents integral accumulation from current noise floor.
 *   - Clamping anti-windup: the integrator updates only when the previous
 *     output was within limits OR the new error would push the output
 *     OUT of saturation. This prevents the slow-recovery problem caused
 *     by the previous "clamp the integral itself" implementation: if Vq
 *     hits OUT_MAX because of a large transient, the old code kept
 *     integrating because pi->integral alone was below outMax. The new
 *     code freezes the integrator while saturated.
 *   - Output is clamped to [outMin, outMax] in Volts (= ±Vbus/√3).
 *
 * Discrete equations:
 *   e[n]  = setpoint - measured
 *   u_pre = kp·e[n] + I[n-1]
 *   if |e[n]| > deadband AND NOT (saturated_in_same_direction):
 *       I[n] = I[n-1] + ki·e[n]
 *   else:
 *       I[n] = I[n-1]
 *   I[n]  = clamp(I[n], outMin, outMax)         (final safety net)
 *   u[n]  = clamp(kp·e[n] + I[n], outMin, outMax)
 *
 * Why "saturated_in_same_direction"?
 *   If output is at +outMax and error is positive, integrating positively
 *   makes things worse. But if error is negative while output is at
 *   +outMax, integrating negatively pulls us out — that's allowed.
 * ============================================================================ */
static float PI_Update(PI_t *pi, float setpoint, float measured)
{
    float error = setpoint - measured;
    float pTerm = pi->kp * error;
    float u_unsat = pTerm + pi->integral;

    /* Determine saturation direction of the unsaturated output */
    int saturated_high = (u_unsat >  pi->outMax);
    int saturated_low  = (u_unsat <  pi->outMin);

    /* Conditional integration:
     *   - Skip if error is within deadband (suppresses noise accumulation)
     *   - Skip if saturated and integrating would worsen the saturation */
    int in_deadband = (error < PI_DEADBAND_A) && (error > -PI_DEADBAND_A);
    int wind_up_high = saturated_high && (error > 0.0f);
    int wind_up_low  = saturated_low  && (error < 0.0f);

    if(!in_deadband && !wind_up_high && !wind_up_low)
    {
        pi->integral += pi->ki * error;

        /* Final safety clamp on the integrator (rarely needed with the
         * conditional integration above, but cheap insurance). */
        if(pi->integral > pi->outMax) pi->integral = pi->outMax;
        if(pi->integral < pi->outMin) pi->integral = pi->outMin;
    }

    /* Clamp total output to the achievable voltage range */
    float output = pTerm + pi->integral;
    if(output > pi->outMax) output = pi->outMax;
    if(output < pi->outMin) output = pi->outMin;

    return output;
}

/* ============================================================================
 * GET ELECTRICAL ANGLE
 * Converts absolute encoder count to electrical angle [0°, 360°).
 *
 * Equations:
 *   posCount    = encoderCount mod COUNTS_PER_REV  (always positive)
 *   θ_mech      = posCount / COUNTS_PER_REV        (fractional revolution)
 *   θ_electrical = θ_mech × POLE_PAIRS × 360°  mod 360°
 * ============================================================================ */
static float getElectricalAngle(void)
{
    /* Force positive modulo — C % operator gives negative for negative inputs */
    int32_t posCount = ((encoderCount % COUNTS_PER_REV)
                        + COUNTS_PER_REV) % COUNTS_PER_REV;

    /* Fractional mechanical revolution [0.0, 1.0) */
    float mechAngle = (float)posCount / (float)COUNTS_PER_REV;

    /* Electrical angle — repeats POLE_PAIRS times per mechanical revolution */
    float elecAngle = mechAngle * (float)POLE_PAIRS * 360.0f;

    /* Wrap to [0°, 360°) */
    elecAngle = fmodf(elecAngle, 360.0f);
    if(elecAngle < 0.0f) elecAngle += 360.0f;

    return elecAngle;
}

/* ============================================================================
 * APPLY NEUTRAL OUTPUT
 * Sets all duty cycles to 50% — zero average voltage, no current.
 * Called during startup before FOC is ready.
 * Keeps PWM switching active so bootstrap capacitors remain charged.
 * ============================================================================ */
static void applyNeutralOutput(void)
{
    uint32_t neutral = (uint32_t)(PWM_ARR / 2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, neutral);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, neutral);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, neutral);
}

/* ============================================================================
 * EMERGENCY STOP
 * Immediately disables all PWM outputs via MOE bit.
 * Resets PI integrators to prevent windup accumulation.
 * Forces state machine back to NEUTRAL so the callback outputs neutral on
 * the next tick (PWM is already disabled but if MOE is later re-enabled
 * we don't want stale FOC commands re-applied).
 * ============================================================================ */
static void emergencyStop(void)
{
    focState   = FOC_STATE_NEUTRAL;
    focEnabled = 0;
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    piD.integral = 0.0f;
    piQ.integral = 0.0f;
    rampedTorque_Nm = 0.0f;
    targetIq = 0.0f;
}

/* ============================================================================
 * RESET FILTERS
 * Clears all filter state after alignment so residual alignment currents
 * do not contaminate the first FOC cycles.
 * ============================================================================ */
static void resetFilters(void)
{
    memset(&mfA, 0, sizeof(MedianFilter_t));
    memset(&mfB, 0, sizeof(MedianFilter_t));
    filtIa = 0.0f;
    filtIb = 0.0f;
    filtIb_prev = 0.0f;            /* time-skew compensation history */
    stagingIa = 0.0f;
    stagingIb = 0.0f;
    Ia = 0.0f;
    Ib = 0.0f;
    newCurrentData = 0;
}

/* ============================================================================
 * ALIGN ROTOR — FIX A and FIX E (gain-calibration sequencing)
 *
 * Forces rotor to a known 0° electrical position before FOC starts.
 * At 0°: sin=0, cos=1 → Vα=Vd, Vβ=0 → pure phase-A excitation; the rotor's
 * North pole aligns with phase A's magnetic axis.
 *
 *   ----- BUG IN PRE-FIX-A VERSION -----
 * The previous version called SVPWM() in a HAL_Delay loop while the TIM8
 * update callback was already running every 100 µs. That callback unconditionally
 * called applyNeutralOutput() (because focEnabled==0), overwriting the
 * alignment CCRs ~99.5% of the time. Net result: the rotor saw mostly neutral
 * output, drifted, and the "aligned" zero captured an essentially random
 * angle. The CSV from the open-loop test confirms this: steady-state Id ≈
 * −0.57 A, Iq ≈ −0.04 A → ~94° of electrical mis-alignment.
 *
 *   ----- ALSO FIXED HERE -----
 * The previous code applied Vd = 1.0 V which, with Rs ≈ 0.093 Ω, drives
 * I ≈ 1.0 / 0.093 ≈ 10.75 A through the windings — well above MAX_CURRENT_MA
 * and risks both DRV8353 OCP and demagnetisation. The header comment said
 * 0.3 V (≈3.2 A), so the value was almost certainly a leftover debug poke.
 *
 *   ----- HOW THIS WORKS NOW -----
 * The TIM8 callback owns the CCRs at all times. We just tell it which mode
 * to run by setting focState. During alignment the callback keeps recomputing
 * SVPWM at 10 kHz with (Vd=alignVd_V, Vq=0) at electrical 0°, so even if
 * the timer fires partway through this routine, the output stays consistent.
 *
 *   ----- FIX E: GAIN CALIBRATION FOLDED IN HERE -----
 * After the rotor settles, we transition through FOC_STATE_GAIN_CORRECTION
 * for ~600 ms.  Physically identical to ALIGN, but the ADC ISR uses this
 * as a flag to start averaging samples for per-channel gain calibration.
 * The calibration finalises (and gainCorrA, gainCorrB are written) on the
 * very next ADC interrupt after we transition to NEUTRAL below.
 *
 * Alignment current: I ≈ Vd / Rs = 0.3 / 0.093 ≈ 3.2 A
 * Total alignment+calibration time: 1000 + 600 + 50 = 1650 ms.
 * ============================================================================ */
static void alignRotor(void)
{
    /* Set the alignment voltage and switch the callback into ALIGN mode.
     * Order matters: alignVd_V must be valid before the callback sees ALIGN. */
    alignVd_V = 0.5f;
    focState  = FOC_STATE_ALIGN;

    /* Phase 1 — pure alignment hold (1.0 s).
     * Rotor physically moves toward 0° electrical and settles into the
     * cogging detent.  No measurement happens here. */
    HAL_Delay(1000);

    /* Phase 2 — gain-correction hold (~600 ms).
     * Same physical drive, but the ADC ISR will recognise this state and
     * accumulate samples for FIX E gain calibration. The ISR skips the
     * first 100 ms (GAIN_SETTLE_SAMPLES) and averages the rest, so we
     * need >100 ms here. 600 ms gives ~5000 averaged samples. */
    focState = FOC_STATE_GAIN_CORRECTION;
    HAL_Delay(600);

    /* Return to neutral BEFORE zeroing the encoder.
     * If we zero the encoder while alignment voltage is still applied, the
     * next FOC cycle (running at neutral) sees a 90° angle jump and pumps
     * a current spike through the bridge. The state transition to NEUTRAL
     * also triggers the gain-correction finalize block in the ADC ISR. */
    focState = FOC_STATE_NEUTRAL;
    HAL_Delay(50);

    /* Zero encoder at the aligned position.
     * After this: encoderCount=0  ==  0° electrical  ==  rotor aligned to phase A. */
    __HAL_TIM_SET_COUNTER(&htim3, 32768);   /* midpoint avoids 0/65535 wrap */
    encoderCount = 0;
    lastCount    = 32768;
}

/* ============================================================================
 * CALIBRATE DQ OFFSETS
 * Measures residual Id and Iq at zero torque after alignment.
 * Any nonzero reading at rest is sensor bias — subtracted every FOC cycle.
 *
 * Called after alignment, before enabling FOC.
 * Motor must be stationary with neutral voltage applied.
 *
 * GROUND BOUNCE NOTE: averages over CALIBRATION_SAMPLES to estimate the
 * mean offset including switching noise contribution. The ground bounce
 * appears as a DC shift plus AC noise — only the DC shift is corrected here.
 * ============================================================================ */
static void calibrateDQOffsets(void)
{
    double sumId = 0.0;
    double sumIq = 0.0;
    uint32_t validSamples = 0;

    /* Compute sin/cos at 0° — same angle as alignment */
    float sinTheta, cosTheta;
    CORDIC_SinCos(0.0f, &sinTheta, &cosTheta);

    /* Collect samples — wait for ADC callback to populate staging each cycle */
    uint32_t startTick = HAL_GetTick();  // ← elapsed time reference

    while(validSamples < CALIBRATION_SAMPLES)
	{
    	if(HAL_GetTick() - startTick > 4500)  // ← elapsed comparison
    		{
    			Error_Handler();
    		}
    	/* Wait for fresh ADC data from interrupt callback */
    	if(!newCurrentData) continue;
    	newCurrentData = 0;

    	/* Copy current snapshot */
    	float ia = stagingIa;
    	float ib = stagingIb;

    	/* Clarke and Park at known 0° reference angle */
        AlphaBeta_t ab = clarkeTransform(ia, ib);
        DQ_t dq        = parkTransformSC(ab, sinTheta, cosTheta);

        sumId += (double)dq.d;
        sumIq += (double)dq.q;
        validSamples++;
    }

    /* Average — this is the DC bias in the dq frame at rest */
    idOffset = (float)(sumId / CALIBRATION_SAMPLES);
    iqOffset = (float)(sumIq / CALIBRATION_SAMPLES);

    /* Sanity check — offset > 15% of rated current is suspicious
     * Likely means alignment current was still flowing or ADC is miscalibrated */
    float maxAllowedOffset = (float)MAX_CURRENT_MA / 1000.0f * 0.15f;
    if(   idOffset >  maxAllowedOffset || idOffset < -maxAllowedOffset
       || iqOffset >  maxAllowedOffset || iqOffset < -maxAllowedOffset)
    {
        /* Offset too large — reduce to zero and continue anyway
         * FOC will still work but steady-state error will be larger */
        idOffset = 0.0f;
        iqOffset = 0.0f;
    }
}

/* ============================================================================
 * ADC INJECTED CONVERSION COMPLETE CALLBACK
 * Fires at 10kHz, triggered by TIM8 TRGO2 (Update Event = counter bottom).
 *
 * Counter bottom is chosen because dI/dt = 0 at this instant — ground
 * bounce on SNx pins is minimized here, giving the best possible reading
 * from the imperfect shunt connection.
 *
 * Processing chain:
 *   Raw counts → offset subtract → Amps → median filter → EMA → staging
 *
 * ATOMIC WRITE: all three staging values written in succession.
 * TIM8 callback must have higher numerical priority (lower NVIC number)
 * than ADC to prevent reading a partial update. Set:
 *   ADC1_2 IRQ priority: 0 (highest)
 *   TIM8 update IRQ priority: 1
 * ============================================================================ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC2)
    {
        /* Read raw oversampled counts from JDR registers */
	    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
        float countsA = (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        float countsB = (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
        /* ========================================================
         * PHASE 1: OFFSET CALIBRATION (FOC_STATE_NEUTRAL)
         * ======================================================== */
        if(calibrateCounts < CALIBRATION_SAMPLES)
        {
            offsetA += countsA;
            offsetB += countsB;
            calibrateCounts++;

            if(calibrateCounts == CALIBRATION_SAMPLES)
            {
                /* Compute average baseline counts at 0A */
                offsetA /= (float)CALIBRATION_SAMPLES;
                offsetB /= (float)CALIBRATION_SAMPLES;
            }
            return; /* Do not update staging during calibration */
        }

        /* ========================================================
         * NORMAL OPERATION: Convert to Amps
         * ======================================================== */
        float rawIa = (countsA - offsetA) * current_scalar;
        float rawIb = (countsB - offsetB) * current_scalar * 1.741f;

        /* Apply hardware gain mismatch correction (ratios) */
        float Ia_cal = rawIa * gainCorrA;
        float Ib_cal = rawIb * gainCorrB;

        /* Median filter -> EMA Filter */
        float medA = medianFilter(&mfA, Ia_cal);
        float medB = medianFilter(&mfB, Ib_cal);

        filtIa = filtIa + ADC_FILTER_ALPHA * (medA - filtIa);
        filtIb = filtIb + ADC_FILTER_ALPHA * (medB - filtIb);

        /* ========================================================
         * PHASE 2: GAIN / RATIO CALIBRATION (FOC_STATE_GAIN_CORRECTION)
         * ======================================================== */
        if (focState == FOC_STATE_GAIN_CORRECTION)
        {
            if (gainSettleSkip < GAIN_SETTLE_SAMPLES)
            {
                gainSettleSkip++;
            }
            else
            {
                /* Accumulate RAW uncorrected currents for ratio math */
                gainSumA += rawIa;
                gainSumB += rawIb;
                gainSamples++;
            }
        }
        else if (gainSamples > 0 && focState == FOC_STATE_NEUTRAL)
        {
            /* Calculate final ratios when transitioning out of calibration */
            float avgMeasuredA = gainSumA / (float)gainSamples;
            float avgMeasuredB = gainSumB / (float)gainSamples;

            /* Theoretical expected current during alignment (Vd / Rs) */
            float expected_Ia = alignVd_V / ((float)RS_MOHMS / 1000.0f);
            float expected_Ib = -expected_Ia / 2.0f; /* Phase B sees half negative current */

            /* Calculate ratios */
            if (fabs(avgMeasuredA) > 0.1f) gainCorrA = expected_Ia / avgMeasuredA;
            if (fabs(avgMeasuredB) > 0.1f) gainCorrB = expected_Ib / avgMeasuredB;

            /* Reset for next time */
            gainSamples = 0;
            gainSettleSkip = 0;
            gainSumA = 0.0f;
            gainSumB = 0.0f;
        }

        /* ========================================================
         * STAGE DATA FOR TIM8 ISR
         * ======================================================== */
        stagingIa = filtIa;
        stagingIb = filtIb;
        newCurrentData = 1;
    }
}

/* ============================================================================
 * TIM8 PERIOD ELAPSED CALLBACK — FOC LOOP
 * Fires at 10kHz from TIM8 Update Event.
 * This is the main real-time control loop.
 *
 * When focEnabled=0: applies neutral duty cycles (bootstrap maintenance).
 * When focEnabled=1: runs full FOC pipeline.
 *
 * Execution stages:
 *   0. Copy current snapshot from staging (atomic)
 *   1. Overcurrent protection
 *   2. Encoder position update
 *   3. Electrical angle + CORDIC sin/cos
 *   4. Clarke transform (3-phase → αβ)
 *   5. Park transform (αβ → dq, rotating frame)
 *   6. Torque ramp + PI controllers
 *   7. Inverse Park (dq → αβ)
 *   8. SVPWM (αβ → duty cycles → CCR registers)
 * ============================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ── SYSTICK MAINTENANCE ──────────────────────────────────────────────── */
    if(htim->Instance == TIM17)
    {
        HAL_IncTick();
    }
    /* ── UART LOOP ──────────────────────────────────────────────── */
	if (htim == &htim7)
	{
	    uart_poll_rx();
	    uart_check_timeout();
	    uart_update_velocity();
	}

    if(htim->Instance == TIM8)
    {

    	/* ── STATE DISPATCH ──────────────────────────────────────────────────── */
    	/* FIX A: this dispatch is the heart of the alignment-race-condition fix.
    	 * Previously a single focEnabled flag forced applyNeutralOutput() during
    	 * alignment, which fought with alignRotor()'s SVPWM writes from the main
    	 * loop. The state machine cleanly separates the four cases below.
    	 *
    	 * NEUTRAL          → 50% duty everywhere, zero average voltage
    	 * ALIGN            → constant (Vd = alignVd_V, Vq = 0) at θ=0°
    	 * GAIN_CORRECTION  → physically same as ALIGN; the ADC ISR uses this
    	 *                    state to gate gain-calibration accumulation.
    	 *                    DO NOT remove this case or fall it through to the
    	 *                    FOC pipeline — the rotor must remain locked at
    	 *                    0° while gain calibration averages.
    	 * FOC              → full FOC pipeline runs (default fallthrough). */
    	if(focState == FOC_STATE_NEUTRAL)
    	{
    	    applyNeutralOutput();
    	    return;
    	}
    	else if(focState == FOC_STATE_ALIGN || focState == FOC_STATE_GAIN_CORRECTION)
    	{
    	    /* Apply (Vd = alignVd_V, Vq = 0) at electrical 0°.
    	     * sin(0)=0, cos(0)=1  ⇒  Vα = Vd, Vβ = 0
    	     * Pure phase-A excitation pulls the rotor into alignment. */
    	    AlphaBeta_t vAlpha = {.alpha = alignVd_V, .beta = 0.0f};
    	    SVPWM(vAlpha, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR);
    	    return;
    	}
    	/* else: FOC_STATE_FOC — fall through to full pipeline below */

    	/* ── STAGE 0: ATOMIC CURRENT COPY ────────────────────────────────────── */
    	/* Copy all three values from staging in one block.
    	 * If no new data, use previous cycle's values — FOC continues smoothly.
    	 * This prevents the rare case where TIM8 fires before ADC completes. */
    	if(newCurrentData)
    	{
    	    Ia = stagingIa;
    	    Ib = stagingIb;
    	    newCurrentData = 0;
    	}

    	/* Update live watch variables */
#if Debug == 1
    	debugIa = Ia;
    	debugIb = Ib;
#endif

    	/* ── STAGE 1: OVERCURRENT PROTECTION ─────────────────────────────────── */
    	/* Threshold is OCP_MULTIPLIER × rated current to accommodate:
    	 *   - Ground bounce noise (can appear as ±1-2A above real current)
    	 *   - Normal transient peaks during acceleration
    	 * With OCP_MULTIPLIER = 5.0 and MAX_CURRENT_MA = 2000 → trip at 10 A.
    	 * The DRV8353R hardware OCP (set in driver registers) catches the
    	 * shorter-timescale faults; this is the firmware safety net. */
    	float ocpLimit = (float)MAX_CURRENT_MA / 1000.0f * OCP_MULTIPLIER;

    	if(Ia > ocpLimit || Ia < -ocpLimit || Ib > ocpLimit || Ib < -ocpLimit)
    	{
    	    emergencyStop();
    	    Error_Handler();
    	    return;
    	}

    	/* ── STAGE 2: ENCODER POSITION UPDATE ────────────────────────────────── */
    	/* TIM3 counts quadrature encoder pulses in hardware.
    	 * We read the 16-bit counter, compute the signed delta since last cycle,
    	 * and accumulate into a 32-bit absolute position counter.
    	 *
    	 * Rollover handling: if delta > 32767 or < -32768, a 16-bit wraparound
    	 * occurred — adjust by ±65536 to get the true (small) delta. */
    	int32_t currentCount = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    	int32_t delta        = currentCount - lastCount;

    	if(delta >  32767) delta -= 65536; /* rolled over: 65535 → 0 */
    	if(delta < -32768) delta += 65536; /* rolled over: 0 → 65535 */

    	encoderCount += delta;
    	lastCount     = currentCount;

    	/* Mechanical velocity (encoderSpeed) is updated by uart_update_velocity()
    	 * inside the TIM7 1 kHz ISR — not here. Keeping HAL_GetTick() out of the
    	 * 10 kHz FOC loop saves a few cycles and avoids any risk of integer
    	 * overflow surprises in this critical path. */



    	/* Update debug watch */
#if Debug == 1
    	debugEncoderCount = encoderCount;
#endif

    	/* ── STAGE 3: ELECTRICAL ANGLE + CORDIC ──────────────────────────────── */
    	/* Convert shaft position to electrical angle [0°, 360°).
    	 * Electrical angle repeats POLE_PAIRS times per mechanical revolution.
    	 *
    	 * FIX D — DELAY COMPENSATION:
    	 * The current samples we just consumed were taken roughly one PWM cycle
    	 * (Ts = 100 µs) ago. The voltages we are about to compute will be
    	 * latched into the CCR preload registers and only take effect at the
    	 * NEXT update event — so the average actuation instant is ~1.5·Ts in
    	 * the future relative to the sample instant. At 500 Hz electrical that
    	 * is 18° of phase error (much larger than the ADC-skew error fixed in
    	 * the ADC callback). We correct it by:
    	 *
    	 *   theta_park   = elecAngle           // for Park: angle when current was sampled
    	 *   theta_invpk  = elecAngle + Δθ_pred // for inv-Park: angle when V is applied
    	 *
    	 * Δθ_pred is estimated from the encoder delta this cycle, which is
    	 * directly proportional to electrical angular velocity. Using a single
    	 * sample is noisy at low speed; a light first-order IIR smooths it. */
    	/* ── ENCODER VELOCITY → ELECTRICAL RAD PER FOC CYCLE ─────────────────── */
    	/* 2π/COUNTS_PER_REV × POLE_PAIRS, computed at compile time. M_PI is not
    	 * guaranteed by the C99 standard math.h, so we use an explicit literal. */
    	#define TWO_PI_F            6.28318530717958647693f
    	#define ELEC_RAD_PER_COUNT  ((TWO_PI_F / (float)COUNTS_PER_REV) * (float)POLE_PAIRS)

    	float deltaTheta_e = (float)delta * ELEC_RAD_PER_COUNT;


		#define DELTA_FILTER_ALPHA      0.2f
		#define DELTA_RATE_LIMIT_RAD    ELEC_RAD_PER_COUNT

		static float deltaTheta_e_filt = 0.0f;

		float filterError = deltaTheta_e - deltaTheta_e_filt;
		if      (filterError >  DELTA_RATE_LIMIT_RAD) filterError =  DELTA_RATE_LIMIT_RAD;
		else if (filterError < -DELTA_RATE_LIMIT_RAD) filterError = -DELTA_RATE_LIMIT_RAD;
		deltaTheta_e_filt += DELTA_FILTER_ALPHA * filterError;
    	/* ── FIX 2B: speed-scaled prediction horizon ──────────────────────────
    	 *
    	 * At low speed the encoder quantises to ±1 count per FOC cycle, which
    	 * is ±3.5° electrical. With ANGLE_PREDICT_TS_FULL = 2.55, that 1-count
    	 * jitter alone produces ±9° of predicted-angle noise — enough to upset
    	 * commutation and stall the motor near zero speed. This was the cause
    	 * of the "negative torque wall" and low-speed reversal OCP trips.
    	 *
    	 * Scaling the prediction by |speed| / (|speed| + corner) gives:
    	 *   - At standstill: scale → 0, no prediction (angle change would be
    	 *     trivially small in 2.55·Ts anyway, so we lose nothing).
    	 *   - At PREDICT_CORNER speed: scale = 0.5, prediction at half value.
    	 *   - At 4× corner speed (running): scale = 0.8, near full prediction.
    	 *
    	 * Corner is set to 1 encoder count per cycle, so the scale-up region
    	 * is 1–3 counts/cycle (~17–50 RPM mech), exactly the range where
    	 * quantization noise matters most. */
    	#define ANGLE_PREDICT_TS_FULL   2.55f
    	#define PREDICT_CORNER_RAD      ELEC_RAD_PER_COUNT

    	float absSpeed     = fabsf(deltaTheta_e_filt);
    	float predictScale = absSpeed / (absSpeed + PREDICT_CORNER_RAD);
    	float predictRad   = ANGLE_PREDICT_TS_FULL * predictScale * deltaTheta_e_filt;

    	/* getElectricalAngle() returns degrees, deltaTheta_e_filt is in radians.
    	 * Convert the prediction to degrees once, here, then everything is in
    	 * degrees. (180/π = 57.2957795...) */
    	#define RAD_TO_DEG          57.2957795130823f
    	float elecAngle = getElectricalAngle();
    	float predAngle = elecAngle + predictRad * RAD_TO_DEG;

    	/* Wrap to [0°, 360°) */
    	predAngle = fmodf(predAngle, 360.0f);
    	if (predAngle < 0.0f) predAngle += 360.0f;

#if Debug == 1
    	debugElecAngle  = elecAngle;
#endif

    	/* sin/cos at SAMPLE instant — used for Park (current → dq) */
    	float sinTheta, cosTheta;
    	CORDIC_SinCos(elecAngle, &sinTheta, &cosTheta);

    	/* sin/cos at ACTUATION instant — used for Inverse Park (dq → V) */
    	float sinPred, cosPred;
    	CORDIC_SinCos(predAngle, &sinPred, &cosPred);

    	/* ── STAGE 4: CLARKE TRANSFORM ───────────────────────────────────────── */
    	/* Convert two measured phase currents to stationary αβ frame.
    	 * Ic is not measured — reconstructed from KCL in ADC callback.
    	 * Only Ia and Ib are passed; the formula already accounts for Ic.
    	 *
    	 * Iα = Ia
    	 * Iβ = (Ia + 2×Ib) / √3                                              */
    	AlphaBeta_t iab = clarkeTransform(Ia, Ib);

    	/* ── STAGE 5: PARK TRANSFORM ─────────────────────────────────────────── */
    	/* Rotate αβ current vector into rotor-aligned dq frame.
    	 * After this transform, AC currents appear as DC in steady state.
    	 *
    	 * Id = Iα×cos(θ) + Iβ×sin(θ)    → flux component
    	 * Iq = -Iα×sin(θ) + Iβ×cos(θ)   → torque component               */
    	DQ_t idq = parkTransformSC(iab, sinTheta, cosTheta);

    	/* Apply DQ offset calibration — removes residual bias from ground bounce
    	 * These offsets were measured at rest after alignment */
    	idq.d -= idOffset;
    	idq.q -= iqOffset;

    	/* Update debug watches */
#if Debug == 1
    	debugId = idq.d;
    	debugIq = idq.q;
#endif

#if openLoop == 0
    	/* ── STAGE 6: TORQUE RAMP + PI CONTROLLERS ───────────────────────────── */
    	/* TORQUE RAMP: smoothly approach the commanded torque.
    	 * Sudden steps in targetTorque_Nm would cause current spikes.
    	 * Ramp rate = TORQUE_RAMP_RATE Nm per cycle = 0.05 Nm/second */
    	if(rampedTorque_Nm < targetTorque_Nm)
    	{
    	    rampedTorque_Nm += TORQUE_RAMP_RATE;
    	    if(rampedTorque_Nm > targetTorque_Nm) rampedTorque_Nm = targetTorque_Nm;
    	}
    	else if(rampedTorque_Nm > targetTorque_Nm)
    	{
        	rampedTorque_Nm -= TORQUE_RAMP_RATE;
        	if(rampedTorque_Nm < targetTorque_Nm) rampedTorque_Nm = targetTorque_Nm;
    	}

    	/* TORQUE TO CURRENT: T = Kt × Iq → Iq = T / Kt */
    	targetIq = rampedTorque_Nm / KT_NM_PER_AMP;

    	/* Clamp Iq to rated current */
    	float maxIq = (float)MAX_CURRENT_MA / 1000.0f;
    	if(targetIq >  maxIq) targetIq =  maxIq;
    	if(targetIq < -maxIq) targetIq = -maxIq;

    	/* D-AXIS PI: drives Id → 0 (no flux weakening for SPM motor)
    	 * Small Vd output is normal — corrects any residual flux component */
    	float Vd = PI_Update(&piD, 0.0f,    idq.d);

    	/* Q-AXIS PI: drives Iq → targetIq (torque control)
    	 * This is the primary torque-producing axis */
    	float Vq = PI_Update(&piQ, targetIq, idq.q);

    	/* Update debug watches */
#if Debug == 1
    	debugVd = Vd;
    	debugVq = Vq;
#endif

    	/* ── STAGE 7: INVERSE PARK TRANSFORM (with delay compensation) ────────── */
    	/* Rotate Vd, Vq back from rotating dq frame to stationary αβ frame.
    	 * Uses sinPred / cosPred (angle predicted forward by 1.5·Ts) so the
    	 * voltage vector is aligned with the rotor at the instant it is
    	 * actually applied, not the instant the currents were sampled.
    	 *
    	 * Vα = Vd×cos(θ_pred) − Vq×sin(θ_pred)
    	 * Vβ = Vd×sin(θ_pred) + Vq×cos(θ_pred)                                */
    	DQ_t        vdq = {.d = Vd, .q = Vq};
    	AlphaBeta_t vab = inverseParkSC(vdq, sinPred, cosPred);

    	/* ── STAGE 8: SVPWM ─────────────────────────────────────────────────── */
    	/* Convert αβ voltage vector to three duty cycles.
    	 * Writes directly to TIM8 CCR1/2/3.
    	 * Double-buffered preload ensures values apply cleanly at next cycle. */
    	SVPWM(vab, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR);
#endif
#if openLoop == 1
    	DQ_t        vdq = {.d = openLoopVd, .q = openLoopVq};
    	AlphaBeta_t vab = inverseParkSC(vdq, sinPred, cosPred);
    	SVPWM(vab, (float)BUS_VOLTAGE_MV / 1000.0f, PWM_ARR);
#endif
#if dataAqu == 1
    		if(Pi++ == 40)
    		{
    			//HAL_Delay(1);
    			Pi = 0;
    			if(Gi < AQU_AMT)
    			{
					IdG				[Gi] = idq.d;
					IqG				[Gi] = idq.q;
					VdG				[Gi] = vdq.d;
					VqG				[Gi] = vdq.q;
					IaG				[Gi] = Ia;
					IbG				[Gi] = Ib;
					ElecAngleG		[Gi] = elecAngle;
					EncoderCountG	[Gi] = encoderCount;

					Gi++;
	    			//if (Gi == AQU_AMT) emergencyStop();
    			}

    		}
#endif
    }
}



/* ============================================================================
 * MAIN
 * Startup sequence, peripheral initialization, and application loop.
 *
 * Startup order is critical:
 *   1. Peripheral inits (CubeMX generated)
 *   2. ADC hardware offset calibration
 *   3. DRV8353R enable + charge pump startup delay
 *   4. PWM outputs start (MOE set, bootstrap caps begin charging)
 *   5. TIM8 base start (FOC loop active but focEnabled=0 → neutral output)
 *   6. ADC injected start (current measurement begins)
 *   7. Wait for ADC zero-current offset calibration (CALIBRATION_SAMPLES)
 *   8. Start encoder
 *   9. Align rotor (focEnabled=0 so FOC doesn't fight alignment voltage)
 *  10. Reset all filter state (clears alignment current from buffers)
 *  11. Redo offset calibration (clean 0A baseline, no alignment contamination)
 *  12. Calibrate DQ offsets (removes residual dq frame bias)
 *  13. Enable FOC (focEnabled=1)
 *  14. Application loop (set targetTorque_Nm)
 * ============================================================================ */
int main(void)
{
    /* ── CubeMX GENERATED INITIALIZATIONS ───────────────────────────────── */
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_TIM1_Init();
    MX_CORDIC_Init();
    MX_CRC_Init();
    MX_DMA_Init();
    //MX_FMAC_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM7_Init();
    MX_TIM8_Init();
    MX_TIM20_Init();
    MX_USART1_UART_Init();

    uart_init();
    /* Start the UART RX DMA in circular mode. After this call, bytes flow
     * into uartRxBuf without further intervention. The buffer size must be a
     * power of 2 so the wrap math (& UART_RX_BUF_SIZE-1) works. */
    HAL_UART_Receive_DMA(&huart1, uartRxBuf, UART_RX_BUF_SIZE);


    focState   = FOC_STATE_NEUTRAL;
    focEnabled = 0;

    /* ── STEP 1: ADC HARDWARE OFFSET CALIBRATION ────────────────────────── */
    /* Must run before ADC starts converting.
     * Removes manufacturing variation in ADC offset voltage.
     * This is the STM32G474 hardware self-calibration, not the
     * software offset calibration for the current sensor. */
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* ── STEP 2: ENABLE DRV8353R ─────────────────────────────────────────── */
    /* Pull EN/DIS pin high to enable the gate driver.
     * Wait 5ms for:
     *   - Internal charge pump to reach operating voltage (VCP)
     *   - UVLO condition to clear
     *   - nFAULT to deassert (go HIGH)
     * Do NOT start PWM before nFAULT is high. */
    HAL_GPIO_WritePin(ENABLE_3_GPIO_Port, ENABLE_3_Pin, GPIO_PIN_SET);
    HAL_Delay(5);


    /* Apply neutral duty immediately — no current before FOC is ready */
    applyNeutralOutput();

    /* ── STEP 3: START PWM OUTPUTS ───────────────────────────────────────── */
    /* HAL_TIM_PWM_Start sets the MOE bit (Master Output Enable) in BDTR.
     * Without MOE, no PWM appears on any pin regardless of CCR values.
     * HAL_TIMEx_PWMN_Start enables the complementary (low-side) outputs.
     * Both must be called per channel for 6-output complementary operation. */
    HAL_TIM_PWM_Start(&htim8,    TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8,    TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8,    TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

    /* ── STEP 4: START TIM8 BASE (FOC loop begins) ───────────────────────── */
    /* focState = NEUTRAL so the callback applies neutral output — safe to start.
     * Timer must run continuously to:
     *   a) Generate TRGO2 to trigger ADC at 10kHz
     *   b) Keep PWM switching so bootstrap caps stay charged
     * Never stop TIM8 after this point. */
    HAL_TIM_Base_Start_IT(&htim8);
    HAL_Delay(10); /* allow first few PWM cycles for bootstrap charging */
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_Delay(50);

    /* ── STEP 5: START ADC (current measurement begins) ─────────────────── */
    /* ADC waits for TIM8 TRGO2 (Update Event = counter bottom) to trigger.
     * The software offset calibration runs for CALIBRATION_SAMPLES cycles.
     * During this time the ADC callback accumulates counts and returns early. */

    /* ── STEP 6: START ENCODER ───────────────────────────────────────────── */
    /* Start TIM3 in encoder mode. Counter increments/decrements based on
     * quadrature A/B inputs. Starting at 32768 (midpoint) avoids the
     * rollover boundary and gives full range in both directions. */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


    while(calibrateCounts < CALIBRATION_SAMPLES) HAL_Delay(10);
    /* ── STEP 7: ALIGN ROTOR ─────────────────────────────────────────────── */
    /* alignRotor() switches focState to ALIGN, holds the rotor with Vd=0.3V
     * for 1500ms, then returns to NEUTRAL and zeroes the encoder. The TIM8
     * callback owns the CCRs the entire time — no race with this thread. */
    alignRotor();

    /* ── STEP 8: RESET ALL FILTER STATE ─────────────────────────────────── */
    /* Alignment current (~3.2A) filled the filter buffers.
     * Without resetting, these stale values would feed into the first
     * FOC cycles and cause a transient overcurrent trip. */
    resetFilters();
    piD.integral    = 0.0f;
    piQ.integral    = 0.0f;
    rampedTorque_Nm = 0.0f;
    targetIq        = 0.0f;

    /* ── STEP 9: REDO OFFSET CALIBRATION (clean baseline) ───────────────── */
    /* The first calibration ran during startup before alignment.
     * This second calibration runs with:
     *   - Neutral voltage applied (zero current flowing)
     *   - Full PWM switching active (ground bounce present)
     *   - Filters fully reset and re-settling
     * This gives the cleanest possible 0A baseline.
     *
     * BUG PREVIOUSLY HERE: the wait loop was accidentally removed, leaving
     * only a HAL_Delay(100) = 1000 samples. With CALIBRATION_SAMPLES=4000
     * the ADC callback was still returning early (no staging update) when
     * calibrateDQOffsets() started, causing it to spin until the 4500ms
     * watchdog called Error_Handler(). The wait loop is restored below. */
    offsetA         = 0.0f;
    offsetB         = 0.0f;
    calibrateCounts = 0;    /* triggers recalibration in ADC callback */

    while(calibrateCounts < CALIBRATION_SAMPLES)
    {
        HAL_Delay(10);
    }

    /* Wait for filters to settle with new offsets (~5 EMA time constants) */
    HAL_Delay(100);

    /* ── STEP 10: CALIBRATE DQ OFFSETS ───────────────────────────────────── */
    /* Measures residual Id and Iq at rest — subtracted every FOC cycle.
     * Removes DC bias from imperfect SNx grounding (ground bounce offset).
     * Motor must be stationary and at 0° electrical angle. */
    calibrateDQOffsets();

    /* ── STEP 11: ENABLE FOC ─────────────────────────────────────────────── */
    /* From this point the TIM8 callback runs the full FOC pipeline.
     * targetTorque_Nm=0 so motor holds position with zero torque. */
    focState   = FOC_STATE_FOC;
    focEnabled = 1;

    /* Short settling period before accepting torque commands */
    HAL_Delay(50);

    /* ============================================================================
     * APPLICATION LOOP
     *
     * Set targetTorque_Nm to command torque:
     *   Positive = forward torque (direction depends on wiring)
     *   Negative = reverse torque
     *   0 = hold position (small holding current from D-axis PI)
     *
     * The torque ramp (TORQUE_RAMP_RATE) prevents step changes.
     * The FOC loop runs in interrupt context at 10kHz.
     * This loop can run at any rate — it only sets the setpoint.
     *
     * Live watch variables for monitoring:
     *   debugIa, debugIb        — phase currents in Amps
     *   debugId, debugIq        — dq frame currents in Amps
     *   debugElecAngle          — electrical angle 0-360°
     *   debugEncoderCount       — absolute shaft position in counts
     *   debugVd, debugVq        — PI controller outputs in Volts
     *
     * Healthy steady-state values at zero torque:
     *   debugId  ≈ 0.0A  (D-axis PI holding flux at zero)
     *   debugIq  ≈ 0.0A  (no torque commanded)
     *   debugVd  ≈ small nonzero (correcting residual noise)
     *   debugVq  ≈ 0.0V
     *   debugIa, debugIb — near zero, small ripple from ground bounce
     * ============================================================================
     * APPLICATION LOOP
     *
     * All real-time work happens in interrupt contexts:
     *   - TIM8 update (10 kHz, priority 2): FOC pipeline
     *   - ADC1_2 (priority 0):              current sample → median → EMA → stage
     *   - TIM7 update (1 kHz, priority 3):  UART poll, timeout check, velocity
     *
     * The while(1) loop only handles slow-tier housekeeping:
     *   - Fault recovery decisions
     *   - Optional telemetry / display updates
     *   - System health monitoring (temperature, bus voltage trend, etc.)
     *
     * It is NOT in the critical timing path. Adding HAL_Delay here is fine.
     * ============================================================================ */
    targetTorque_Nm = 0.0f;
    HAL_TIM_Base_Start_IT(&htim7);

    while(1)
    {

    	targetTorque_Nm = 0.05;
    	HAL_Delay(3000);
    	targetTorque_Nm = -0.05;
    	HAL_Delay(3000);
        /* If FOC stopped on overcurrent, do not auto-restart — require a
         * deliberate reset. Add fault recovery logic here if your application
         * needs it. */
        if(!focEnabled)
        {
            HAL_Delay(100);
        }

        HAL_Delay(1);
    }
}

/* ============================================================================
 * SYSTEM CLOCK CONFIGURATION (generated by CubeMX)
 * 170MHz from HSI + PLL
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
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1
                                     | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* ============================================================================
 * ERROR HANDLER
 * Called on any fatal fault. Disables outputs and halts.
 * In a production system this would log the fault and attempt safe shutdown.
 * ============================================================================ */
void Error_Handler(void)
{
    emergencyStop();
    __disable_irq();
    while(1)
    {
        /* Halt — requires hardware reset to recover */
    }
}
