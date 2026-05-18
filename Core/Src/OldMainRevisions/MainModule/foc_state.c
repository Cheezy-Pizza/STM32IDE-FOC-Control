/* ============================================================================
 * foc_state.c — Definitions of all FOC global state, plus fault helpers.
 * ============================================================================ */
#include "foc_state.h"
#include "tim.h"


/* ============================================================================
 * ENCODER STATE
 * ============================================================================ */
volatile int32_t  encoderCount[MTR_AMT] = {0, 0};
volatile int32_t  lastCount[MTR_AMT]    = {0, 0};
volatile int32_t  encoderSpeed[MTR_AMT] = {0, 0};


/* ============================================================================
 * CURRENT-SENSE OFFSET CALIBRATION
 * ============================================================================ */
volatile float    offsetA[MTR_AMT]         = {0.0f, 0.0f};
volatile float    offsetB[MTR_AMT]         = {0.0f, 0.0f};
volatile uint32_t calibrateCounts[MTR_AMT] = {0, 0};

const float current_scalar = (float)VREF_MV
                           / ((float)ADC_COUNTS
                              * ((float)SHUNT_UOHMS / 1000.0f)
                              * (float)AMP_GAIN);


/* ============================================================================
 * PER-CHANNEL GAIN CORRECTION
 * ============================================================================ */
float gainCorrA[MTR_AMT] = {1.0f, 1.0f};
float gainCorrB[MTR_AMT] = {1.0f, 1.0f};


/* ============================================================================
 * ADC FILTER STATE
 * ============================================================================ */
MedianFilter_t mfA[MTR_AMT] = { { {0.0f}, 0 }, { {0.0f}, 0 } };
MedianFilter_t mfB[MTR_AMT] = { { {0.0f}, 0 }, { {0.0f}, 0 } };
float filtIa[MTR_AMT] = {0.0f, 0.0f};
float filtIb[MTR_AMT] = {0.0f, 0.0f};


/* ============================================================================
 * STAGING BUFFER
 * ============================================================================ */
volatile float   stagingIa[MTR_AMT]    = {0.0f, 0.0f};
volatile float   stagingIb[MTR_AMT]    = {0.0f, 0.0f};
volatile uint8_t newCurrentData[MTR_AMT] = {0, 0};


/* ============================================================================
 * FOC ISR WORKING COPIES + DQ OFFSET
 * ============================================================================ */
float Ia[MTR_AMT] = {0.0f, 0.0f};
float Ib[MTR_AMT] = {0.0f, 0.0f};

float idOffset[MTR_AMT] = {0.0f, 0.0f};
float iqOffset[MTR_AMT] = {0.0f, 0.0f};


/* ============================================================================
 * PI CONTROLLERS
 * ============================================================================ */
PI_t piD[MTR_AMT] = {
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX },
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX }
};
PI_t piQ[MTR_AMT] = {
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX },
    { .kp = 0.2f, .ki = 0.003f, .integral = 0.0f, .outMin = -OUT_MAX, .outMax = OUT_MAX }
};


/* ============================================================================
 * TORQUE COMMAND
 * ============================================================================ */
volatile float targetTorque_Nm[MTR_AMT] = {0.0f, 0.0f};
float          rampedTorque_Nm[MTR_AMT] = {0.0f, 0.0f};
float          targetIq[MTR_AMT]        = {0.0f, 0.0f};


/* ============================================================================
 * FOC STATE MACHINE
 * ============================================================================ */
volatile FocState_t focState[MTR_AMT]   = {FOC_STATE_NEUTRAL, FOC_STATE_NEUTRAL};
volatile uint8_t    focEnabled[MTR_AMT] = {0, 0};
float               alignVd_V[MTR_AMT]  = {0.0f, 0.0f};


/* ============================================================================
 * ALIGNMENT CONTROL
 * ============================================================================ */
volatile float alignSweepAngle_deg[MTR_AMT] = {0.0f, 0.0f};
volatile float encoderOffset_deg[MTR_AMT]   = {0.0f, 0.0f};


/* ============================================================================
 * FAULTS
 * ============================================================================ */
volatile uint32_t faultFlags = 0u;
volatile uint8_t  gainFaultA = 0;
volatile uint8_t  gainFaultB = 0;


/* ============================================================================
 * GAIN CALIBRATION ACCUMULATORS
 * ============================================================================ */
float    gainSumA[MTR_AMT]       = {0.0f, 0.0f};
float    gainSumB[MTR_AMT]       = {0.0f, 0.0f};
uint32_t gainSamples[MTR_AMT]    = {0, 0};
uint32_t gainSettleSkip[MTR_AMT] = {0, 0};


/* ============================================================================
 * DEBUG LIVE-WATCH
 * ============================================================================ */
#if Debug == 1
volatile float   debugId[MTR_AMT]              = {0.0f, 0.0f};
volatile float   debugIq[MTR_AMT]              = {0.0f, 0.0f};
volatile float   debugElecAngle[MTR_AMT]       = {0.0f, 0.0f};
volatile int32_t debugEncoderCount[MTR_AMT]    = {0, 0};
volatile float   debugVd[MTR_AMT]              = {0.0f, 0.0f};
volatile float   debugVq[MTR_AMT]              = {0.0f, 0.0f};
volatile float   debugIa[MTR_AMT]              = {0.0f, 0.0f};
volatile float   debugIb[MTR_AMT]              = {0.0f, 0.0f};
volatile float   debugProbeSpread[MTR_AMT]     = {0.0f, 0.0f};
volatile float   debugLinregSlope[MTR_AMT]     = {0.0f, 0.0f};
volatile float   debugLinregResidual[MTR_AMT]  = {0.0f, 0.0f};
#endif


/* ============================================================================
 * DATA ACQUISITION ARRAYS (M1 only, optional)
 * ============================================================================ */
#if dataAqu == 1
volatile float   IdG[AQU_AMT]           = {0};
volatile float   IqG[AQU_AMT]           = {0};
volatile float   ElecAngleG[AQU_AMT]    = {0};
volatile float   VdG[AQU_AMT]           = {0};
volatile float   VqG[AQU_AMT]           = {0};
volatile float   IaG[AQU_AMT]           = {0};
volatile float   IbG[AQU_AMT]           = {0};
volatile int32_t EncoderCountG[AQU_AMT] = {0};
#endif


/* ============================================================================
 * OPEN-LOOP CHARACTERISATION
 * ============================================================================ */
#if openLoop == 1
volatile float openLoopVd[MTR_AMT] = {0.0f, 0.0f};
volatile float openLoopVq[MTR_AMT] = {0.2f, 0.2f};
#endif


/* ============================================================================
 * UART TELEMETRY GLOBALS
 * ============================================================================ */
volatile uint32_t uartFramesAccepted = 0;
volatile uint32_t uartFramesDropped  = 0;
volatile uint32_t uartSyncHunts      = 0;
volatile uint32_t uartSeqErrors      = 0;
volatile uint8_t  isInit             = 0;
volatile float    pos_M1_Offset      = 0.0f;
volatile float    pos_M2_Offset      = 0.0f;


/* ============================================================================
 * FAULT HELPERS
 *
 * emergencyStop is a HARD fault — both motors latch FAULT, both PWM stages
 * disabled. It's intentional that one motor's hard fault disables the
 * other: we don't yet know which motor blew, and a stale PWM pattern on
 * the other side can cause its OWN fault if shoot-through is in progress.
 * ============================================================================ */
void emergencyStop(uint32_t reason)
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


void raiseSoftFault(uint32_t flag)
{
    faultFlags |= flag;
    if (flag & FAULT_GAIN_BAD_PHA) gainFaultA = 1;
    if (flag & FAULT_GAIN_BAD_PHB) gainFaultB = 1;
}


void clearSoftFault(uint32_t flag)
{
    faultFlags &= ~flag;
    if (flag & FAULT_GAIN_BAD_PHA) gainFaultA = 0;
    if (flag & FAULT_GAIN_BAD_PHB) gainFaultB = 0;
}
