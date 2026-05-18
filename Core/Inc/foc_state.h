/* ============================================================================
 * foc_state.h — Shared global state for FOC controller.
 *
 * All variables here are extern declarations. Definitions live in foc_state.c.
 * ============================================================================ */
#ifndef FOC_STATE_H
#define FOC_STATE_H

#include <stdint.h>
#include "foc_config.h"
#include "foc_types.h"


/* ============================================================================
 * ENCODER STATE
 * ============================================================================ */
extern volatile int32_t  encoderCount[MTR_AMT];
extern volatile int32_t  lastCount[MTR_AMT];
extern volatile int32_t  encoderSpeed[MTR_AMT];


/* ============================================================================
 * CURRENT-SENSE OFFSET CALIBRATION
 * ============================================================================ */
extern volatile float    offsetA[MTR_AMT];
extern volatile float    offsetB[MTR_AMT];
extern volatile uint32_t calibrateCounts[MTR_AMT];

extern const float       current_scalar;


/* ============================================================================
 * PER-CHANNEL GAIN CORRECTION
 * ============================================================================ */
extern float gainCorrA[MTR_AMT];
extern float gainCorrB[MTR_AMT];


/* ============================================================================
 * ADC FILTER STATE
 * ============================================================================ */
extern MedianFilter_t mfA[MTR_AMT];
extern MedianFilter_t mfB[MTR_AMT];
extern float          filtIa[MTR_AMT];
extern float          filtIb[MTR_AMT];


/* ============================================================================
 * STAGING BUFFER (ADC ISR → FOC ISR)
 * ============================================================================ */
extern volatile float   stagingIa[MTR_AMT];
extern volatile float   stagingIb[MTR_AMT];
extern volatile uint8_t newCurrentData[MTR_AMT];


/* ============================================================================
 * FOC ISR WORKING COPIES + DQ OFFSET
 * ============================================================================ */
extern float Ia[MTR_AMT];
extern float Ib[MTR_AMT];

extern float idOffset[MTR_AMT];
extern float iqOffset[MTR_AMT];


/* ============================================================================
 * PI CONTROLLERS
 * ============================================================================ */
extern PI_t piD[MTR_AMT];
extern PI_t piQ[MTR_AMT];


/* ============================================================================
 * TORQUE COMMAND
 * ============================================================================ */
extern volatile float targetTorque_Nm[MTR_AMT];
extern float          rampedTorque_Nm[MTR_AMT];
extern float          targetIq[MTR_AMT];


/* ============================================================================
 * FOC STATE MACHINE
 * ============================================================================ */
extern volatile FocState_t focState[MTR_AMT];
extern volatile uint8_t    focEnabled[MTR_AMT];
extern float               alignVd_V[MTR_AMT];


/* ============================================================================
 * ALIGNMENT CONTROL
 * ============================================================================ */
extern volatile float alignSweepAngle_deg[MTR_AMT];
extern volatile float encoderOffset_deg[MTR_AMT];


/* ============================================================================
 * FAULTS
 * ============================================================================ */
extern volatile uint32_t faultFlags;
extern volatile uint8_t  gainFaultA;
extern volatile uint8_t  gainFaultB;


/* ============================================================================
 * GAIN CALIBRATION ACCUMULATORS
 * ============================================================================ */
extern float    gainSumA[MTR_AMT];
extern float    gainSumB[MTR_AMT];
extern uint32_t gainSamples[MTR_AMT];
extern uint32_t gainSettleSkip[MTR_AMT];


/* ============================================================================
 * DEBUG LIVE-WATCH
 * ============================================================================ */
#if Debug == 1
extern volatile float   debugId[MTR_AMT];
extern volatile float   debugIq[MTR_AMT];
extern volatile float   debugElecAngle[MTR_AMT];
extern volatile int32_t debugEncoderCount[MTR_AMT];
extern volatile float   debugVd[MTR_AMT];
extern volatile float   debugVq[MTR_AMT];
extern volatile float   debugIa[MTR_AMT];
extern volatile float   debugIb[MTR_AMT];
extern volatile float   debugProbeSpread[MTR_AMT];
extern volatile float   debugLinregSlope[MTR_AMT];
extern volatile float   debugLinregResidual[MTR_AMT];
#endif


/* ============================================================================
 * DATA ACQUISITION ARRAYS (M1 only, optional)
 * ============================================================================ */
#if dataAqu == 1
extern volatile float   IdG[AQU_AMT];
extern volatile float   IqG[AQU_AMT];
extern volatile float   ElecAngleG[AQU_AMT];
extern volatile float   VdG[AQU_AMT];
extern volatile float   VqG[AQU_AMT];
extern volatile float   IaG[AQU_AMT];
extern volatile float   IbG[AQU_AMT];
extern volatile int32_t EncoderCountG[AQU_AMT];
#endif


/* ============================================================================
 * OPEN-LOOP CHARACTERISATION (only if openLoop = 1)
 * ============================================================================ */
#if openLoop == 1
extern volatile float openLoopVd[MTR_AMT];
extern volatile float openLoopVq[MTR_AMT];
#endif


/* ============================================================================
 * UART TELEMETRY GLOBALS
 * ============================================================================ */
extern volatile uint32_t uartFramesAccepted;
extern volatile uint32_t uartFramesDropped;
extern volatile uint32_t uartSyncHunts;
extern volatile uint32_t uartSeqErrors;
extern volatile uint8_t  isInit;
extern volatile float    pos_M1_Offset;
extern volatile float    pos_M2_Offset;


/* ============================================================================
 * FAULT HELPERS
 * ============================================================================ */
void emergencyStop  (uint32_t reason);
void raiseSoftFault (uint32_t flag);
void clearSoftFault (uint32_t flag);


#endif /* FOC_STATE_H */
