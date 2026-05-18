/* ============================================================================
 * uart_proto.c — UART host protocol implementation.
 *
 * 10-byte command frames from host:  [SYNC][seq][tq_M1 f32][tq_M2 f32]
 * 18-byte response frames to host:   [SYNC][seq][pos_M1 f32][pos_M2 f32]
 *                                    [vel_M1 f32][vel_M2 f32]
 *
 * Special torque-encoded commands (transmitted as NaN-payload f32):
 *   UART_INIT_NAN          0x7FC00001 — arm motors waiting in WAIT_UART
 *   UART_ZERO_ENCODERS_NAN 0x7FC00013 — capture current encoder pos as 0
 *
 * Reception is via UART1 DMA into a circular buffer; host-direction is
 * polled at 1 kHz from TIM7. Transmission is interrupt-driven.
 * ============================================================================ */
#include <string.h>
#include "uart_proto.h"
#include "foc_state.h"
#include "foc_config.h"
#include "usart.h"
#include "tim.h"
#include "stm32g4xx_hal.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;


static uint8_t          uartRxBuf[UART_RX_BUF_SIZE];
static uint16_t         uartRxTail   = 0;
static uint8_t          uartTxBuf[UART_RSP_FRAME_LEN];
static volatile uint8_t uartTxBusy   = 0;

static uint8_t          uartLastSeq     = 0;
static uint8_t          uartFirstFrame  = 1;
static uint32_t         uartLastValidMs = 0;


/* ============================================================================
 * HAL TX-COMPLETE CALLBACK
 * ============================================================================ */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        uartTxBusy = 0;
}


/* ============================================================================
 * LOCAL UTILITIES
 * ============================================================================ */
static inline float load_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}

static inline void store_f32(uint8_t *p, float v)
{
    memcpy(p, &v, 4);
}


/* ============================================================================
 * SEND RESPONSE
 * ============================================================================ */
static void sendResponse(uint8_t seq)
{
    if (uartTxBusy) {
        uartFramesDropped++;
        return;
    }

    float pos_M1 = (float)encoderCount[MOTOR_1] / (float)COUNTS_PER_REV - pos_M1_Offset;
    float vel_M1 = (float)encoderSpeed[MOTOR_1] / (float)COUNTS_PER_REV;
    float pos_M2 = (float)encoderCount[MOTOR_2] / (float)COUNTS_PER_REV - pos_M2_Offset;
    float vel_M2 = (float)encoderSpeed[MOTOR_2] / (float)COUNTS_PER_REV;

    uartTxBuf[0] = UART_SYNC_TX;
    uartTxBuf[1] = seq;

    if (isInit == 1) {
        uint32_t initSend = UART_INIT_NAN_SEND;
        isInit = 0;
        memcpy(&uartTxBuf[2], &initSend, 4);
    } else {
        store_f32(&uartTxBuf[2], pos_M1);
    }
    store_f32(&uartTxBuf[6],  pos_M2);
    store_f32(&uartTxBuf[10], vel_M1);
    store_f32(&uartTxBuf[14], vel_M2);

    uartTxBusy = 1;
    if (HAL_UART_Transmit_IT(&huart1, uartTxBuf, UART_RSP_FRAME_LEN) != HAL_OK) {
        uartTxBusy = 0;
        uartFramesDropped++;
    }
}


/* ============================================================================
 * APPLY ONE FRAME
 * ============================================================================ */
static void applyFrame(const uint8_t *frame)
{
    const uint8_t seq    = frame[1];
    float         tq_M1  = load_f32(&frame[2]);
    float         tq_M2  = load_f32(&frame[6]);

    if (!uartFirstFrame) {
        if (seq != (uint8_t)(uartLastSeq + 1u))
            uartSeqErrors++;
    }
    uartFirstFrame = 0;
    uartLastSeq    = seq;

    const float TQ_LIMIT = (float)MAX_CURRENT_MA * 0.001f * KT_NM_PER_AMP;

    /* Special command: init NaN — arm motors waiting in WAIT_UART. */
    uint32_t initCheck;
    memcpy(&initCheck, &frame[2], 4);
    if (initCheck == UART_INIT_NAN) {
        uint8_t armed = 0;
        for (int m = 0; m < MTR_AMT; m++) {
            if (focState[m] == FOC_STATE_WAIT_UART) {
                focState[m] = FOC_STATE_UART_ENABLED;
                armed = 1;
            }
        }
        if (armed) isInit = 1;
    }
    if (initCheck == UART_ZERO_ENCODERS_NAN) {
        pos_M1_Offset = (float)encoderCount[MOTOR_1] / (float)COUNTS_PER_REV;
        pos_M2_Offset = (float)encoderCount[MOTOR_2] / (float)COUNTS_PER_REV;
    }

    /* Per-motor NaN/clamp handling. */
    uint8_t cmdClamped = 0;
    if (tq_M1 != tq_M1) tq_M1 = 0.0f;
    if      (tq_M1 >  TQ_LIMIT) { tq_M1 =  TQ_LIMIT; cmdClamped = 1; }
    else if (tq_M1 < -TQ_LIMIT) { tq_M1 = -TQ_LIMIT; cmdClamped = 1; }

    if (tq_M2 != tq_M2) tq_M2 = 0.0f;
    if      (tq_M2 >  TQ_LIMIT) { tq_M2 =  TQ_LIMIT; cmdClamped = 1; }
    else if (tq_M2 < -TQ_LIMIT) { tq_M2 = -TQ_LIMIT; cmdClamped = 1; }

    if (cmdClamped) raiseSoftFault(FAULT_TORQUE_CMD_CLAMPED);
    else            clearSoftFault(FAULT_TORQUE_CMD_CLAMPED);

    if (faultFlags & (FAULT_HARD_MASK | FAULT_ALIGN_NOT_MOVED)) {
        tq_M1 = 0.0f;
        tq_M2 = 0.0f;
    }

    /* Per-motor gating: torque only applied to a motor that's UART-ENABLED. */
    targetTorque_Nm[MOTOR_1] = (focState[MOTOR_1] == FOC_STATE_UART_ENABLED) ? tq_M1 : 0.0f;
    targetTorque_Nm[MOTOR_2] = (focState[MOTOR_2] == FOC_STATE_UART_ENABLED) ? tq_M2 : 0.0f;

    uartLastValidMs = HAL_GetTick();
    uartFramesAccepted++;
    clearSoftFault(FAULT_HOST_TIMEOUT);

    sendResponse(seq);
}


/* ============================================================================
 * POLL RX BUFFER (DMA circular)
 * ============================================================================ */
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

        if (uartRxBuf[uartRxTail] != UART_SYNC_RX) {
            uartSyncHunts++;
            uartRxTail = (uint16_t)((uartRxTail + 1u) & (UART_RX_BUF_SIZE - 1u));
            continue;
        }

        uint8_t frame[UART_CMD_FRAME_LEN];
        for (uint16_t i = 0; i < UART_CMD_FRAME_LEN; i++)
            frame[i] = uartRxBuf[(uartRxTail + i) & (UART_RX_BUF_SIZE - 1u)];

        uartRxTail = (uint16_t)((uartRxTail + UART_CMD_FRAME_LEN)
                                & (UART_RX_BUF_SIZE - 1u));

        applyFrame(frame);

        dmaHead = (uint16_t)(UART_RX_BUF_SIZE
                             - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    }
}


/* ============================================================================
 * HOST TIMEOUT WATCHDOG
 * ============================================================================ */
void uart_check_timeout(void)
{
    if (uartFirstFrame) return;

    if ((HAL_GetTick() - uartLastValidMs) > UART_CMD_TIMEOUT_MS)
    {
        for (int motor = 0; motor < MTR_AMT; motor++)
            targetTorque_Nm[motor] = 0.0f;
        raiseSoftFault(FAULT_HOST_TIMEOUT);
    }
}


/* ============================================================================
 * VELOCITY UPDATE (10 ms moving window)
 * ============================================================================ */
void uart_update_velocity(void)
{
    static uint32_t lastTick = 0;
    static int32_t  lastCnt[MTR_AMT] = {0, 0};

    uint32_t now = HAL_GetTick();
    if ((now - lastTick) >= 10u)
    {
        for (int motor = 0; motor < MTR_AMT; motor++)
        {
            const int32_t delta = encoderCount[motor] - lastCnt[motor];
            encoderSpeed[motor] = delta * 100;
            lastCnt[motor]      = encoderCount[motor];
        }
        lastTick = now;
    }
}


/* ============================================================================
 * INIT
 * ============================================================================ */
void uart_init(void)
{
    HAL_UART_Receive_DMA(&huart1, uartRxBuf, UART_RX_BUF_SIZE);
}
