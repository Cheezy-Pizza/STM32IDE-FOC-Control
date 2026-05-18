/* ============================================================================
 * foc_init.c — Hardware bring-up sequencer.
 *
 * Both timers start at CNT=0, in phase. Earlier code attempted a 180°
 * phase-shift via TIM20.CR1.DIR, but DIR is read-only in center-aligned
 * mode (STM32G4 RM0440 §30.4.1) — the write was a silent no-op. With
 * priority-2 ADC ISRs serializing within the 100 µs PWM window, in-phase
 * operation is fine.
 * ============================================================================ */
#include "foc_init.h"
#include "foc_state.h"
#include "foc_isr.h"
#include "foc_config.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "main.h"   /* for ENABLE_x_GPIO_Port / ENABLE_x_Pin defines */


void startHardwareSync(void)
{
    /* 1. Lock ISRs in NEUTRAL so they don't apply torque prematurely. */
    for (int m = 0; m < MTR_AMT; m++) {
        focState[m]   = FOC_STATE_NEUTRAL;
        focEnabled[m] = 0;
    }

    /* 2. ADC hardware calibration. */
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

    /* 3. Start encoders. Snapshot the live counter so the FOC ISR's first
     * delta read is correct. */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    lastCount[MOTOR_1] = htim3.Instance->CNT;
    lastCount[MOTOR_2] = htim4.Instance->CNT;

    /* 4. Enable gate drivers (DRV8353), wait for charge pumps. */
    HAL_GPIO_WritePin(ENABLE_3_GPIO_Port, ENABLE_3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENABLE_1_GPIO_Port, ENABLE_1_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    applyNeutralOutput(MOTOR_1);
    applyNeutralOutput(MOTOR_2);

    /* 5. Start PWM generation. */
    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_1); HAL_TIMEx_PWMN_Start(&htim8,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_2); HAL_TIMEx_PWMN_Start(&htim8,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_3); HAL_TIMEx_PWMN_Start(&htim8,  TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1); HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2); HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_3); HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_3);

    /* 6. Start ADCs in injected mode. */
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);

    /* 7. Synchronized FOC timer kickoff: both at CNT=0, then enable. */
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
