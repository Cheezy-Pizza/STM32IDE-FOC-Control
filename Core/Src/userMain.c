/* ============================================================================
 * main.c — STM32G474VET6 FOC Motor Control (rev 5, modular)
 *
 * Top-level orchestration only. All real-time / domain logic lives in
 * dedicated modules:
 *
 *   foc_config.h       Build-time constants
 *   foc_types.h        Type definitions
 *   foc_state.{h,c}    Shared global state + fault helpers
 *   foc_math.{h,c}     CORDIC, Clarke/Park, SVPWM, PI, angle helpers
 *   foc_isr.{h,c}      FOC routine, ADC ISR, HAL callbacks
 *   foc_calib.{h,c}    Linreg alignment, gain cal, DQ offset cal
 *   foc_init.{h,c}     Hardware bring-up sequencer
 *   uart_proto.{h,c}   UART host protocol
 *
 * BOOT SEQUENCE
 *   1. HAL_Init + SystemClock_Config + MX peripheral inits
 *   2. uart_init        — DMA receive armed
 *   3. startHardwareSync — encoders, gate drivers, PWM, ADCs, FOC timers
 *                          all started; both motors locked in NEUTRAL.
 *   4. calibrateMotor(M1) — sweep cal + gain cal + DQ offset cal.
 *      M1 stays in NEUTRAL on exit (no bouncing during step 5).
 *   5. calibrateMotor(M2) if ENABLE_MOTOR_2.
 *   6. Arm both motors together if no hard faults / alignment failures.
 *   7. Start TIM7 (1 kHz) for UART poll + watchdog + velocity update.
 *   8. Application loop: bench torque test if BENCH_TEST_LOOP, else idle.
 *
 * REQUIRED CUBEMX SETTING
 *   TIM20 OC4 Pulse = 10 (must match TIM8.PulseWidth = 10).
 *
 * SAFETY ARCHITECTURE
 *   1. DRV8353R hardware: VDS shoot-through, gate UVLO, OTSD.
 *   2. Firmware ADC layer: per-cycle current ceiling → emergencyStop.
 *   3. Firmware command layer: torque ramp + watchdog timeout → soft fault.
 * ============================================================================ */

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "crc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include "foc_config.h"
#include "foc_state.h"
#include "foc_init.h"
#include "foc_calib.h"
#include "uart_proto.h"


/* Forward declaration of the system clock helper. */
static void SystemClock_Config(void);


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

    /* 1. Bring up hardware. Both motors stay in NEUTRAL. */
    startHardwareSync();

    /* 2. Per-motor calibration. M1 stays parked while M2 calibrates. */
    calibrateMotor(MOTOR_1);

#if ENABLE_MOTOR_2 == 1
    calibrateMotor(MOTOR_2);
#endif

    /* 3. Arm both motors together — only if no blocking faults. */
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

    /* 4. Start TIM7 (1 kHz) — UART poll + watchdog + velocity update. */
    HAL_TIM_Base_Start_IT(&htim7);


    /* ========================================================================
     * APPLICATION LOOP
     * ======================================================================== */
    while (1)
    {
#if BENCH_TEST_LOOP == 1
        targetTorque_Nm[MOTOR_1] = 0.05f;
  #if ENABLE_MOTOR_2 == 1
        targetTorque_Nm[MOTOR_2] = 0.05f;
  #endif
        HAL_Delay(2000);

        targetTorque_Nm[MOTOR_1] = -0.05f;
  #if ENABLE_MOTOR_2 == 1
        targetTorque_Nm[MOTOR_2] = -0.05f;
  #endif
        HAL_Delay(2000);
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
