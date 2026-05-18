/* ============================================================================
 * foc_isr.h — ISR-side functions for FOC controller.
 *
 * The HAL callbacks (HAL_TIM_PeriodElapsedCallback, HAL_ADCEx_InjectedConvCpltCallback)
 * are defined in foc_isr.c and dispatch into FOCroutine() / ADCloop().
 * They are also called from uart_proto.c via HAL_TIM_PeriodElapsedCallback,
 * but uart_proto handles its own TIM7 case there.
 * ============================================================================ */
#ifndef FOC_ISR_H
#define FOC_ISR_H

#include <stdint.h>


/* Per-motor FOC pipeline. Called at FOC_FS_HZ from TIM8 (M1) / TIM20 (M2). */
void FOCroutine(uint8_t motor);


/* Per-motor ADC injected complete handler. Called from ADC2/ADC3 ISRs. */
void ADCloop(uint8_t motor);


/* Park PWM at 50 % duty on all phases for one motor. */
void applyNeutralOutput(uint8_t motor);


#endif /* FOC_ISR_H */
