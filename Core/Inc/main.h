/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_H_A_1_Pin GPIO_PIN_2
#define IN_H_A_1_GPIO_Port GPIOE
#define IN_H_B_1_Pin GPIO_PIN_3
#define IN_H_B_1_GPIO_Port GPIOE
#define IN_L_A_1_Pin GPIO_PIN_4
#define IN_L_A_1_GPIO_Port GPIOE
#define IN_L_B_1_Pin GPIO_PIN_5
#define IN_L_B_1_GPIO_Port GPIOE
#define IN_L_C_1_Pin GPIO_PIN_6
#define IN_L_C_1_GPIO_Port GPIOE
#define N_FAULT_1_Pin GPIO_PIN_9
#define N_FAULT_1_GPIO_Port GPIOF
#define IN_L_C_2_Pin GPIO_PIN_0
#define IN_L_C_2_GPIO_Port GPIOF
#define IN_H_A_2_Pin GPIO_PIN_0
#define IN_H_A_2_GPIO_Port GPIOC
#define IN_H_B_2_Pin GPIO_PIN_1
#define IN_H_B_2_GPIO_Port GPIOC
#define IN_H_C_2_Pin GPIO_PIN_2
#define IN_H_C_2_GPIO_Port GPIOC
#define IN_H_C_1_Pin GPIO_PIN_2
#define IN_H_C_1_GPIO_Port GPIOF
#define SOA_2_Pin GPIO_PIN_0
#define SOA_2_GPIO_Port GPIOA
#define SOB_2_Pin GPIO_PIN_1
#define SOB_2_GPIO_Port GPIOA
#define ENC_B_3_Pin GPIO_PIN_4
#define ENC_B_3_GPIO_Port GPIOA
#define ENC_A_2_Pin GPIO_PIN_5
#define ENC_A_2_GPIO_Port GPIOA
#define SOA_3_Pin GPIO_PIN_6
#define SOA_3_GPIO_Port GPIOA
#define SOB_3_Pin GPIO_PIN_7
#define SOB_3_GPIO_Port GPIOA
#define IN_L_B_3_Pin GPIO_PIN_0
#define IN_L_B_3_GPIO_Port GPIOB
#define SOA_1_Pin GPIO_PIN_1
#define SOA_1_GPIO_Port GPIOB
#define ENABLE_2_Pin GPIO_PIN_7
#define ENABLE_2_GPIO_Port GPIOE
#define ENABLE_3_Pin GPIO_PIN_8
#define ENABLE_3_GPIO_Port GPIOE
#define IN_L_B_2_Pin GPIO_PIN_10
#define IN_L_B_2_GPIO_Port GPIOE
#define ENABLE_1_Pin GPIO_PIN_11
#define ENABLE_1_GPIO_Port GPIOE
#define N_FAULT_2_Pin GPIO_PIN_15
#define N_FAULT_2_GPIO_Port GPIOE
#define IN_L_A_2_Pin GPIO_PIN_13
#define IN_L_A_2_GPIO_Port GPIOB
#define ENC_A_1_Pin GPIO_PIN_12
#define ENC_A_1_GPIO_Port GPIOD
#define ENC_B_1_Pin GPIO_PIN_13
#define ENC_B_1_GPIO_Port GPIOD
#define ENC_A_3_Pin GPIO_PIN_6
#define ENC_A_3_GPIO_Port GPIOC
#define IN_H_B_3_Pin GPIO_PIN_7
#define IN_H_B_3_GPIO_Port GPIOC
#define IN_H_C_3_Pin GPIO_PIN_8
#define IN_H_C_3_GPIO_Port GPIOC
#define N_FAULT_3_Pin GPIO_PIN_10
#define N_FAULT_3_GPIO_Port GPIOA
#define IN_H_A_3_Pin GPIO_PIN_15
#define IN_H_A_3_GPIO_Port GPIOA
#define IN_L_A_3_Pin GPIO_PIN_10
#define IN_L_A_3_GPIO_Port GPIOC
#define IN_L_C_3_Pin GPIO_PIN_12
#define IN_L_C_3_GPIO_Port GPIOC
#define ENC_B_2_Pin GPIO_PIN_4
#define ENC_B_2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
