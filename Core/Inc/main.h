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
#include "stm32f1xx_hal.h"

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

/* ----------------------------------------------------------------------------
 * POWER_SAVE_LOW_CLOCK
 *   0 = Default. SYSCLK = 72 MHz (HSE 8 MHz x PLL9), PCLK1 = 36 MHz.
 *   1 = SYSCLK = HCLK = PCLK1 = PCLK2 = 8 MHz (HSE bypass, PLL OFF,
 *       FLASH_LATENCY_0). Saves ~15-20 mA on the MCU core.
 *
 * Both `SystemClock_Config()` (main.cpp) and `MX_CAN_Init()` (can.c) read
 * this macro and pick consistent settings. The CAN bit timing is recomputed
 * to keep 250 kbps with the same 81.25 % sample point at either PCLK1.
 *
 * Test procedure: set to 1, flash, verify the node appears on the bus and
 * the temperature reading is stable. Revert to 0 if N2K traffic is lost.
 * --------------------------------------------------------------------------*/
#ifndef POWER_SAVE_LOW_CLOCK
#define POWER_SAVE_LOW_CLOCK 0
#endif

#if POWER_SAVE_LOW_CLOCK
  /* PCLK1 = 8 MHz: prescaler 2 -> 4 MHz TQ clock, 16 TQ/bit -> 250 kbps. */
  #define N2K_CAN_PRESCALER 2u
#else
  /* PCLK1 = 36 MHz: prescaler 9 -> 4 MHz TQ clock, 16 TQ/bit -> 250 kbps. */
  #define N2K_CAN_PRESCALER 9u
#endif

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_SPI_Pin GPIO_PIN_3
#define EN_SPI_GPIO_Port GPIOA
#define CS_ADC_Pin GPIO_PIN_0
#define CS_ADC_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
