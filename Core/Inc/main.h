/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_ll_pwr.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Vkeyboard_L_Pin GPIO_PIN_0
#define Vkeyboard_L_GPIO_Port GPIOA
#define Vkeyboard_H_Pin GPIO_PIN_1
#define Vkeyboard_H_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define Audio_out_R_Pin GPIO_PIN_4
#define Audio_out_R_GPIO_Port GPIOA
#define Audio_out_L_Pin GPIO_PIN_5
#define Audio_out_L_GPIO_Port GPIOA
#define SPK_hi_B_Pin GPIO_PIN_7
#define SPK_hi_B_GPIO_Port GPIOA
#define SPK_hi_A_Pin GPIO_PIN_0
#define SPK_hi_A_GPIO_Port GPIOB
#define SPK_lo_B_Pin GPIO_PIN_8
#define SPK_lo_B_GPIO_Port GPIOA
#define SPK_lo_A_Pin GPIO_PIN_9
#define SPK_lo_A_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Instp_Pin GPIO_PIN_3
#define Instp_GPIO_Port GPIOB
#define Volp_Pin GPIO_PIN_4
#define Volp_GPIO_Port GPIOB
#define Instm_Pin GPIO_PIN_5
#define Instm_GPIO_Port GPIOB
#define Volm_Pin GPIO_PIN_6
#define Volm_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
