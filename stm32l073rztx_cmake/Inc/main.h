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
#include "stm32l0xx_hal.h"

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
#define SX_dio1_Pin GPIO_PIN_13
#define SX_dio1_GPIO_Port GPIOC
#define LORA_radiosw_Pin GPIO_PIN_1
#define LORA_radiosw_GPIO_Port GPIOA
#define GPS_rst_Pin GPIO_PIN_2
#define GPS_rst_GPIO_Port GPIOB
#define SX_dio0_Pin GPIO_PIN_11
#define SX_dio0_GPIO_Port GPIOB
#define GPS_vshifter_Pin GPIO_PIN_6
#define GPS_vshifter_GPIO_Port GPIOC
#define SX_dio5_Pin GPIO_PIN_15
#define SX_dio5_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_11
#define GPS_RX_GPIO_Port GPIOC
#define SX_dio4_Pin GPIO_PIN_3
#define SX_dio4_GPIO_Port GPIOB
#define SX_dio3_Pin GPIO_PIN_4
#define SX_dio3_GPIO_Port GPIOB
#define GPS_1PPS_Pin GPIO_PIN_5
#define GPS_1PPS_GPIO_Port GPIOB
#define SX_dio2_Pin GPIO_PIN_9
#define SX_dio2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
