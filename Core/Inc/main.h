/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define ADDR3raw_Pin GPIO_PIN_13
#define ADDR3raw_GPIO_Port GPIOC
#define ADDR2raw_Pin GPIO_PIN_14
#define ADDR2raw_GPIO_Port GPIOC
#define ADDR1raw_Pin GPIO_PIN_15
#define ADDR1raw_GPIO_Port GPIOC
#define ADDR0raw_Pin GPIO_PIN_0
#define ADDR0raw_GPIO_Port GPIOH
#define WS25_CS_Pin GPIO_PIN_0
#define WS25_CS_GPIO_Port GPIOC
#define W5500_RST_Pin GPIO_PIN_3
#define W5500_RST_GPIO_Port GPIOA
#define W5500_INT_Pin GPIO_PIN_4
#define W5500_INT_GPIO_Port GPIOA
#define ETH_CS_Pin GPIO_PIN_4
#define ETH_CS_GPIO_Port GPIOC
#define Wiz_LED_Pin GPIO_PIN_5
#define Wiz_LED_GPIO_Port GPIOC
#define PA1_LED_Pin GPIO_PIN_0
#define PA1_LED_GPIO_Port GPIOB
#define PA2_LED_Pin GPIO_PIN_1
#define PA2_LED_GPIO_Port GPIOB
#define LSM6_INT1_Pin GPIO_PIN_10
#define LSM6_INT1_GPIO_Port GPIOB
#define LSM6_INT2_DEN_Pin GPIO_PIN_12
#define LSM6_INT2_DEN_GPIO_Port GPIOB
#define ADXL_INT2_Pin GPIO_PIN_14
#define ADXL_INT2_GPIO_Port GPIOB
#define ADXL_INT1_Pin GPIO_PIN_15
#define ADXL_INT1_GPIO_Port GPIOB
#define MCU_INT_Pin GPIO_PIN_6
#define MCU_INT_GPIO_Port GPIOC
#define BMP_INT_Pin GPIO_PIN_7
#define BMP_INT_GPIO_Port GPIOC
#define LIS_INT_Pin GPIO_PIN_9
#define LIS_INT_GPIO_Port GPIOA
#define LIS_DRDY_Pin GPIO_PIN_10
#define LIS_DRDY_GPIO_Port GPIOA
#define P_IO_2_Pin GPIO_PIN_15
#define P_IO_2_GPIO_Port GPIOA
#define P_IO_1_Pin GPIO_PIN_10
#define P_IO_1_GPIO_Port GPIOC
#define RS485_MODE_Pin GPIO_PIN_11
#define RS485_MODE_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
