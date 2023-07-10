/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "tasks.h"

#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/platforms/stm32/HAL_UARTDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/peripherals/LED/LED.h"
#include "device/platforms/stm32/HAL_I2CDevice.h"
//#include "device/peripherals/W25Q/W25Q.h"
#include "device/peripherals/ADXL375/ADXL375.h"
#include "device/peripherals/BMP3XX/BMP3XX.h"
#include "device/peripherals/LIS3MDL/LIS3MDL.h"
#include "device/peripherals/LSM6DSL/LSM6DSL.h"
#include "device/peripherals/MS5607/MS5607.h"
#include "device/peripherals/SHTC3/SHTC3.h"
#include "device/peripherals/TMP117/TMP117.h"
#include "SensorModuleDeviceMap.h"

//#include "sched/macros/call.h"
/* USER CODE END Includes */
 
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	LED** led;
	bool on;
	uint32_t on_time;
	uint32_t period; // Do not make this smaller than on_time, there is no check for this
} led_flash_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
SensorModuleDeviceMap deviceMap = SensorModuleDeviceMap(nullptr);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_SPI1_Init(void);

static void MX_SPI2_Init(void);

static void MX_I2C3_Init(void);

static void MX_UART5_Init(void);

/* USER CODE BEGIN PFP */
static MS5607 *ms5607 = nullptr;
static BMP3XX *bmp3XX = nullptr;
static ADXL375 *adxl375 = nullptr;
static LIS3MDL *lis3mdl = nullptr;
static LSM6DSL *lsm6dsl = nullptr;
static TMP117 *tmp117 = nullptr;
static SHTC3 *shtc3 = nullptr;
static LED *ledOne = nullptr;
static LED *ledTwo = nullptr;
static LED *wizLED = nullptr;
static HALUARTDevice *uartDev = nullptr;
static HALI2CDevice *i2cDev = nullptr;
static HALSPIDevice *wizSPI = nullptr;
static HALGPIODevice *wizCS = nullptr;
static HALSPIDevice *flashSPI = nullptr;

//static W5500 *w5500 = nullptr;
//static IPv4UDPStack *stack = nullptr;
//static IPv4UDPSocket *sock = nullptr;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Don't do the variable arguments stuff to save time
// I don't actually know how much time this saves
void swprint(const char* msg) {
#ifdef DEBUG
	int len = strlen(msg);
	for (int i = 0; i < len; i++) {
		ITM_SendChar(msg[i]);
	}
#endif
}

// send stuff down serial wire out if DEBUG flag set
// Do not send a message larger than 256 bytes
int swprintf(const char* fmt, ...) {
#ifdef DEBUG
	va_list ap;
	va_start(ap, fmt);
	char msg[256];
	int status = vsnprintf(msg, 256, fmt, ap);
	va_end(ap);

	if (status > 0) {
		for (int i = 0; i < status; i++) {
			ITM_SendChar(msg[i]);
		}
	}

	return status;
#else
	return 0;
#endif
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_I2C3_Init();
    MX_UART5_Init();
    /* USER CODE BEGIN 2 */
    HALUARTDevice uart("UART", &huart5);
    RetType ret = uart.init();

    if (ret != RET_SUCCESS) {
    	swprint("Failed to init UART\n");
        return -1;
    }
    uartDev = &uart;
    swprint("UART Initalized\n");

    if (!sched_init(&HAL_GetTick)) {
    	swprint("Failed to init scheduler\n");
        return -1;
    }

    // Initialize peripherals
    HALGPIODevice ledOneGPIO("LED 1 GPIO", PA1_LED_GPIO_Port, PA1_LED_Pin);
    ret = ledOneGPIO.init();
    LED ledOneLocal(ledOneGPIO);
    ledOneLocal.set_state(LED_OFF);
    ledOne = &ledOneLocal;

    HALGPIODevice ledTwoGPIO("LED 2 GPIO", PA2_LED_GPIO_Port, PA2_LED_Pin);
    ret = ledTwoGPIO.init();
    LED ledTwoLocal(ledTwoGPIO);
    ledOneLocal.set_state(LED_OFF);
    ledTwo = &ledTwoLocal;

    HALGPIODevice wiznetLEDGPIO("Wiznet LED GPIO", Wiz_LED_GPIO_Port, Wiz_LED_Pin);
    ret = wiznetLEDGPIO.init();
    LED wiznetLED(wiznetLEDGPIO);
    wiznetLED.set_state(LED_ON);
    wizLED = &wiznetLED;

    HALGPIODevice wizChipSelect("Wiznet CS", ETH_CS_GPIO_Port, ETH_CS_Pin);
    ret = wizChipSelect.init();
    wizCS = &wizChipSelect;
    wizChipSelect.set(1);

    static HALI2CDevice i2c("HAL I2C3", &hi2c3);
    ret = i2c.init();
    if (RET_SUCCESS != ret) {
    	swprint("Failed to init I2C3\n");
        return -1;
    }
	i2cDev = &i2c;

    static HALSPIDevice wizSpi("WIZNET SPI", &hspi1);
    ret = wizSpi.init();
    if (RET_SUCCESS != ret) {
    	swprint("Failed to init SPI\n");
    	return -1;
    }
    wizSPI = &wizSpi;

//    sched_start(netStackInitTask, {});

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1) {
        sched_dispatch();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void) {

    /* USER CODE BEGIN I2C3_Init 0 */

    /* USER CODE END I2C3_Init 0 */

    /* USER CODE BEGIN I2C3_Init 1 */

    /* USER CODE END I2C3_Init 1 */
    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C3_Init 2 */

    /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void) {

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void) {

    /* USER CODE BEGIN UART5_Init 0 */

    /* USER CODE END UART5_Init 0 */

    /* USER CODE BEGIN UART5_Init 1 */

    /* USER CODE END UART5_Init 1 */
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 115200;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART5_Init 2 */

    /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, WS25_CS_Pin | ETH_CS_Pin | Wiz_LED_Pin | RS485_MODE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, PA1_LED_Pin | PA2_LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : ADDR3raw_Pin ADDR2raw_Pin ADDR1raw_Pin P_IO_1_Pin */
    GPIO_InitStruct.Pin = ADDR3raw_Pin | ADDR2raw_Pin | ADDR1raw_Pin | P_IO_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : ADDR0raw_Pin */
    GPIO_InitStruct.Pin = ADDR0raw_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADDR0raw_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : WS25_CS_Pin ETH_CS_Pin Wiz_LED_Pin RS485_MODE_Pin */
    GPIO_InitStruct.Pin = WS25_CS_Pin | ETH_CS_Pin | Wiz_LED_Pin | RS485_MODE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : W5500_RST_Pin */
    GPIO_InitStruct.Pin = W5500_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(W5500_RST_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : W5500_INT_Pin LIS_INT_Pin LIS_DRDY_Pin */
    GPIO_InitStruct.Pin = W5500_INT_Pin | LIS_INT_Pin | LIS_DRDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PA1_LED_Pin PA2_LED_Pin */
    GPIO_InitStruct.Pin = PA1_LED_Pin | PA2_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : LSM6_INT1_Pin */
    GPIO_InitStruct.Pin = LSM6_INT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LSM6_INT1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LSM6_INT2_DEN_Pin ADXL_INT2_Pin ADXL_INT1_Pin */
    GPIO_InitStruct.Pin = LSM6_INT2_DEN_Pin | ADXL_INT2_Pin | ADXL_INT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : MCU_INT_Pin BMP_INT_Pin */
    GPIO_InitStruct.Pin = MCU_INT_Pin | BMP_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : P_IO_2_Pin */
    GPIO_InitStruct.Pin = P_IO_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(P_IO_2_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
