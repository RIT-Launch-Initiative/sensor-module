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
#include "device/platforms/stm32/swdebug.h"

#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/peripherals/LED/LED.h"
#include "device/peripherals/W25Q/W25Q.h"

#include "sched/macros/call.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	LED** led;
	bool on;
	uint32_t on_time;
	uint32_t period;
} led_flash_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef DEBUG
#define DEBUG
#endif
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_SPI1_Init(void);

static void MX_SPI2_Init(void);

static void MX_I2C3_Init(void);

static void MX_UART5_Init(void);

/* USER CODE BEGIN PFP */
// devices
static HALGPIODevice *flash_cs = nullptr;
static HALSPIDevice *flash_spi = nullptr;

// peripherals
static LED *ledOne = nullptr;
static LED *ledTwo = nullptr;
static LED *wizLED = nullptr;
static W25Q* w25q = nullptr;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


RetType print_heartbeat_task(void*) {
	RESUME();
	static int i = 0;
	swprintf("#GRN#Heartbeat %d\n", i++);
	SLEEP(1000);
	RESET();
	return RET_SUCCESS;
}


RetType flash_led_task(void* task_led) {
    LED* led = *((LED **) task_led);

	RESUME();

    CALL(led->flash());

	RESET();
	return RET_SUCCESS;
}

RetType init_led_task(void*) {
	RESUME();

	CALL(ledOne->init());
	CALL(ledTwo->init());
	CALL(wizLED->init());

    CALL(ledOne->set_state(LED_OFF));
    CALL(ledTwo->set_state(LED_OFF));
    CALL(wizLED->set_state(LED_OFF));

	RESET();
	return RET_ERROR; // kill when done
}

RetType w25q_poll_task(void *) {
    RESUME();
    CALL(w25q->poll());
    RESET();
    return RET_SUCCESS;
}

RetType flash_spi_poll_task(void *) {
    RESUME();
    CALL(flash_spi->poll());
    RESET();
    return RET_SUCCESS;
}

RetType w25q_test_task(void*) {
	RESUME();
	RetType ret;

    ledOne->set_flash(10, 50);
    sched_start(&flash_led_task, &ledOne);

	swprint("Initializing W25Q\n");
	static W25Q w25q_local("Flash memory", *flash_spi, *flash_cs);
	ret = CALL(w25q_local.init());
	if (RET_SUCCESS != ret) {
		swprint("#RED#Failed to initialize W25Q\n");
		goto w25q_test_end;
	}

	w25q = &w25q_local;
	swprintf("W25Q 0x%6x with %d blocks of %d bytes each\n",
			w25q->m_dev_id, w25q->getNumBlocks(), w25q->getBlockSize());
    sched_start(&w25q_poll_task, {});

	// set up input
	uint8_t page_in[256];
	memset(page_in, '\0', sizeof(page_in));
	const char text[] = "Testing text for page write";
	strncpy((char*) page_in, text, sizeof(text));
	swprintf("Page in:\n\t%256s\n", (char*) page_in);
	// write to this block
	uint32_t address = 0xFFFF;

/*
	swprintf("Writing to page 0x%4x\n", address);
	ret = CALL(w25q->write(address, page_in));
	if (RET_SUCCESS != ret) {
		swprint("#RED#Failed to write page\n");
		goto w25q_test_end;
	}

	// set up output
	uint8_t page_out[256];
	memset(page_out, '\0', sizeof(page_out));
	// read from page;
	swprintf("Reading from page 0x%4x\n", address);
	ret = CALL(w25q->read(address, page_out));
	if (RET_SUCCESS != ret) {
		swprint("#RED#Failed to read page\n");
		goto w25q_test_end;
	}
	swprintf("Page out:\n\t%256s\n", (char*) page_out);
*/
	w25q_test_end:
	swprint("Exiting flash test task\n");
    ledOne->set_flash(20, 1000);
	RESET();
	return RET_ERROR;
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
    RetType ret;
    // Initialize scheduler
    if (!sched_init(&HAL_GetTick)) {
    	swprint("Failed to init scheduler\n");
        return -1;
    }

    // Initialize peripherals
    static HALGPIODevice ledOneGPIO("LED 1 GPIO", PA1_LED_GPIO_Port, PA1_LED_Pin);
    static LED ledOneLocal(ledOneGPIO);
    ledOne = &ledOneLocal;

    static HALGPIODevice ledTwoGPIO("LED 2 GPIO", PA2_LED_GPIO_Port, PA2_LED_Pin);
    static LED ledTwoLocal(ledTwoGPIO);
    ledTwo = &ledTwoLocal;

    static HALGPIODevice wiznetLEDGPIO("Wiznet LED GPIO", Wiz_LED_GPIO_Port, Wiz_LED_Pin);
    static LED wiznetLED(wiznetLEDGPIO);
    wizLED = &wiznetLED;

    static HALGPIODevice flash_cs_local("Flash CS", WS25_CS_GPIO_Port, WS25_CS_Pin);
    flash_cs = &flash_cs_local;

    static HALSPIDevice flash_spi_local("Flash SPI", &hspi2);
    ret = flash_spi_local.init();
    if (RET_SUCCESS != ret) {
    	swprint("Failed to init flash SPI device");
    	return -1;
    }
    flash_spi = &flash_spi_local;


    swprint("Starting tasks: reporting inside SPI functions\n");
    // start initialization tasks

    sched_start(&init_led_task, {});
    sched_start(&flash_spi_poll_task, {});
    sched_start(&print_heartbeat_task, {});
    sched_start(&w25q_test_task, {});

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
