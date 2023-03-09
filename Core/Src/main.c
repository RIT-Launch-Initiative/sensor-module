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

#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/platforms/stm32/HAL_UARTDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/platforms/stm32/HAL_I2CDevice.h"



#include "device/peripherals/LED/LED.h"
#include "device/peripherals/W25Q/W25Q.h"
#include "device/peripherals/BMP3XX/BMP3XX.h"



//#include "filesystem/ChainFS/ChainFS.h" // TODO: Unfinished
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static BMP3XX *bmp390 = nullptr;
static LED *led = nullptr;
HALUARTDevice *uartDev = nullptr;
static HALI2CDevice *i2cDev = nullptr;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RetType i2cDevPollTask(void*) {
    RESUME();
    CALL(i2cDev->poll());
    RESET();

    return RET_SUCCESS;
}

RetType ledTask(void*) {
    RESUME();

    RetType ret = CALL(led->toggle());

    RESET();
    return RET_SUCCESS;
}

RetType bmpTask(void*) {
    RESUME();

    static double pressure = 69;
    static double temperature = 420;
    static bmp3_settings settings;
    static char buffer[100];


    RetType ret = CALL(bmp390->getSensorData(&pressure, &temperature));
    if (ret == RET_ERROR) {
        CALL(uartDev->write((uint8_t *) "Failed to get BMP data\r\n", 24));
    }

    size_t size = sprintf(buffer, "BMP Pressure: %f \r\nBMP Temperature: %f\r\n", pressure, temperature);
    CALL(uartDev->write((uint8_t *)buffer, size));

    RESET();
    return RET_SUCCESS;
}

// TODO: Figure out the initialization task
RetType sensorInitTask(void*) {
    RESUME();


    // TODO: LED is not a sensor but here for testing purposes
    CALL(uartDev->write((uint8_t *) "LED Initializing\r\n", 18));
    RetType ret = CALL(led->init());
    tid_t ledTID = -1;
    if (ret != RET_ERROR) {
        CALL(uartDev->write((uint8_t *) "LED Success Init\r\n", 18));

        ledTID = sched_start(&ledTask, {});
        if (-1 == ledTID) {
            CALL(uartDev->write((uint8_t *) "Failed to init LED task\n\r", 25));
        }
    }

    CALL(uartDev->write((uint8_t *) "BMP Initializing\r\n", 18));

    static BMP3XX bmp(*i2cDev);
    bmp390 = &bmp;
    tid_t bmp390TID = -1;
    RetType bmp390Ret = CALL(bmp390->init());

    if (bmp390Ret == RET_ERROR) {
        CALL(uartDev->write((uint8_t *) "BMP Failed to Initialize\r\n", 26));
    } else {
        CALL(uartDev->write((uint8_t *) "BMP Initialized\n\r", 19));

        bmp390TID = sched_start(bmpTask, {});

        if (-1 == bmp390TID) {
            CALL(uartDev->write((uint8_t *) "BMP Task Startup Failed\n\r", 25));

        } else {
            CALL(uartDev->write((uint8_t *) "BMP Task Running\r\n", 18));
        }
    }



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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
    HALUARTDevice uart("UART", &huart2);
    RetType uartRet = uart.init();
    uartDev = &uart;

    char uartBuffer[MAX_UART_BUFF_SIZE];

    if(!sched_init(&HAL_GetTick)) {
        HAL_UART_Transmit_IT(&huart2, (uint8_t *) "Failed to init scheduler\n\r", 30);
        return -1;
    }

    // Initialize peripherals
    HALGPIODevice gpioDevice("LED GPIO", GPIOA, GPIO_PIN_5);
    RetType gpioRet = gpioDevice.init();
    LED localLED(gpioDevice);
    led = &localLED;

    static HALI2CDevice i2c("HAL I2C1", &hi2c1);
    if (i2c.init() != RET_SUCCESS) {
        HAL_UART_Transmit_IT(&huart2, (uint8_t *) "Failed to init I2C1 Device. Exiting.\n\r", 38);

        return -1;
    }

    i2cDev = &i2c;
    sched_start(i2cDevPollTask, {});
    sched_start(sensorInitTask, {});


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1) {
//        const char *whileString = "Task Dispatched\n\r";
//        HAL_UART_Transmit(&huart2, (uint8_t *) whileString, strlen(whileString), 100);
        sched_dispatch();
        HAL_Delay(25);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA5 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB6 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void print_bmp_data(BMP3XX *bmp) {
//    RetType bmpRetAPI = RET_SUCCESS;
//    bmp3_status bmpStatus = {};
//    bmp3_data bmpData = {};
//    uint8_t uartBuffer2[100];
//
//    bmpData = bmp->getSensorData();
//    HAL_UART_Transmit(&huart2, (const uint8_t *) "Readings: \n\r", 12, 100);
//    sprintf((char *) uartBuffer2, "\tTemperature: %f\r\n", bmpData.temperature);
//    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);
//    sprintf((char *) uartBuffer2, "\tPressure: %f\r\n", bmpData.pressure);
//    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);
//
//    bmpRetAPI = bmp->getStatus(&bmpStatus);
//    if (bmpRetAPI != RET_SUCCESS) {
//        const char *bmpErrStr = "Failed to get bmp390 status\n\r";
//        HAL_UART_Transmit(&huart2, (const uint8_t *) bmpErrStr, strlen(bmpErrStr), 100);
//    }
//
//    HAL_UART_Transmit(&huart2, (const uint8_t *) "Errors: \n\r", 10, 100);
//    sprintf((char *) uartBuffer2, "\tFatal: %d\r\n", bmpStatus.err.fatal);
//    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);
//
//    sprintf((char *) uartBuffer2, "\tCmd: %d\r\n", bmpStatus.err.cmd);
//    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);
//
//    sprintf((char *) uartBuffer2, "\tConf: %d\r\n", bmpStatus.err.conf);
//    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);
//
//    HAL_UART_Transmit(&huart2, (const uint8_t *) "------------------------\r\n", 26, 100);
//}
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
        HAL_UART_Transmit(&huart2, (uint8_t *) "Error\n\r", 7, 100);
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
