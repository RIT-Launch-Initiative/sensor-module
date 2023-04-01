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

#include "device/peripherals/LED/LED.h"
#include "device/peripherals/LIS3MDL/LIS3MDL.h"
#include "device/peripherals/SHTC3/SHTC3.h"
#include "device/peripherals/W25Q/W25Q.h"
#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/platforms/stm32/HAL_I2CDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/platforms/stm32/HAL_UARTDevice.h"

// #include "filesystem/ChainFS/ChainFS.h" // TODO: Unfinished
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
// static MS5607 *ms5607 = nullptr;
// static BMP3XX *bmp3XX = nullptr;
// static ADXL375 *adxl375 = nullptr;
static LIS3MDL *lis3mdl = nullptr;
static SHTC3 *shtc3 = nullptr;
static LED *led = nullptr;
HALUARTDevice *uartDev = nullptr;
static HALI2CDevice *i2cDev = nullptr;
// static TMP117 *tmp117 = nullptr;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RetType i2cDevPollTask(void *) {
    RESUME();
    CALL(i2cDev->poll());
    RESET();
    return RET_SUCCESS;
}

RetType ledTask(void *) {
    RESUME();

    CALL(led->toggle());

    RESET();
    return RET_SUCCESS;
}

RetType bmpTask(void *) {
    RESUME();

    //    RetType ret = CALL(bmp390->pullSensorData());

    RESET();
    return RET_SUCCESS;
}

RetType lisTask(void *) {
    RESUME();
    static float magX = 0;
    static float magY = 0;
    static float magZ = 0;
    static float temp = 0;

    RetType ret = CALL(lis3mdl->pullSensorData(&magX, &magY, &magZ, &temp));
    if (ret != RET_SUCCESS) {
        CALL(uartDev->write((uint8_t *)"LIS3MDL: Failed to get sensor data\r\n", 35));
    }

    static char buffer[100];
    size_t size = snprintf(buffer, 100, "Mag: \r\n\tX: %f\r\n\tY: %f\r\n\tZ: %f\r\nTemp: %f\r\n", magX, magY, magZ, temp);
    CALL(uartDev->write((uint8_t *)buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType shtTask(void *) {
    RESUME();
    static float temp = 0;
    static float humidity = 0;

    RetType ret = CALL(shtc3->getHumidityAndTemp(&temp, &humidity));
    if (ret == RET_ERROR) {
        CALL(uartDev->write((uint8_t *)"SHT: Error\r\n", 12));
    }

    static char buffer[150];
    size_t size = snprintf(buffer, 100, "Humidity: %f\r\nTemperature: %f\r\n", humidity, temp);
    CALL(uartDev->write((uint8_t *)buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType sensorInitTask(void *) {
    RESUME();

    // TODO: LED is not a sensor but here for testing purposes
    CALL(uartDev->write((uint8_t *)"LED: Initializing\r\n", 19));
    RetType ret = CALL(led->init());
    tid_t ledTID = -1;
    if (ret != RET_ERROR) {
        ledTID = sched_start(ledTask, {});

        if (-1 == ledTID) {
            CALL(uartDev->write((uint8_t *)"LED: Task Init Failed\r\n", 23));
        } else {
            CALL(uartDev->write((uint8_t *)"LED: Initialized\r\n", 18));
        }
    }

    // CALL(uartDev->write((uint8_t *)"LSM6DSL: Initializing\r\n", 23));
    // static LSM6DSL lsm(*i2cDev);
    // lsm6dsl = &lsm;
    // tid_t lsmTID = -1;
    // RetType lsm6dslRet = CALL(lsm6dsl->init());
    // if (lsm6dslRet != RET_ERROR) {
    //     lsmTID = sched_start(lsmTask, {});

    //     if (-1 == lsmTID) {
    //         CALL(uartDev->write((uint8_t *)"LSM6DSL: Task Init Failed\r\n", 27));
    //     } else {
    //         CALL(uartDev->write((uint8_t *)"LSM6DSL: Initialized\r\n", 22));
    //     }
    // } else {
    //     CALL(uartDev->write((uint8_t *)"LSM6DSL: Sensor Init Failed\r\n", 29));
    // }

    // CALL(uartDev->write((uint8_t *)"MS5607: Initializing\r\n", 22));
    // static MS5607 ms5(*i2cDev);
    // ms5607 = &ms5;
    // tid_t ms5TID = -1;
    // RetType ms5Ret = CALL(ms5607->init());
    // if (ms5Ret != RET_ERROR) {
    //     ms5TID = sched_start(ms5607Task, {});

    //     if (-1 == ms5TID) {
    //         CALL(uartDev->write((uint8_t *)"MS5607: Task Init Failed\r\n", 26));
    //     } else {
    //         CALL(uartDev->write((uint8_t *)"MS5607: Initialized\r\n", 21));
    //     }
    // } else {
    //     CALL(uartDev->write((uint8_t *)"LED: Device Init Failed\r\n", 25));
    // }

    //    CALL(uartDev->write((uint8_t *) "LIS: Initializing\r\n", 19));
    //    static LIS3MDL lis(*i2cDev);
    //    lis3mdl = &lis;
    //    tid_t lisTID = -1;
    //    RetType lis3mdlRet = CALL(lis3mdl->init());
    //    if (lis3mdlRet != RET_ERROR) {
    //        lisTID = sched_start(lisTask, {});
    //
    //        if (-1 == lisTID) {
    //            CALL(uartDev->write((uint8_t *) "LIS: Task Init Failed\r\n", 23));
    //        } else {
    //            CALL(uartDev->write((uint8_t *) "LIS: Initialized\r\n", 18));
    //        }
    //    } else {
    //        CALL(uartDev->write((uint8_t *) "LIS: Sensor Init Failed\r\n", 25));
    //    }

    CALL(uartDev->write((uint8_t *)"SHT: Initializing\r\n", 19));
    static SHTC3 sht(*i2cDev);
    shtc3 = &sht;
    tid_t shtTID = -1;
    RetType sht3mdlRet = CALL(shtc3->init());
    if (sht3mdlRet != RET_ERROR) {
        shtTID = sched_start(shtTask, {});

        if (-1 == shtTID) {
            CALL(uartDev->write((uint8_t *)"SHT: Task Init Failed\r\n", 23));
        } else {
            CALL(uartDev->write((uint8_t *)"SHT: Initialized\r\n", 19));
        }
    } else {
        CALL(uartDev->write((uint8_t *)"SHT: Sensor Init Failed\r\n", 25));
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
    RetType ret = uart.init();
    if (ret != RET_SUCCESS) {
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Failed to init UART Device. Exiting.\n\r", 38);
        return -1;
    }
    uartDev = &uart;

    char uartBuffer[MAX_UART_BUFF_SIZE];

    if (!sched_init(&HAL_GetTick)) {
        snprintf(uartBuffer, MAX_UART_BUFF_SIZE, "Failed to init scheduler\n\r");
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer));
        return -1;
    }

    // Initialize peripherals
    HALGPIODevice gpioDevice("LED GPIO", GPIOA, GPIO_PIN_5);
    ret = gpioDevice.init();
    LED localLED(gpioDevice);
    led = &localLED;

    static HALI2CDevice i2c("HAL I2C1", &hi2c1);
    if (i2c.init() != RET_SUCCESS) {
        snprintf(uartBuffer, MAX_UART_BUFF_SIZE, "Failed to init I2C1 Device. Exiting.\n\r");
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer));

        return -1;
    }

    i2cDev = &i2c;
    sched_start(i2cDevPollTask, {});
    sched_start(sensorInitTask, {});

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1) {
        sched_dispatch();
        HAL_Delay(50);
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
        HAL_UART_Transmit(&huart2, (uint8_t *)"Error\n\r", 7, 100);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
