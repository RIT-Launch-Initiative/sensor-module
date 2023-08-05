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

#include "SensorModuleDeviceMap.h"

#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/platforms/stm32/HAL_UARTDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/peripherals/LED/LED.h"
#include "device/platforms/stm32/HAL_I2CDevice.h"
// #include "device/peripherals/W25Q/W25Q.h"
#include "device/peripherals/ADXL375/ADXL375.h"
#include "device/peripherals/BMP3XX/BMP3XX.h"
#include "device/peripherals/LIS3MDL/LIS3MDL.h"
#include "device/peripherals/LSM6DSL/LSM6DSL.h"
//#include "device/peripherals/MS5607/MS5607.h"
#include "device/peripherals/SHTC3/SHTC3.h"
#include "device/peripherals/TMP117/TMP117.h"
#include "sched/macros.h"
#include "device/DeviceMap.h"
#include "init/init.h"


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
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
SensorModuleDeviceMap *deviceMap;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_SPI1_Init(void);

static void MX_SPI2_Init(void);

static void MX_I2C3_Init(void);

static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RetType ledTask(void *) {
    RESUME();
    static auto *led = (LED *) deviceMap->get("i2c");

    CALL(led->toggle());

    RESET();
    return RET_SUCCESS;
}

RetType bmpTask(void *) {
    RESUME();
    static auto *bmp3xx = (BMP3XX *) deviceMap->get("bmp3xx");
    static auto *uartDev = (StreamDevice *) deviceMap->get("uart");

    static char buffer[100];
    static double pressure = 0;
    static double temperature = 0;

    RetType ret = CALL(bmp3xx->getPressureAndTemp(&pressure, &temperature));
    if (ret == RET_ERROR) {
        CALL(uartDev->write((uint8_t *) "Failed to get BMP data\r\n", 24));
    }

    size_t size = sprintf(buffer, "BMP Pressure: %f Pa \r\nBMP Temperature: %f C\r\n", pressure, temperature);
    CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType tmpTask(void *) {
    RESUME();
    static auto *uartDev = (StreamDevice *) deviceMap->get("uart");
    static auto *tmp117 = (TMP117 *) deviceMap->get("tmp117");

    static char buffer[100];
    static float temp = 0;

    RetType ret = CALL(tmp117->readTempCelsius(&temp));
    if (ret == RET_ERROR) {
        CALL(uartDev->write((uint8_t *) "Failed to get TMP data\r\n", 9));
    }

    size_t size = sprintf(buffer, "TMP Temperature: %f C\r\n", temp);
    CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType adxlTask(void *) {
    RESUME();
    static auto *adxl375 = (ADXL375 *) deviceMap->get("adxl375");
    static auto *uartDev = (StreamDevice *) deviceMap->get("uart");

    static int16_t x = 0;
    static int16_t y = 0;
    static int16_t z = 0;

    RetType ret = CALL(adxl375->readXYZ(&x, &y, &z));
    if (ret != RET_SUCCESS) {
        HAL_UART_Transmit(&huart5, (uint8_t *) "ADXL Task Failed\r\n", 18, 100);
        return ret;
    }

    static char buffer[100];
    size_t size = snprintf(buffer, 100, "ADXL375: x: %d, y: %d, z: %d\r\n", x, y, z);

    // Use below if you want to print the values in multiple lines
    // size_t size = snprintf(buffer, 100, "ADXL375:\r\n\tX-Axis: %d m/s^2\r\n\tY-Axis: %d m/s^2\r\n\tZ-Axis: %d m/s^2\r\n", x, y, z);

    CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType lsmTask(void *) {
    RESUME();
    static auto *lsm6dsl = (LSM6DSL *) deviceMap->get("lsm6dsl");
    static auto *uartDev = (StreamDevice *) deviceMap->get("uart");

    static int32_t accX = 0;
    static int32_t accY = 0;
    static int32_t accZ = 0;

    static int32_t gyroX = 0;
    static int32_t gyroY = 0;
    static int32_t gyroZ = 0;

    RetType ret = CALL(lsm6dsl->getAccelAxesMS2(&accX, &accY, &accZ));
    if (ret != RET_SUCCESS) {
        CALL(uartDev->write((uint8_t *) "LSM6DSL: Failed to get Accel Axes\r\n", 34));
    }

    ret = CALL(lsm6dsl->getGyroAxes(&gyroX, &gyroY, &gyroZ));
    if (ret != RET_SUCCESS) {
        CALL(uartDev->write((uint8_t *) "LSM6DSL: Failed to get Gyro Axes\r\n", 34));
    }

    static char buffer[120];
    size_t size = snprintf(buffer, 120,
                           "LSM6DSL: \r\n\tAccel: \r\n\t\tX: %ld m/s^2\r\n\t\tY: %ld m/s^2\r\n\t\tZ: %ld m/s^2\r\n\tGyro: \r\n\t\tX: %ld dps\r\n\t\tY: %ld dps\r\n\t\tZ: %ld dps\r\n",
                           accX, accY, accZ, gyroX, gyroY, gyroZ);
    CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType lisTask(void *) {
    RESUME();
    static auto *lis3mdl = (LIS3MDL *) deviceMap->get("lis3mdl");
    static auto *uartDev = (StreamDevice *) deviceMap->get("uart");

    static float magX = 0;
    static float magY = 0;
    static float magZ = 0;
    static float temp = 0;

    RetType ret = CALL(lis3mdl->pullSensorData(&magX, &magY, &magZ, &temp));
    if (ret != RET_SUCCESS) {
        CALL(uartDev->write((uint8_t *) "LIS3MDL: Failed to get sensor data\r\n", 35));
    }

    static char buffer[100];
    size_t size = snprintf(buffer, 100, "Mag: \r\n\tX: %f\r\n\tY: %f\r\n\tZ: %f\r\nTemp: %f\r\n", magX, magY, magZ,
                           temp);
    CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

//RetType ms5607Task(void *) {
//    RESUME();
//    static auto *ms5607 = (MS5607 *) deviceMap->get("ms5607");
//    static auto *uartDev = (StreamDevice *) deviceMap->get("uart");
//
//    static float pressure = 0;
//    static float temperature = 0;
//
//    RetType ret = CALL(ms5607->getPressureTemp(&pressure, &temperature));
//    if (ret == RET_ERROR) {
//        CALL(uartDev->write((uint8_t *) "Failed to get MS5607 data\r\n", 27));
//    }
//
//    static float altitude = ms5607->getAltitude(pressure, temperature);
//
//    static char buffer[100];
//    size_t size = sprintf(buffer, "MS5607:\r\n\tPressure: %.2f mBar\r\n\tTemperature: %.2f C\r\n\tAltitude: %f\r\n",
//                          pressure, temperature, altitude);
//    CALL(uartDev->write((uint8_t *) buffer, size));
//
//    RESET();
//    return RET_SUCCESS;
//}

RetType shtc3Task(void *) {
    RESUME();
    static auto *shtc3 = (SHTC3 *) deviceMap->get("shtc3");
    static auto *uartDev = (StreamDevice *) deviceMap->get("uart");
    CALL(uartDev->write((uint8_t *) "SHTC3: Started\r\n", 16));
    static float temp = 0;
    static float humidity = 0;

    RetType ret = CALL(shtc3->getHumidityAndTemp(&temp, &humidity));
    if (ret == RET_ERROR) {
        CALL(uartDev->write((uint8_t *) "SHT: Error\r\n", 12));
    }

    static char buffer[150];
    size_t size = snprintf(buffer, 100, "Humidity: %f\r\nTemperature: %f\r\n", humidity, temp);
    CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
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
    if (!sched_init(&HAL_GetTick)) {
        HAL_UART_Transmit_IT(&huart5, (uint8_t *) "Failed to init scheduler\n\r", 30);
        return -1;
    }

    // Initialize peripherals
    HALUARTDevice uart("UART", &huart5);
    HALI2CDevice i2c("HAL I2C1", &hi2c3);

    HALGPIODevice ledGPIOOne("LED GPIO", PA1_LED_GPIO_Port, PA1_LED_Pin);
    LED localLEDOne(ledGPIOOne);

    HALGPIODevice ledGPIOTwo("LED GPIO", PA2_LED_GPIO_Port, PA2_LED_Pin);
    LED localLEDTwo(ledGPIOTwo);


    HALSPIDevice wiznetSPI("Wiznet SPI", &hspi1);
    HALGPIODevice wiznetCS("Wiznet CS", ETH_CS_GPIO_Port, ETH_CS_Pin);
    HALGPIODevice wiznetLED("Wiznet LED", Wiz_LED_GPIO_Port, Wiz_LED_Pin);


    HALSPIDevice flashSPI("Flash SPI", &hspi2);
    HALGPIODevice flashCS("Flash CS", WS25_CS_GPIO_Port, WS25_CS_Pin);

    SensorModuleDeviceMap map(i2c, wiznetSPI, flashSPI, wiznetCS, flashCS, ledGPIOOne, ledGPIOTwo, wiznetLED, uart);
    deviceMap = &map;

    if (RET_SUCCESS != map.init()) {
        swprintf("Failed to initialize Device Map!");
        HAL_UART_Transmit(&huart5, (uint8_t *) "Failed to init SensorModuleDeviceMap. Exiting.\r\n", 47, 1000);
        return -1;
    } else {
        swprintf("Initialized Device Map!");
    }

    const int num_tasks = 7; // Increment for each task
//    task_func_t tasks[12] = {ledTask, ms5607Task, bmpTask, adxlTask, lisTask, lsmTask, shtc3Task, tmpTask};
    task_func_t tasks[num_tasks] = {ledTask, bmpTask, adxlTask, lisTask, lsmTask, shtc3Task, tmpTask};

    init_arg_t initArgs = {
            .dev_map = deviceMap,
            .tasks = tasks,
            .args = {},
            .num_tasks = num_tasks,
    };
    map.print();
    sched_start(init, static_cast<void *>(&initArgs));
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
        HAL_UART_Transmit(&huart5, (uint8_t *) "Error\n\r", 7, 100);
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
       ex: printf("Wrong parameters value: file %s on line %ld\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
