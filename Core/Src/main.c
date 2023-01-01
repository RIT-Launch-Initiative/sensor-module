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
#include "device/platforms/stm32/HAL_TimerDevice.h"



#include "device/peripherals/LED/LED.h"
#include "device/peripherals/W25Q/W25Q.h"
//#include "device/peripherals/BMP390/BMP390.h"
#include "device/peripherals/BMP390/BMP3902.h"



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
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

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

static void MX_I2C2_Init(void);

static void MX_I2C3_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
static void print_bmp_data(BMP390 *bmp);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */
    HALTimerDevice timer = HALTimerDevice();

    HALUARTDevice uart("UART", &huart2);
    uint8_t uartBuffer[100] = "Launch Initiative\r\n";
    RetType uartRet = uart.init();

    HALGPIODevice gpioDevice("LED GPIO", GPIOA, GPIO_PIN_5);
    LED led(gpioDevice);
    RetType gpioRet = gpioDevice.init();
    RetType ledRet = led.init();

    HALSPIDevice spiDevice("W25Q SPI", &hspi1);
    HALGPIODevice csPin("W25Q CS", GPIOA, GPIO_PIN_6);
    HALGPIODevice clkPin("W25Q CLK", GPIOB, GPIO_PIN_13);
    W25Q w25q(spiDevice, csPin, clkPin);
    spiDevice.init();
    csPin.init();
    clkPin.init();
    RetType w25qInit = w25q.init();
    if (w25qInit != RET_SUCCESS) {
        HAL_UART_Transmit(&huart2, (uint8_t *) "W25Q init failed\r\n", 18, 100);
    }


    w25q.toggleWrite(WRITE_SET_ENABLE);

    HALI2CDevice bmpI2C = HALI2CDevice("BMP390 I2C", &hi2c1);
    RetType bmpI2CRet = bmpI2C.init();
    HAL_Delay(1000);

    BMP390 bmp390(&bmpI2C, &timer);
    RetType bmpRet = bmp390.init();
    if (bmpRet != RET_SUCCESS) {
        const char *bmpErrStr = "Failed to init bmp390\n\r";
        HAL_UART_Transmit(&huart2, (const uint8_t *) bmpErrStr, strlen(bmpErrStr), 100);
    }

    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        led.toggle();
        print_bmp_data(&bmp390);

        HAL_Delay(1000);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void) {

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */

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
void bmp_delay(uint32_t period, void *intf_ptr) {
    (void) intf_ptr;
    HAL_Delay(period);
}

int8_t bmp_write(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intfPtr) {
    uint8_t deviceAddr = *(uint8_t *) intfPtr;
    // TODO: Interrupt Mode <3
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, deviceAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT,
                                                 (uint8_t *) data, len, 10);

    return status == HAL_OK ? 0 : -1;
}

int8_t bmp_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intfPtr) {
    uint8_t deviceAddr = *(uint8_t *) intfPtr;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, deviceAddr << 1, regAddr, 1, data, len, 10);
    // TODO: Interrupt mode not working atm. Check on this later.
//    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(&hi2c1, 0x77 << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, len);

    return status == HAL_OK ? 0 : -1;
}


int8_t bmp_write_it(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intfPtr) {
    uint8_t deviceAddr = *(uint8_t *) intfPtr;
    // TODO: Interrupt Mode <3

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(&hi2c1, deviceAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT,
                                                 (uint8_t *) data, len);

    HAL_Delay(25);


    return status == HAL_OK ? 0 : -1;
}

int8_t bmp_read_it(uint8_t regAddr, uint8_t *data, uint32_t len, void *intfPtr) {
    uint8_t deviceAddr = *(uint8_t *) intfPtr;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(&hi2c1, deviceAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, len);
    HAL_Delay(25);

    return status == HAL_OK ? 0 : -1;
}

void print_bmp_data(BMP390 *bmp) {
    RetType bmpRetAPI = RET_SUCCESS;
    bmp3_status bmpStatus = {};
    bmp3_data bmpData = {};
    uint8_t uartBuffer2[100];

    bmpData = bmp->getSensorData();
    HAL_UART_Transmit(&huart2, (const uint8_t *) "Readings: \n\r", 12, 100);
    sprintf((char *) uartBuffer2, "\tTemperature: %f\r\n", bmpData.temperature);
    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);
    sprintf((char *) uartBuffer2, "\tPressure: %f\r\n", bmpData.pressure);
    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);

    bmpRetAPI = bmp->getStatus(&bmpStatus);
    if (bmpRetAPI != RET_SUCCESS) {
        const char *bmpErrStr = "Failed to get bmp390 status\n\r";
        HAL_UART_Transmit(&huart2, (const uint8_t *) bmpErrStr, strlen(bmpErrStr), 100);
    }

    HAL_UART_Transmit(&huart2, (const uint8_t *) "Errors: \n\r", 10, 100);
    sprintf((char *) uartBuffer2, "\tFatal: %d\r\n", bmpStatus.err.fatal);
    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);

    sprintf((char *) uartBuffer2, "\tCmd: %d\r\n", bmpStatus.err.cmd);
    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);

    sprintf((char *) uartBuffer2, "\tConf: %d\r\n", bmpStatus.err.conf);
    HAL_UART_Transmit(&huart2, uartBuffer2, strlen((char *) uartBuffer2), 100);

    HAL_UART_Transmit(&huart2, (const uint8_t *) "------------------------\r\n", 26, 100);

}

void print_bmp_data_it(BMP390 *bmp, HALUARTDevice *uart) {
    RetType bmpRetAPI = RET_SUCCESS;
    bmp3_status bmpStatus = {};
    bmp3_data bmpData = {};
    uint8_t uartBuffer2[100];

    bmpData = bmp->getSensorData();
    uart->write((uint8_t *) "Temperature: ", 13);
    sprintf((char *) uartBuffer2, "\tTemperature: %f\r\n", bmpData.temperature);
    uart->write(uartBuffer2, strlen((char *) uartBuffer2));
    uart->write((uint8_t *) "Pressure: ", 10);
    sprintf((char *) uartBuffer2, "\tPressure: %f\r\n", bmpData.pressure);
    uart->write(uartBuffer2, strlen((char *) uartBuffer2));

    bmpRetAPI = bmp->getStatus(&bmpStatus);
    if (bmpRetAPI != RET_SUCCESS) {
        const char *bmpErrStr = "Failed to get bmp390 status\n\r";
        HAL_UART_Transmit(&huart2, (const uint8_t *) bmpErrStr, strlen(bmpErrStr), 100);
    }

    uart->write((uint8_t *) "Errors: \n\r", 10);
    sprintf((char *) uartBuffer2, "\tFatal: %d\r\n", bmpStatus.err.fatal);
    uart->write(uartBuffer2, strlen((char *) uartBuffer2));

    sprintf((char *) uartBuffer2, "\tCmd: %d\r\n", bmpStatus.err.cmd);
    uart->write(uartBuffer2, strlen((char *) uartBuffer2));

    sprintf((char *) uartBuffer2, "\tConf: %d\r\n", bmpStatus.err.conf);
    uart->write(uartBuffer2, strlen((char *) uartBuffer2));
    uart->write((uint8_t *) "------------------------\r\n", 26);

}


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
