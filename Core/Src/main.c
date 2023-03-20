/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  ******************************************************************************}}}}}
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
#include <stdarg.h>
#include <string.h>

#include "device/platforms/stm32/HAL_GPIODevice.h"
//#include "device/platforms/stm32/HAL_UARTDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
//#include "device/platforms/stm32/HAL_I2CDevice.h"


#include "device/peripherals/LED/LED.h"
//#include "device/peripherals/W25Q/W25Q.h"
//#include "device/peripherals/BMP3XX/BMP3XX.h"
//#include "device/peripherals/ADXL375/ADXL375.h"
#include "device/peripherals/LIS3MDL/LIS3MDL.h"
//#include "device/peripherals/LSM6DSL/LSM6DSL.h"

#include "sched/macros/call.h"


//#include "filesystem/ChainFS/ChainFS.h" // TODO: Unfinished
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_MAX_BUFF 120
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
const char* ok = "OK\r\n";
const char* fail = "FAILED\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
//static HALUARTDevice *uart_dev = nullptr;
//static HALI2CDevice *i2c_dev = nullptr;

static HALSPIDevice *spi_dev = nullptr;

static HALGPIODevice *gpio_led = nullptr;
static HALGPIODevice *gpio_mag = nullptr;
static LED *led = nullptr;
//static LIS3MDL *mag = nullptr;
//static LIS3MDL_SPI *mag_spi = nullptr;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int print_uart(const char* fmt, ...)
{
	char buf[UART_MAX_BUFF];
	int status;
	va_list ap;

	va_start(ap, fmt);
	status = vsnprintf(buf, UART_MAX_BUFF, fmt, ap);
	va_end(ap);
	if (status >= UART_MAX_BUFF || status < 0) {
		return -1;
	}

	status = (int) HAL_UART_Transmit(&huart2,
			(const uint8_t*) buf,
			(uint16_t) (status + 1), 100);
	return status;
}

RetType led_toggle_task(void*) {
	RESUME();
	CALL(led->toggle());
	SLEEP(500);

	RESET();
	return RET_SUCCESS;
}

//RetType i2c_device_poll(void*) {
//	RESUME();
//	CALL(i2c_dev->poll());
//	RESET();
//	return RET_SUCCESS;
//}

RetType spi_device_poll(void*) {
	RESUME();
	CALL(spi_dev->poll());
	RESET();
	return RET_SUCCESS;
}

RetType mag_task(void*) {
	RESUME();

	static float x = 0;
	static float y = 0;
	static float z = 0;
	static float temp = 0;
	static RetType status;


	/*
	// dummy read of i2c address
	static uint8_t whoami = 0;
    I2CAddr_t mag_addr = {
            .dev_addr = 0x1C << 1,
            .mem_addr = 0x0F,
            .mem_addr_size = sizeof(uint8_t),
    };
	status = CALL(i2c_dev->read(mag_addr , &whoami, 1, 50));
	if (status == RET_SUCCESS){
		print_uart("WHOAMI: 0x%2x\r\n", whoami);
	} else {
		print_uart("Failed to read using I2C device\r\n");
	}
/**/

//	status = CALL(mag->pullSensorData(&x, &y, &z, &temp));
//	status = CALL(mag_spi->pullSensorData(&x, &y, &z, &temp));

	if (status == RET_SUCCESS) {
		print_uart("X: %5.3f, Y: %5.3f, Z: %5.3f, T: %3.1f\r\n", x, y, z, temp);
	} else {
		print_uart("Failed to read data from mag\r\n");
	}
/**/
	SLEEP(100);

	RESET();
	return RET_SUCCESS;
}

int report_hal(HAL_StatusTypeDef status) {
        switch (status) {
			case(HAL_OK):
				print_uart("HAL OK... ");
				return 0;
				break;
			case(HAL_BUSY):
				print_uart("HAL BUSY... ");
				return 1;
				break;
			case(HAL_TIMEOUT):
				print_uart("HAL TIMEOUT... ");
				return 1;
				break;
			case(HAL_ERROR):
				print_uart("HAL ERROR... ");
				return 1;
				break;
        }
}


RetType mag_init_task(void*) {
	RESUME();
	static RetType status;
//	static tid_t mag_tid = -1;
//	static LIS3MDL mag_local(*i2c_dev);
//	mag = &mag_local;
//	static LIS3MDL_SPI mag_spi_local(*spi_dev, *gpio_mag);
//	mag_spi = &mag_spi_local;

	// dummy read of i2c address

/*
	static uint8_t whoami = 0;
    I2CAddr_t mag_addr = {
            .dev_addr = 0x1C << 1,
            .mem_addr = 0x0F,
            .mem_addr_size = sizeof(uint8_t),
    };
	status = CALL(i2c_dev->read(mag_addr , &whoami, 1, 50));
	if (status == RET_SUCCESS){
		print_uart("WHOAMI: 0x%2x\r\n", whoami);
	} else {
		print_uart("Failed to read using I2C device\r\n");
	}
/**/

//	print_uart("Reading control registers before init\r\n");
	const static uint32_t tmt = 200;
	static uint8_t instruction = 0x0FU | 0b1100000;
	static uint8_t whoami = 0;
	static uint8_t controls[5];

//	print_uart("Setting gpio... ");
//	status = CALL(gpio_mag->set(0));
//	if (status == RET_SUCCESS) {
//		print_uart(ok);
//	} else {
//		print_uart(fail);
//	}

//	print_uart("Writing to device... ");
//	status = CALL(spi_dev->write(&instruction, 1, tmt));
//	if (status == RET_SUCCESS) {
//		print_uart(ok);
//	} else {
//		print_uart(fail);
//	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(300);

	HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(&hspi1, &instruction, 1, tmt);
	report_hal(hal_status);
	hal_status = HAL_SPI_Receive(&hspi1, &whoami, 1, tmt);
	report_hal(hal_status);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

//	print_uart("Reading from device... ");
//	status = CALL(spi_dev->read(&whoami, 1, tmt));
//	if (status == RET_SUCCESS) {
//		print_uart(ok);
//	} else {
//		print_uart(fail);
//	}

//	print_uart("Unsetting GPIO... ");
//	status = CALL(gpio_mag->set(1));
//	if (status == RET_SUCCESS) {
//		print_uart(ok);
//	} else {
//		print_uart(fail);
//	}

	print_uart("WHOAMI: 0x%02x\r\n", whoami);

/*
	if (status == RET_SUCCESS) {
		print_uart("Magnetometer control registers before init: 0x");
		for (int i = 0; i < 5; i++) {
			print_uart("%02x", controls[i]);
		}
		print_uart("\r\n");
	} else {
		print_uart("Failed to read control registers from magnetometer\r\n");
	}
/**/

//	print_uart("Initailizing mag... ");
//	status = CALL(mag->init());
	/*
	status = CALL(mag_spi->init());

	if (status == RET_SUCCESS) {
		print_uart(ok);
		print_uart("Starting mag task... ");
		mag_tid = sched_start(mag_task, {});

		if (mag_tid == -1) {
			print_uart(fail);
		} else {
			print_uart(ok);
		}
	} else {
		print_uart(fail);
	}


	CALL(gpio_mag->set(0));
	CALL(spi_dev->write(&instruction, 1, tmt));
	status = CALL(spi_dev->read(controls, 5, tmt));
	CALL(gpio_mag->set(1));

	if (status == RET_SUCCESS) {
		print_uart("Magnetometer control registers after init: 0x");
		for (int i = 0; i < 5; i++) {
			print_uart("%02x", controls[i]);
		}
		print_uart("\n");
	} else {
		print_uart("Failed to read control registers from magnetometer\n");
	}
/**/

	RESET();
	return RET_ERROR;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  	RetType status;
	print_uart("Initializing scheduler: ");
	if (sched_init(&HAL_GetTick)) {
		print_uart(ok);
	} else {
		print_uart(fail);
		return -1;
	}

  	print_uart("\r\nInitializing devices...\r\n");

  	// Initialize GPIO Device for LED
	HALGPIODevice gpio_led_local("LED", GPIOA, GPIO_PIN_5);
	gpio_led = &gpio_led_local;

	LED led_local(*gpio_led);
	led = &led_local;


	// GPIO device for mag
	HALGPIODevice gpio_mag_local("MAG", GPIOB, GPIO_PIN_6);
	gpio_mag = &gpio_mag_local;
	gpio_mag->set(0);
/**/


	// Initialize UART device
//	HALUARTDevice uart_local("UART", &huart2);
//	print_uart("Initializing UART device... ");
//	status = uart_local.init();
//	if (status == RET_SUCCESS) {
//		print_uart(ok);
//		uart_dev = &uart_local;
//	} else {
//		print_uart(fail);
//	}

	// Initialize I2C device
//	HALI2CDevice i2c_local("I2C", &hi2c1);
//	print_uart("Initializing I2C device... ");
//	status = i2c_local.init();
//	if (status == RET_SUCCESS) {
//		i2c_dev = &i2c_local;
//		print_uart(ok);
//	} else {
//		print_uart(fail);
//	}

	// Initialize SPI device


	HALSPIDevice spi_local("SPI", &hspi1);
	print_uart("Initializing SPI device... ");
	status = spi_local.init();
	if (status == RET_SUCCESS) {
		spi_dev = &spi_local;
		print_uart(ok);
	} else {
		print_uart(fail);
	}
/**/

//	sched_start(gpio_device_poll, {});
//	sched_start(i2c_device_poll, {});
	sched_start(spi_device_poll, {});
	sched_start(mag_init_task, {});
//	sched_start(led_toggle_task, {});

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1) {
    	sched_dispatch();
//    	HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  return 0;
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
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
static void MX_SPI1_Init(void)
{

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_SPI2_Init(void)
{

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
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

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
void Error_Handler(void)
{
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
     ex: printf("Wrong parameters value: file %s on line %ld\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
