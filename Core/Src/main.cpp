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
#include "device/peripherals/LED/LED.h"
#include "device/platforms/stm32/HAL_I2CDevice.h"
// #include "device/peripherals/W25Q/W25Q.h"
#include "device/peripherals/ADXL375/ADXL375.h"
#include "device/peripherals/BMP3XX/BMP3XX.h"
#include "device/peripherals/LIS3MDL/LIS3MDL.h"
#include "device/peripherals/LSM6DSL/LSM6DSL.h"
#include "device/peripherals/MS5607/MS5607.h"
#include "device/peripherals/SHTC3/SHTC3.h"
#include "device/peripherals/TMP117/TMP117.h"
#include "device/peripherals/W5500/W5500.h"

#include "net/packet/Packet.h"
#include "net/stack/IPv4UDP/IPv4UDPStack.h"
#include "net/stack/IPv4UDP/IPv4UDPSocket.h"

#include "sched/macros/call.h"


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

static W5500 *w5500 = nullptr;
static IPv4UDPStack *stack = nullptr;
static IPv4UDPSocket *sock = nullptr;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RetType i2cDevPollTask(void *) {
    RESUME();
    CALL(i2cDev->poll());
    RESET();
    return RET_SUCCESS;
}

RetType spiDevPollTask(void *) {
    RESUME();
    CALL(wizSPI->poll());
//    CALL(flashSPI->poll());
    RESET();
    return RET_SUCCESS;
}

RetType startupLEDTask(void *) {
    RESUME();

    CALL(ledOne->toggle());
    CALL(ledTwo->toggle());

    RESET();
    return RET_SUCCESS;
}

RetType wizLEDToggleTask(void *) {
    RESUME();

    CALL(wizLED->toggle());

    RESET();
    return RET_SUCCESS;
}

RetType bmpTask(void *) {
    RESUME();

//    static char buffer[100];
    static BMP3XX_DATA_STRUCT(bmp_data);

    RetType ret = CALL(bmp3XX->getPressureAndTemp(&bmp_data.pressure, &bmp_data.temperature));
    if (ret == RET_ERROR) {
        // CALL(uartDev->write((uint8_t *) "Failed to get BMP data\r\n", 24));
        RESET();
        return RET_SUCCESS;
    }

//     size_t size = sprintf(buffer, "BMP Pressure: %f Pa \r\nBMP Temperature: %f C\r\n", pressure, temperature);
    // CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType tmpTask(void *) {
    RESUME();
    static IPv4UDPSocket::addr_t addr;
    addr.ip[0] = 10;
    addr.ip[1] = 10;
    addr.ip[2] = 10;
    addr.ip[3] = 69;
    addr.port = 8000;

    static char buffer[100];
    static TMP117_DATA_STRUCT(tmp_data);

    RetType ret = CALL(tmp117->readTempCelsius(&tmp_data.temperature));
    if (ret == RET_ERROR) {
        // CALL(uartDev->write((uint8_t *) "Failed to get TMP data\r\n", 9));
        RESET();
        return RET_SUCCESS;
    }

//    size_t size = sprintf(buffer, "TMP Temperature: %f C\r\n", data.temp);
    // CALL(uartDev->write((uint8_t *) buffer, size));

    Packet packet = alloc::Packet<32, 0>();
    packet.push<TMP117_DATA_T>(tmp_data);

    sock->send(packet.read_ptr<uint8_t>(), packet.size(), &addr);

    RESET();
    return RET_SUCCESS;
}

RetType adxlTask(void *) {
    RESUME();
    static ADXL375_DATA_STRUCT(adxl_data);

    RetType ret = CALL(adxl375->readXYZ(&adxl_data.x_accel, &adxl_data.y_accel, &adxl_data.z_accel));
    if (ret != RET_SUCCESS) {
       // CALL(uartDev->write((uint8_t *) "Failed to get ADXL data\r\n", 24)
        RESET();
        return RET_SUCCESS;
    }

//    static char buffer[100];
//    size_t size = snprintf(buffer, 100, "ADXL375: x: %d, y: %d, z: %d\r\n", x, y, z);

    // Use below if you want to print the values in multiple lines
    // size_t size = snprintf(buffer, 100, "ADXL375:\r\n\tX-Axis: %d m/s^2\r\n\tY-Axis: %d m/s^2\r\n\tZ-Axis: %d m/s^2\r\n", x, y, z);

    // CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType lsmTask(void *) {
    RESUME();

    static LSM6DSL_DATA_STRUCT(lsm_data);
    RetType ret = CALL(lsm6dsl->getAccelAxesMS2(&lsm_data.x_gyro, &lsm_data.y_gyro, &lsm_data.z_gyro));
    if (ret != RET_SUCCESS) {
        // CALL(uartDev->write((uint8_t *) "LSM6DSL: Failed to get Accel Axes\r\n", 34));
        RESET();
        return RET_SUCCESS;
    }

    ret = CALL(lsm6dsl->getGyroAxes(&lsm_data.x_gyro, &lsm_data.y_gyro, &lsm_data.z_gyro));
    if (ret != RET_SUCCESS) {
        // CALL(uartDev->write((uint8_t *) "LSM6DSL: Failed to get Gyro Axes\r\n", 34));
        RESET();
        return RET_SUCCESS;
    }

//    static char buffer[120];
//    size_t size = snprintf(buffer, 120,
//                           "LSM6DSL: \r\n\tAccel: \r\n\t\tX: %ld m/s^2\r\n\t\tY: %ld m/s^2\r\n\t\tZ: %ld m/s^2\r\n\tGyro: \r\n\t\tX: %ld dps\r\n\t\tY: %ld dps\r\n\t\tZ: %ld dps\r\n",
//                           accX, accY, accZ, gyroX, gyroY, gyroZ);
    // CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType lisTask(void *) {
    RESUME();
    static LIS3MDL_DATA_STRUCT(lis_data);
    RetType ret = CALL(lis3mdl->pullSensorData(&lis_data.x_mag, &lis_data.y_mag, &lis_data.z_mag, &lis_data.temperature));
    if (ret != RET_SUCCESS) {
        // CALL(uartDev->write((uint8_t *) "LIS3MDL: Failed to get sensor data\r\n", 35));
        RESET();
        return RET_SUCCESS;
    }

//    static char buffer[100];
//    size_t size = snprintf(buffer, 100, "Mag: \r\n\tX: %f\r\n\tY: %f\r\n\tZ: %f\r\nTemp: %f\r\n", magX, magY, magZ,
//                           temp);
    // CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType ms5607Task(void *) {
    RESUME();

    static MS5607_DATA_STRUCT(ms5607_data);

    RetType ret = CALL(ms5607->getPressureTemp(&ms5607_data.pressure, &ms5607_data.temperature));
    if (ret == RET_ERROR) {
        // CALL(uartDev->write((uint8_t *) "Failed to get MS5607 data\r\n", 27));
        RESET();
        return RET_SUCCESS;
    }

//    static float altitude = ms5607->getAltitude(pressure, temperature);
//    static char buffer[100];
//    size_t size = sprintf(buffer, "MS5607:\r\n\tPressure: %.2f mBar\r\n\tTemperature: %.2f C\r\n\tAltitude: %f\r\n",
//                          pressure, temperature, altitude);
    // CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}

RetType shtc3Task(void *) {
    RESUME();
    static SHTC3_DATA_STRUCT(shtc3_data);

    RetType ret = CALL(shtc3->getHumidityAndTemp(&shtc3_data.temperature, &shtc3_data.humidity));
    if (ret == RET_ERROR) {
        // CALL(uartDev->write((uint8_t *) "SHT: Error\r\n", 12));
        RESET();
        return ret;
    }

//    static char buffer[150];
//    size_t size = snprintf(buffer, 100, "Humidity: %f\r\nTemperature: %f\r\n", humidity, temp);
    // CALL(uartDev->write((uint8_t *) buffer, size));

    RESET();
    return RET_SUCCESS;
}



RetType sensorInitTask(void *) {
    RESUME();

#ifdef DEBUG
    // CALL(uartDev->write((uint8_t *) "LED: Initializing\r\n", 19));
    RetType ret = CALL(ledOne->init());
    ret = CALL(ledTwo->init());
    static tid_t ledTID = -1;
    if (ret != RET_ERROR) {
        ledTID = sched_start(startupLEDTask, {});

        if (-1 == ledTID) {
            // CALL(uartDev->write((uint8_t *) "LED: Task Init Failed\r\n", 23));
        } else {
            // CALL(uartDev->write((uint8_t *) "LED: Initialized\r\n", 18));
        }
    }
#endif

    // CALL(uartDev->write((uint8_t *) "TMP117: Initializing\r\n", 23));
    static TMP117 tmp(*i2cDev);
    tmp117 = &tmp;
    tid_t tmpTID = -1;
    RetType tmp3Ret = CALL(tmp117->init());
    if (tmp3Ret != RET_ERROR) {
        tmpTID = sched_start(tmpTask, {});

        if (-1 == tmpTID) {
            // CALL(uartDev->write((uint8_t *) "TMP117: Task Init Failed\r\n", 27));
        } else {
            // CALL(uartDev->write((uint8_t *) "TMP117: Initialized\r\n", 22));
        }
    } else {
        // CALL(uartDev->write((uint8_t *) "TMP117: Sensor Init Failed\r\n", 29));
    }

    // CALL(uartDev->write((uint8_t *) "LSM6DSL: Initializing\r\n", 23));
    static LSM6DSL lsm(*i2cDev);
    lsm6dsl = &lsm;
    tid_t lsmTID = -1;
    RetType lsm6dslRet = CALL(lsm6dsl->init(LSM6DSL_I2C_ADDR_SECONDARY));
    if (lsm6dslRet != RET_ERROR) {
        lsmTID = sched_start(lsmTask, {});

        if (-1 == lsmTID) {
            // CALL(uartDev->write((uint8_t *) "LSM6DSL: Task Init Failed\r\n", 27));
        } else {
            // CALL(uartDev->write((uint8_t *) "LSM6DSL: Initialized\r\n", 22));
        }
    } else {
        // CALL(uartDev->write((uint8_t *) "LSM6DSL: Sensor Init Failed\r\n", 29));
    }

//    // CALL(uartDev->write((uint8_t *) "MS5607: Initializing\r\n", 22));
//    static MS5607 ms5(*i2cDev);
//    ms5607 = &ms5;
//    tid_t ms5TID = -1;
//    RetType ms5Ret = CALL(ms5607->init());
//    if (ms5Ret != RET_ERROR) {
//        ms5TID = sched_start(ms5607Task, {});
//
//        if (-1 == ms5TID) {
//            // CALL(uartDev->write((uint8_t *) "MS5607: Task Init Failed\r\n", 26));
//        } else {
//            // CALL(uartDev->write((uint8_t *) "MS5607: Initialized\r\n", 21));
//        }
//    } else {
//        // CALL(uartDev->write((uint8_t *) "MS5607: Sensor Init Failed\r\n", 28));
//    }

    // CALL(uartDev->write((uint8_t *) "ADXL375: Initializing\r\n", 23));
    static ADXL375 adxl(*i2cDev);
    adxl375 = &adxl;
    tid_t adxl375TID = -1;
    RetType adxl375Ret = CALL(adxl375->init(0x1D));
    if (adxl375Ret != RET_ERROR) {
        adxl375TID = sched_start(adxlTask, {});

        if (-1 == adxl375TID) {
            // CALL(uartDev->write((uint8_t *) "ADXL375: Task Init Failed\r\n", 27));
        } else {
            // CALL(uartDev->write((uint8_t *) "ADXL375: Initialized\r\n", 22));
        }
    } else {
        // CALL(uartDev->write((uint8_t *) "ADXL375: Sensor Init Failed\r\n", 29));
    }

    // CALL(uartDev->write((uint8_t *) "LIS3MDL: Initializing\r\n", 23));
    static LIS3MDL lis(*i2cDev);
    lis3mdl = &lis;
    tid_t lisTID = -1;
    RetType lis3mdlRet = CALL(lis3mdl->init(LIS3MDL_I2C_ADDR_PRIMARY));
    if (lis3mdlRet != RET_ERROR) {
        lisTID = sched_start(lisTask, {});

        if (-1 == lisTID) {
            // CALL(uartDev->write((uint8_t *) "LIS3MDL: Task Init Failed\r\n", 27));
        } else {
            // CALL(uartDev->write((uint8_t *) "LIS3MDL: Initialized\r\n", 22));
        }
    } else {
        // CALL(uartDev->write((uint8_t *) "LIS3MDL: Sensor Init Failed\r\n", 29));
    }

    // CALL(uartDev->write((uint8_t *) "BMP388: Initializing\r\n", 22));
    static BMP3XX bmp(*i2cDev);
    bmp3XX = &bmp;
    tid_t bmpTID = -1;
    RetType bmp3Ret = CALL(bmp3XX->init());
    if (bmp3Ret != RET_ERROR) {
        bmpTID = sched_start(bmpTask, {});

        if (-1 == bmpTID) {
            // CALL(uartDev->write((uint8_t *) "BMP388: Task Init Failed\r\n", 26));
        } else {
            // CALL(uartDev->write((uint8_t *) "BMP388: Initialized\r\n", 21));
        }
    } else {
        // CALL(uartDev->write((uint8_t *) "BMP388: Sensor Init Failed\r\n", 28));
    }

    // CALL(uartDev->write((uint8_t *) "SHTC3: Initializing\r\n", 19));
    static SHTC3 sht(
            *i2cDev);
    shtc3 = &sht;
    tid_t shtTID = -1;
    RetType sht3mdlRet = CALL(shtc3->init(0x70));
    if (sht3mdlRet != RET_ERROR) {
        shtTID = sched_start(shtc3Task, {});

        if (-1 == shtTID) {
            // CALL(uartDev->write((uint8_t *) "SHT: Task Init Failed\r\n", 23));
        } else {
            // CALL(uartDev->write((uint8_t *) "SHT: Initialized\r\n", 19));
        }
    } else {
        // CALL(uartDev->write((uint8_t *) "SHT: Sensor Init Failed\r\n", 25));
    }

#ifdef DEBUG
    sched_block(ledTID);

    CALL(ledOne->setState(LED_OFF)); // Keep it off or on? ON can signal it is powered
    CALL(ledTwo->setState(LED_ON));
#endif

    RESET();
    return RET_ERROR;
}

RetType netStackInitTask(void *) {
    RESUME();

    static tid_t ledToggleTID = sched_start(wizLEDToggleTask, {});

    static W5500 wiznet(*wizSPI, *wizCS);
    w5500 = &wiznet;

    static IPv4UDPStack iPv4UdpStack{10, 10, 10, 1, \
                              255, 255, 255, 0,
                              *w5500};
    stack = &iPv4UdpStack;

    static uint8_t ip_addr[4] = {192, 168, 1, 10};
    static uint8_t subnet_mask[4] = {255, 255, 255, 0};
    static uint8_t gateway_addr[4] = {192, 168, 1, 1};
    static uint8_t mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static IPv4UDPSocket::addr_t addr;

    sock = stack->get_socket();
    addr.ip[0] = addr.ip[1] = addr.ip[2] = addr.ip[3] = 0;
    addr.port = 8000;
    sock->bind(addr); // TODO: Error handling

    ipv4::IPv4Addr_t temp_addr;
    ipv4::IPv4Address(10, 10, 10, 69, &temp_addr);
    stack->add_multicast(temp_addr);

    // CALL(uartDev->write((uint8_t *) "W5500: Initializing\r\n", 23));
    RetType ret = CALL(wiznet.init(gateway_addr, subnet_mask, mac_addr, ip_addr));
    if (ret != RET_SUCCESS) {
        // CALL(uartDev->write((uint8_t *) "W5500: Failed to initialize\r\n", 29));
        goto netStackInitDone;
    }

    // CALL(uartDev->write((uint8_t *) "W5500: Initialized\r\n", 20));

    if (RET_SUCCESS != stack->init()) {
        // CALL(uartDev->write((uint8_t *) "Failed to initialize network stack\r\n", 35));
        goto netStackInitDone;
    }

    sched_block(ledToggleTID);
    CALL(wizLED->setState(LED_OFF));


    netStackInitDone:
    RESET();
    return RET_ERROR; // Kill task
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
        HAL_UART_Transmit_IT(&huart5, (uint8_t *) "Failed to init UART Device. Exiting.\n\r", 38);
        return -1;
    }
    uartDev = &uart;
    HAL_UART_Transmit(&huart5, (uint8_t *) "UART Device Initialized\n\r", 25, 1000);

    if (!sched_init(&HAL_GetTick)) {
        HAL_UART_Transmit_IT(&huart5, (uint8_t *) "Failed to init scheduler\n\r", 30);
        return -1;
    }

    // Initialize peripherals
    HALGPIODevice ledOneGPIO("LED 1 GPIO", PA1_LED_GPIO_Port, PA1_LED_Pin);
    ret = ledOneGPIO.init();
    LED ledOneLocal(ledOneGPIO);
    ledOneLocal.setState(LED_OFF);
    ledOne = &ledOneLocal;

    HALGPIODevice ledTwoGPIO("LED 2 GPIO", PA2_LED_GPIO_Port, PA2_LED_Pin);
    ret = ledTwoGPIO.init();
    LED ledTwoLocal(ledTwoGPIO);
    ledOneLocal.setState(LED_OFF);
    ledTwo = &ledTwoLocal;

    HALGPIODevice wiznetLEDGPIO("Wiznet LED GPIO", Wiz_LED_GPIO_Port, Wiz_LED_Pin);
    ret = wiznetLEDGPIO.init();
    LED wiznetLED(wiznetLEDGPIO);
    wiznetLED.setState(LED_ON);
    wizLED = &wiznetLED;

    HALGPIODevice wizChipSelect("Wiznet CS", ETH_CS_GPIO_Port, ETH_CS_Pin);
    ret = wizChipSelect.init();
    wizCS = &wizChipSelect;
    wizChipSelect.set(1);

    static HALI2CDevice i2c("HAL I2C3", &hi2c3);
    if (i2c.init() != RET_SUCCESS) {
        HAL_UART_Transmit_IT(&huart5, (uint8_t *) "Failed to init I2C1 Device. Exiting.\n\r", 38);
        return -1;
    }

    static HALSPIDevice wizSpi("WIZNET SPI", &hspi1);
    ret = wizSpi.init();
    wizSPI = &wizSpi;

    i2cDev = &i2c;
    sched_start(i2cDevPollTask, {});
    sched_start(spiDevPollTask, {});
    sched_start(netStackInitTask, {});
    sched_start(sensorInitTask, {});

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1) {
        sched_dispatch();

#ifdef DEBUG
//        HAL_Delay(3);
#endif
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

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
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WS25_CS_Pin|ETH_CS_Pin|Wiz_LED_Pin|RS485_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PA1_LED_Pin|PA2_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADDR3raw_Pin ADDR2raw_Pin ADDR1raw_Pin MCU_INT_Pin
                           BMP_INT_Pin P_IO_1_Pin */
  GPIO_InitStruct.Pin = ADDR3raw_Pin|ADDR2raw_Pin|ADDR1raw_Pin|MCU_INT_Pin
                          |BMP_INT_Pin|P_IO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ADDR0raw_Pin */
  GPIO_InitStruct.Pin = ADDR0raw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADDR0raw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WS25_CS_Pin ETH_CS_Pin Wiz_LED_Pin RS485_MODE_Pin */
  GPIO_InitStruct.Pin = WS25_CS_Pin|ETH_CS_Pin|Wiz_LED_Pin|RS485_MODE_Pin;
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

  /*Configure GPIO pins : W5500_INT_Pin LIS_INT_Pin LIS_DRDY_Pin P_IO_2_Pin */
  GPIO_InitStruct.Pin = W5500_INT_Pin|LIS_INT_Pin|LIS_DRDY_Pin|P_IO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1_LED_Pin PA2_LED_Pin */
  GPIO_InitStruct.Pin = PA1_LED_Pin|PA2_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LSM6_INT1_Pin LSM6_INT2_DEN_Pin ADXL_INT2_Pin ADXL_INT1_Pin */
  GPIO_InitStruct.Pin = LSM6_INT1_Pin|LSM6_INT2_DEN_Pin|ADXL_INT2_Pin|ADXL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
  while (1)
  {
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