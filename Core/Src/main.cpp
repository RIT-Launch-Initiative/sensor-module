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
//#include "device/peripherals/W5500/W5500.h"
//#include "device/peripherals/wiznet/wiznet.h"
//#include "net/packet/Packet.h"
//#include "net/stack/IPv4UDP/IPv4UDPStack.h"
//#include "net/stack/IPv4UDP/IPv4UDPSocket.h"

//#include "sched/macros/call.h"


// #include "filesystem/ChainFS/ChainFS.h" // TODO: Unfinished
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    LED **led;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_SPI1_Init(void);

static void MX_SPI2_Init(void);

static void MX_I2C3_Init(void);

static void MX_UART5_Init(void);

void clearI2CBusyFlag(I2C_HandleTypeDef *hi2c);


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
void swprint(const char *msg) {
#ifdef DEBUG
    int len = strlen(msg);
    for (int i = 0; i < len; i++) {
        ITM_SendChar(msg[i]);
    }
#endif
}

// send stuff down serial wire out if DEBUG flag set
// Do not send a message larger than 256 bytes
int swprintf(const char *fmt, ...) {
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

RetType print_heartbeat_task(void *) {
    RESUME();
    static int i = 0;
    swprintf("Ping %d\n", i++);
    SLEEP(1000);
    RESET();
    return RET_SUCCESS;
}

RetType flash_led_task(void *params) {
    RESUME();
    led_flash_t *arg = ((led_flash_t *) params);
    LED *task_led = *(arg->led);

    if (NULL != task_led) {
        if ((arg->on) && (arg->period - arg->on_time > 0)) {
            task_led->set_state(LED_OFF);
            arg->on = false;
            SLEEP(arg->period - arg->on_time);
        } else if (arg->on_time > 0) {
            task_led->set_state(LED_ON);
            arg->on = true;
            SLEEP(arg->on_time);
        }
    }
    RESET();
    return RET_SUCCESS;
}

RetType init_led_task(void *) {
    RESUME();

    CALL(ledOne->init());
    CALL(ledTwo->init());
    CALL(wizLED->init());

    RESET();
    return RET_ERROR;
}


RetType i2cDevPollTask(void *) {
    RESUME();
    CALL(i2cDev->poll());
//    swprint("I2C Poll\n");
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

RetType bmpTask(void *) {
    RESUME();

    static BMP3XX_DATA_STRUCT(bmp_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = bmp_data.id;

    RetType ret = CALL(bmp3XX->getPressureAndTemp(&bmp_data.pressure, &bmp_data.temperature));
    if (ret == RET_ERROR) {
//    	swprint("Failed to get BMP data\n");
        RESET();
        return RET_SUCCESS;
    }

//    swprintf("BMP\n\tP: %3.2f Pa\n\tT: %3.2f C\n", bmp_data.pressure, bmp_data.temperature);
//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&bmp_data), sizeof(bmp_data), &addr));

    RESET();
    return RET_SUCCESS;
}

RetType tmpTask(void *) {
    RESUME();

    static char buffer[100];
    static TMP117_DATA_STRUCT(tmp_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = tmp_data.id;

    RetType ret = CALL(tmp117->getData(&tmp_data));
    if (ret == RET_ERROR) {
        // CALL(uartDev->write((uint8_t *) "Failed to get TMP data\r\n", 9));
        RESET();
        return RET_SUCCESS;
    }

//    size_t size = sprintf(buffer, "TMP Temperature: %f C\r\n", data.temp);
    // CALL(uartDev->write((uint8_t *) buffer, size));
    swprintf("TMP117\n\tTemperature: %3.2f C\n", tmp_data.temperature);

//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&tmp_data), sizeof(tmp_data), &addr));

    RESET();
    return RET_SUCCESS;
}

RetType adxlTask(void *) {
    RESUME();
    static ADXL375_DATA_STRUCT(adxl_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = adxl_data.id;

    RetType ret = CALL(adxl375->getData(&adxl_data));
    if (ret != RET_SUCCESS) {
        // CALL(uartDev->write((uint8_t *) "Failed to get ADXL data\r\n", 24)
        RESET();
        return RET_SUCCESS;
    }

    static char buffer[100];
//    size_t size = snprintf(buffer, 100, "ADXL375: x: %d, y: %d, z: %d\r\n", x, y, z);
//
    // Use below if you want to print the values in multiple lines
    // size_t size = snprintf(buffer, 100, "ADXL375:\r\n\tX-Axis: %d m/s^2\r\n\tY-Axis: %d m/s^2\r\n\tZ-Axis: %d m/s^2\r\n", x, y, z);

    // CALL(uartDev->write((uint8_t *) buffer, size));
    swprintf("ADXL375\n\tX-Axis: %d m/s^2\n\tY-Axis: %d m/s^2\n\tZ-Axis: %d m/s^2\n", adxl_data.x_accel,
             adxl_data.y_accel, adxl_data.z_accel);
//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&adxl_data), sizeof(adxl_data), &addr));

    RESET();
    return RET_SUCCESS;
}

RetType lsmTask(void *) {
    RESUME();
    static LSM6DSL_DATA_STRUCT(lsm_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = lsm_data.id;

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

//    swprintf("LSM6DSL\n\tAccel:\n\t\tX: %3.2f m/s^2\n\t\tY: %3.2f m/s^2\n\t\tZ: %3.2f m/s^2\n",
//            lsm_data.x_accel, lsm_data.y_accel, lsm_data.z_accel);
//    swprintf("\tGyro:\n\t\tX: %3.2f dps\n\t\tY: %3.2f dps\n\t\tZ: %3.2f dps\n",
//            lsm_data.x_gyro, lsm_data.y_gyro, lsm_data.z_gyro);
//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&lsm_data), sizeof(lsm_data), &addr));

    RESET();
    return RET_SUCCESS;
}

RetType lisTask(void *) {
    RESUME();
    static LIS3MDL_DATA_STRUCT(lis_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = lis_data.id;

    RetType ret = CALL(
            lis3mdl->pullSensorData(&lis_data.x_mag, &lis_data.y_mag, &lis_data.z_mag, &lis_data.temperature));
    if (ret != RET_SUCCESS) {
        // CALL(uartDev->write((uint8_t *) "LIS3MDL: Failed to get sensor data\r\n", 35));
        RESET();
        return RET_SUCCESS;
    }

//    swprintf("LIS3MDL\n\tX: %3.2f gauss\n\tY: %3.2f gauss\n\tZ: %3.2f gauss\n\tTemp: %3.2f C\n", lis_data.x_mag,
//             lis_data.y_mag, lis_data.z_mag, lis_data.temperature);

//    static char buffer[100];
//    size_t size = snprintf(buffer, 100, "Mag: \r\n\tX: %f\r\n\tY: %f\r\n\tZ: %f\r\nTemp: %f\r\n", magX, magY, magZ,
//                           temp);
    // CALL(uartDev->write((uint8_t *) buffer, size));

//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&lis_data), sizeof(lis_data), &addr));


    RESET();
    return RET_SUCCESS;
}

RetType ms5607Task(void *) {
    RESUME();

    static MS5607_DATA_STRUCT(ms5607_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = ms5607_data.id;

    RetType ret = CALL(ms5607->getData(&ms5607_data));
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

    swprintf("MS5607:\n\tPressure: %.2f mBar\n\tTemperature: %.2f C\n", ms5607_data.pressure, ms5607_data.temperature);
//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&ms5607_data), sizeof(ms5607_data), &addr));


    RESET();
    return RET_SUCCESS;
}

RetType shtc3Task(void *) {
    RESUME();
    static SHTC3_DATA_STRUCT(shtc3_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = shtc3_data.id;

    RetType ret = CALL(shtc3->getHumidityAndTemp(&shtc3_data.temperature, &shtc3_data.humidity));
    if (RET_ERROR == ret) {
//    	swprint("#RED#SHTC3: Data read fail\n");
        goto shtc3_end;
    }
//    swprintf("SHTC3:\n\t T = %3.2f, RH = %3.2f\n", shtc3_data.temperature, shtc3_data.humidity);

//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&shtc3_data), sizeof(shtc3_data), &addr));
//    if (RET_ERROR == ret) {
//    	swprint("#RED#SHTC3: Socket send fail\n");
//    	goto shtc3_end;
//    }

    shtc3_end:
RESET();
    return RET_SUCCESS;
}

static void check_i2c_ids() {
    for (uint8_t i = 0; i < 128; i++) {
        if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c3, i << 1, 10, 100)) {
            swprintf("Found I2C device at 0x%02X\r\n", i);
        }
    }

    swprint("Done checking I2C");

    HAL_Delay(1000);
}

static led_flash_t led1_flash = {.led = &ledOne, .on_time = 100, .period = 250};
static led_flash_t led2_flash = {.led = &ledTwo, .on_time = 100, .period = 250};
static led_flash_t wiz_flash = {.led = &wizLED, .on_time = 100, .period = 250};

RetType sensorInitTask(void *) {
    RESUME();

//    tid_t flash1 = sched_start(flash_led_task, &led1_flash);
//    tid_t flash2 = sched_start(flash_led_task, &led2_flash);
//    tid_t hbeat = sched_start(print_heartbeat_task, {});

    swprint("Initializing TMP117\n");
    static TMP117 tmp(*i2cDev);
    tmp117 = &tmp;
    tid_t tmpTID = -1;
    RetType tmp3Ret = CALL(tmp117->init());
    if (tmp3Ret != RET_ERROR) {
        tmpTID = sched_start(tmpTask, {});

        if (-1 == tmpTID) {
            swprint("#RED#TMP117 task start failed\n");
        } else {
            swprint("#GRN#TMP117 task start OK\n");
        }
    } else {
        swprint("#RED#TMP117 init failed\n");
    }

    swprint("Initializing LSM6DSL\n");
    static LSM6DSL lsm(*i2cDev, LSM6DSL_I2C_ADDR_SECONDARY);
    lsm6dsl = &lsm;
    tid_t lsmTID = -1;
    RetType lsm6dslRet = CALL(lsm6dsl->init());
    if (lsm6dslRet != RET_ERROR) {
//        lsmTID = sched_start(lsmTask, {}); // TODO: Causes no other I2C tasks to run

        if (-1 == lsmTID) {
            swprint("#RED#LSM6DSL task start failed\n");
        } else {
            swprint("#GRN#LSM6DSL task start OK\n");
        }
    } else {
        swprint("#RED#LSM6DSL init failed\n");
    }

    swprint("Initializing MS5607\n");
    static MS5607 ms5(*i2cDev);
    ms5607 = &ms5;
    tid_t ms5TID = -1;
    RetType ms5Ret = CALL(ms5607->init());
    if (ms5Ret != RET_ERROR) {
        ms5TID = sched_start(ms5607Task, {}); // TODO: Doesn't print data?

        if (-1 == ms5TID) {
            swprint("#RED#MS5607 task start failed\n");
        } else {
            swprint("#GRN#MS5607 task start OK\n");
        }
    } else {
        swprint("#RED#MS5607 init failed\n");
    }

    swprint("Initializing ADXL375\n");
    static ADXL375 adxl(*i2cDev, 0x1D);
    adxl375 = &adxl;
    tid_t adxl375TID = -1;
    RetType adxl375Ret = CALL(adxl375->init());
    if (adxl375Ret != RET_ERROR) {
        adxl375TID = sched_start(adxlTask, {});

        if (-1 == adxl375TID) {
            swprint("#RED#ADXL375 task start failed\n");
        } else {
            swprint("#GRN#ADXL375 task start OK\n");
        }
    } else {
        swprint("#RED#ADXL375 init \n");
    }

    swprint("Initializing LIS3MDL\n");
    static LIS3MDL lis(*i2cDev, LIS3MDL_I2C_ADDR_PRIMARY);
    lis3mdl = &lis;
    tid_t lisTID = -1;
    RetType lis3mdlRet = CALL(lis3mdl->init());
    if (lis3mdlRet != RET_ERROR) {
        lisTID = sched_start(lisTask, {});

        if (-1 == lisTID) {
            swprint("#RED#LIS3MDL task start failed\n");
        } else {
            swprint("#GRN#LIS3MDL task start OK\n");
        }
    } else {
        swprint("#RED#LIS3MDL init failed\n");
    }

    swprint("Initializing SHTC3\n");
    static SHTC3 sht(*i2cDev, 0x70);
    shtc3 = &sht;
    tid_t shtTID = -1;
    RetType sht3mdlRet = CALL(shtc3->init());
    if (sht3mdlRet != RET_ERROR) {
//        shtTID = sched_start(shtc3Task, {}); // TODO: Causes no other I2C tasks to run

        if (-1 == shtTID) {
            swprint("#RED#SHTC3 task start failed\n");
        } else {
            swprint("#GRN#SHTC3 task start OK\n");
        }
    } else {
        swprint("#RED#SHTC3 init failed\n");
    }

    led1_flash.period = 1000;
    led2_flash.period = 1000;

    swprint("Finished initializing sensors\n");

    RESET();
    return RET_ERROR;
}

//RetType wizRecvTestTask(void *) {
//    RESUME();
//    static Packet packet = alloc::Packet<IPv4UDPSocket::MTU_NO_HEADERS - IPv4UDPSocket::HEADERS_SIZE, IPv4UDPSocket::HEADERS_SIZE>();
//    static uint8_t *buff;
//
//    RetType ret = CALL(w5500->recv_data(stack->get_eth(), packet));
//    buff = packet.raw();
//
//    RESET();
//    return RET_SUCCESS;
//}
//
//RetType wizSendTestTask(void *) {
//    RESUME();
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 1;
//    addr.port = 8000;
//
//    static uint8_t buff[7] = {'L', 'a', 'u', 'n', 'c', 'h', '!'};
//    RetType ret = CALL(sock->send(buff, 7, &addr));
//
//    RESET();
//    return RET_SUCCESS;
//}

//RetType netStackInitTask(void *) {
//    RESUME();
//
//	sched_start(flash_led_task, &wiz_flash);
//
//    static W5500 wiznet(*wizSPI, *wizCS);
//    w5500 = &wiznet;
//
//    static IPv4UDPStack iPv4UdpStack{10, 10, 10, 1, \
//                              255, 255, 255, 0,
//                                     *w5500};
//    stack = &iPv4UdpStack;
//
//    static uint8_t ip_addr[4] = {192, 168, 1, 10};
//    static uint8_t subnet_mask[4] = {255, 255, 255, 0};
//    static uint8_t gateway_addr[4] = {192, 168, 1, 1};
//    static uint8_t mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//    static IPv4UDPSocket::addr_t addr;
//
//    sock = stack->get_socket();
//    addr.ip[0] = addr.ip[1] = addr.ip[2] = addr.ip[3] = 0;
//    addr.port = 8000;
//    sock->bind(addr); // TODO: Error handling
//
//    ipv4::IPv4Addr_t temp_addr;
//    ipv4::IPv4Address(10, 10, 10, 69, &temp_addr);
//    stack->add_multicast(temp_addr);
//
//
//    swprint("Initializing W5500\n");
//    RetType ret = CALL(wiznet.init(gateway_addr, subnet_mask, mac_addr, ip_addr));
//    if (RET_SUCCESS != ret) {
//		swprint("#RED#W5500 init failed\n");
//        goto netStackInitDone;
//    } else {
//		swprint("#GRN#W5500 init OK\n");
//    }
//
//
//    swprint("Initializing network stack\n");
//    ret = stack->init();
//    if (RET_SUCCESS != ret) {
//    	swprint("#RED#Net stack init failed");
//        goto netStackInitDone;
//    } else {
//		swprint("#GRN#Net stack init OK\n");
//    }
//
//    swprint("Successfully initialized network interface\n");
//
//    netStackInitDone:
//    wiz_flash.period = 1000;
//    RESET();
//    return RET_ERROR; // Kill task
//}

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
    check_i2c_ids();

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

    sched_start(i2cDevPollTask, {});
    sched_start(spiDevPollTask, {});
//    sched_start(netStackInitTask, {});
    sched_start(sensorInitTask, {});

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

void clearI2CBusyFlag(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct;
    int timeout = 100;
    int timeout_cnt = 0;

    // Clear PE bit.
    hi2c->Instance->CR1 &= ~(0x0001);
    uint16_t I2C3_SCL_PIN = GPIO_PIN_8;
    uint16_t I2C3_SDA_PIN = GPIO_PIN_9;
    GPIO_TypeDef *I2C3_SCL_PORT = GPIOA;
    GPIO_TypeDef *I2C3_SDA_PORT = GPIOC;

    //  Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = I2C3_SCL_PIN;
    HAL_GPIO_Init(I2C3_SCL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C3_SCL_PORT, I2C3_SCL_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = I2C3_SDA_PIN;
    HAL_GPIO_Init(I2C3_SDA_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C3_SDA_PORT, I2C3_SDA_PIN, GPIO_PIN_SET);

    // Check SCL and SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C3_SCL_PORT, I2C3_SCL_PIN)) {
        timeout_cnt++;
        if (timeout_cnt > timeout) return;
    }

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C3_SDA_PORT, I2C3_SDA_PIN)) {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C3_SCL_PORT, I2C3_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C3_SCL_PORT, I2C3_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if (timeout_cnt > timeout) return;
    }

    // Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C3_SDA_PORT, I2C3_SDA_PIN, GPIO_PIN_RESET);

    // Check SDA Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C3_SDA_PORT, I2C3_SDA_PIN)) {
        timeout_cnt++;
        if (timeout_cnt > timeout) return;
    }

    // Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C3_SCL_PORT, I2C3_SCL_PIN, GPIO_PIN_RESET);

    // Check SCL Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C3_SCL_PORT, I2C3_SCL_PIN)) {
        timeout_cnt++;
        if (timeout_cnt > timeout) return;
    }

    // Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C3_SCL_PORT, I2C3_SCL_PIN, GPIO_PIN_SET);

    // Check SCL High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C3_SCL_PORT, I2C3_SCL_PIN)) {
        timeout_cnt++;
        if (timeout_cnt > timeout) return;
    }

    // Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C3_SDA_PORT, I2C3_SDA_PIN, GPIO_PIN_SET);

    // Check SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C3_SDA_PORT, I2C3_SDA_PIN)) {
        timeout_cnt++;
        if (timeout_cnt > timeout) return;
    }

    // Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;

    GPIO_InitStruct.Pin = I2C3_SCL_PIN;
    HAL_GPIO_Init(I2C3_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C3_SDA_PIN;
    HAL_GPIO_Init(I2C3_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(I2C3_SCL_PORT, I2C3_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C3_SDA_PORT, I2C3_SDA_PIN, GPIO_PIN_SET);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    hi2c->Instance->CR1 |= 0x8000;

    asm("nop");

    // Clear SWRST bit in I2Cx_CR1 register.
    hi2c->Instance->CR1 &= ~0x8000;

    asm("nop");

    // Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    hi2c->Instance->CR1 |= 0x0001;

    // Call initialization function.
    HAL_I2C_Init(hi2c);
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
