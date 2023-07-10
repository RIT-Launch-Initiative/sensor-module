/**
 * Device Map For Sensor Module
 *
 * @author Aaron Chan
 */
#ifndef SENSOR_MODULE_SENSORMODULEDEVICEMAP_H
#define SENSOR_MODULE_SENSORMODULEDEVICEMAP_H

#include "device/DeviceMap.h"
#include "sched/macros.h"

#include "device/platforms/stm32/HAL_I2CDevice.h"
#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/platforms/stm32/HAL_UARTDevice.h"
#include "device/platforms/stm32/HAL_GPIODevice.h"

#include "device/StreamDevice.h"
#include "device/peripherals/ADXL375/ADXL375.h"
#include "device/peripherals/BMP3XX/BMP3XX.h"
#include "device/peripherals/LIS3MDL/LIS3MDL.h"
#include "device/peripherals/LSM6DSL/LSM6DSL.h"
#include "device/peripherals/MS5607/MS5607.h"
#include "device/peripherals/SHTC3/SHTC3.h"
#include "device/peripherals/TMP117/TMP117.h"
#include "device/peripherals/wiznet/wiznet.h"
#include "device/peripherals/W25Q/W25Q.h"

static const size_t MAP_SIZE = 22;

extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart5;




class SensorModuleDeviceMap : public alloc::DeviceMap<MAP_SIZE> {
public:
    SensorModuleDeviceMap() : DeviceMap("Sensor Module") {}

    /// @brief initialize the Sensor Module specific map
    RetType init() {
        // RET_SUCCESS is a 0
        int ret = add("i2c", &i2cDevice);
        ret += add("prompt", &prompt);

//        ret += add("led_one", &ledOne);
//        ret += add("led_two", &ledTwo);

        ret += add("wiznet_spi", &wiznetSPI);
        ret += add("wiznet_cs", &wiznetCS);
        ret += add("wiznet_reset", &wiznetReset);
        ret += add("wiznet", &wiznet);

//        ret += add("flash_spi", &flashSPI);
//        ret += add("flash_cs", &flashCS);

        ret += add("adxl375", &adxl375);
        ret += add("bmp3xx", &bmp3xx);
        ret += add("lis3mdl", &lis3mdl);
        ret += add("lsm6dsl", &lsm6dsl);
        ret += add("ms5607", &ms5607);
        ret += add("shtc3", &shtc3);
        ret += add("tmp117", &tmp117);


        // SUCCESS if ret == 0
        return static_cast<RetType>(ret);
    }

private:
    HALI2CDevice i2cDevice = HALI2CDevice("hi2c3", &hi2c3);

    HALSPIDevice wiznetSPI = HALSPIDevice("WIZNET SPI", &hspi1);
    HALGPIODevice wiznetCS = HALGPIODevice("WIZNET CS", ETH_CS_GPIO_Port, ETH_CS_Pin);
    HALGPIODevice wiznetLEDGPIO = HALGPIODevice("WIZNET LED", Wiz_LED_GPIO_Port, Wiz_LED_Pin);
    HALGPIODevice wiznetReset = HALGPIODevice("WIZNET RESET", W5500_RST_GPIO_Port, W5500_RST_Pin);
//    LED wiznetLED;

//    HALSPIDevice flashSPI;
//    HALGPIODevice flashCS;

    HALGPIODevice ledOneGPIO = HALGPIODevice("LED ONE", PA1_LED_GPIO_Port, PA1_LED_Pin);
    LED ledOne = LED(ledOneGPIO);

    HALGPIODevice ledTwoGPIO = HALGPIODevice("LED TWO", PA2_LED_GPIO_Port, PA2_LED_Pin);
    LED ledTwo = LED(ledTwoGPIO);

    HALUARTDevice prompt = HALUARTDevice("PROMPT", &huart5);

    // Sensors
    ADXL375 adxl375 = ADXL375(i2cDevice);
    BMP3XX bmp3xx = BMP3XX(i2cDevice);
    LIS3MDL lis3mdl = LIS3MDL(i2cDevice);
    LSM6DSL lsm6dsl = LSM6DSL(i2cDevice);
    MS5607 ms5607 = MS5607(i2cDevice);
    SHTC3 shtc3 = SHTC3(i2cDevice);
    TMP117 tmp117 = TMP117(i2cDevice);

    // SPI Devices
    Wiznet wiznet = Wiznet(wiznetSPI, wiznetCS, wiznetReset);
//    W25Q w25q;
};

#endif //SENSOR_MODULE_SENSORMODULEDEVICEMAP_H
