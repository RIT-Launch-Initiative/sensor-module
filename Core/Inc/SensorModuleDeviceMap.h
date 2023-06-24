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

static const size_t MAP_SIZE = 15;


class SensorModuleDeviceMap : public alloc::DeviceMap<MAP_SIZE> {
public:
    /// @brief get the singleton instance of the Sensor Module Device Map
    SensorModuleDeviceMap() {
        if (instance == nullptr) {
            instance = new SensorModuleDeviceMap(nullptr);
        }
    }

    /// @brief initialize the Sensor Module specific map
    RetType init() {
        // RET_SUCCESS is a 0
        int ret = add("i2c", &i2cDevice);
        ret += add("wiznet_spi", &wiznetSPI);
        ret += add("wiznet_cs", &wiznetCS);
        ret += add("flash_spi", &flashSPI);
        ret += add("flash_cs", &flashCS);

        ret += add("led", &ledGPIO);

        ret += add("debug_uart", &prompt);

        ret += add("ms5607", &ms5607);
        ret += add("bmp3xx", &bmp3xx);
        ret += add("adxl375", &adxl375);
        ret += add("lsm6dsl", &lsm6dsl);
        ret += add("lis3mdl", &lis3mdl);
        ret += add("shtc3", &shtc3);
        ret += add("tmp117", &tmp117);

        // SUCCESS if ret == 0
        return static_cast<RetType>(ret);
    }

private:
    /// @brief constructor
    SensorModuleDeviceMap(const char *name) : alloc::DeviceMap<MAP_SIZE>("Sensor Module Device Map") {};

    // Sensors
    MS5607 ms5607 = MS5607(i2cDevice);
    BMP3XX bmp3xx = BMP3XX(i2cDevice);
    ADXL375 adxl375 = ADXL375(i2cDevice);
    LSM6DSL lsm6dsl = LSM6DSL(i2cDevice);
    LIS3MDL lis3mdl = LIS3MDL(i2cDevice);
    SHTC3 shtc3 = SHTC3(i2cDevice);
    TMP117 tmp117 = TMP117(i2cDevice);

    HALI2CDevice &i2cDevice = HALI2CDevice("HAL I2C3", &i2c);

    HALSPIDevice &wiznetSPI = HALSPIDevice("HAL SPI1", &wiznetSPI);
    HALGPIODevice &wiznetCS = HALGPIODevice("HAL GPIOA", &wiznetCS, wiznetCSPort);

    HALSPIDevice &flashSPI = HALSPIDevice("HAL SPI2", &flashSPI);
    HALGPIODevice &flashCS = HALGPIODevice("HAL GPIOB", &flashCS, flashCSPort);

    HALGPIODevice &ledGPIO = HALGPIODevice("HAL GPIOC", &ledGPIO, ledGPIOPort);

    HALUARTDevice &prompt = HALUARTDevice("PROMPT", &uart);
};

#endif //SENSOR_MODULE_SENSORMODULEDEVICEMAP_H
