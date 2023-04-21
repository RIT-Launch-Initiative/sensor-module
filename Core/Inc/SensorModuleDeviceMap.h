/**
 * Device Map For Sensor Module
 *
 * @author Aaron Chan
 */
#ifndef SENSOR_MODULE_SENSORMODULEDEVICEMAP_H
#define SENSOR_MODULE_SENSORMODULEDEVICEMAP_H

#include "device/DeviceMap.h"
#include "sched/macros/macros.h"

#include "device/StreamDevice.h"
#include "device/peripherals/ADXL375/ADXL375.h"
#include "device/peripherals/BMP3XX/BMP3XX.h"
#include "device/peripherals/LIS3MDL/LIS3MDL.h"
#include "device/peripherals/LSM6DSL/LSM6DSL.h"
#include "device/peripherals/MS5607/MS5607.h"
#include "device/peripherals/SHTC3/SHTC3.h"
#include "device/peripherals/TMP117/TMP117.h"

static const size_t MAP_SIZE = 12;

class SensorModuleDeviceMap : public alloc::DeviceMap<MAP_SIZE> {
public:
    /// @brief constructor
    SensorModuleDeviceMap(I2CDevice &i2cDevice, SPIDevice &wiznetSPI, SPIDevice &flashSPI,
                          GPIODevice &wiznetCS, GPIODevice &flashCS, GPIODevice &ledGPIO, StreamDevice &uart)
                          : alloc::DeviceMap<MAP_SIZE>("Sensor Module Device Map"),
                            MS5607(i2cDevice), BMP3XX(i2cDevice), ADXL375(i2cDevice),
                            LSM6DSL(i2cDevice), LIS3MDL(i2cDevice), SHTC3(i2cDevice), TMP117(i2cDevice),
                            i2cDevice(i2cDevice),
                            wiznetSPI(wiznetSPI), wiznetCS(wiznetCS),
                            flashSPI(flashSPI), flashCS(flashCS),
                            ledGPIO(ledGPIO), debugUART(uart) {};

    /// @brief initialize the Linux platform specific map
    RetType init() {
        // RET_SUCCESS is a 0
        int ret = add("i2c", &i2cDevice);
        ret += add("wiznet_spi", &wiznetSPI);
        ret += add("wiznet_cs", &wiznetCS);
        ret += add("flash_spi", &flashSPI);
        ret += add("flash_cs", &flashCS);

        ret += add("led", &ledGPIO);

        ret += add("debug_uart", &debugUART);

        ret += add("ms5607", &MS5607);
        ret += add("bmp3xx", &BMP3XX);
        ret += add("adxl375", &ADXL375);
        ret += add("lsm6dsl", &LSM6DSL);
        ret += add("lis3mdl", &LIS3MDL);
        ret += add("shtc3", &SHTC3);
        ret += add("tmp117", &TMP117);

        // SUCCESS if ret == 0
        return static_cast<RetType>(ret);
    }

private:
    // Sensors
    MS5607 MS5607;
    BMP3XX BMP3XX;
    ADXL375 ADXL375;
    LSM6DSL LSM6DSL;
    LIS3MDL LIS3MDL;
    SHTC3 SHTC3;
    TMP117 TMP117;

    I2CDevice &i2cDevice;

    SPIDevice &wiznetSPI;
    GPIODevice &wiznetCS;

    SPIDevice &flashSPI;
    GPIODevice &flashCS;

    GPIODevice &ledGPIO;

    StreamDevice &debugUART;
};

#endif //SENSOR_MODULE_SENSORMODULEDEVICEMAP_H
