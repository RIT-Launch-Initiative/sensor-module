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

static const size_t MAP_SIZE = 15;


class SensorModuleDeviceMap : public alloc::DeviceMap<MAP_SIZE> {
public:
    SensorModuleDeviceMap(I2C_HandleTypeDef &i2c, UART_HandleTypeDef &promptUART, SPI_HandleTypeDef &hspi1,
                          SPI_HandleTypeDef &hspi2, GPIO_TypeDef *csPortOne, uint16_t csPinOne, GPIO_TypeDef *csPortTwo,
                          uint16_t csPinTwo, GPIO_TypeDef *ledPortOne, uint16_t &ledPinOne, GPIO_TypeDef *ledPortTwo,
                          uint16_t ledPinTwo, GPIO_TypeDef *wizLEDPort, uint16_t wizLEDPin, GPIO_TypeDef *wizResetPort, uint16_t wizResetPin) : DeviceMap("Sensor Module") {
        i2cDevice = HALI2CDevice("HAL I2C3", &i2c);


        wiznetSPI = HALSPIDevice("WIZNET SPI", &hspi1);
        wiznetCS = HALGPIODevice("WIZNET CS", csPortOne, csPinOne);
        wiznetReset = HALGPIODevice("WIZNET RESET", wizResetPort, wizResetPin);
        wiznetLEDGPIO = HALGPIODevice("WIZNET LED", wizLEDPort, wizLEDPin);

        flashSPI = HALSPIDevice("FLASH SPI", &hspi2);
        flashCS = HALGPIODevice("FLASH CS", csPortTwo, csPinTwo);

        ledOneGPIO = HALGPIODevice("LED ONE", ledPortOne, ledPinOne);
        ledTwoGPIO = HALGPIODevice("LED ONE", ledPortTwo, ledPinTwo);

        prompt = HALUARTDevice("PROMPT", &promptUART);

        adxl375 = ADXL375(i2cDevice);
        bmp3xx = BMP3XX(i2cDevice);
        lis3mdl = LIS3MDL(i2cDevice);
        lsm6dsl = LSM6DSL(i2cDevice);
        ms5607 = MS5607(i2cDevice);
        shtc3 = SHTC3(i2cDevice);
        tmp117 = TMP117(i2cDevice);

        wiznet = Wiznet(wiznetSPI, wiznetCS, wiznetReset);
//        w25q = W25Q("W25Q", flashSPI, flashCS, 0)
    }

    /// @brief initialize the Sensor Module specific map
    RetType init() {
        // RET_SUCCESS is a 0
        int ret = add("i2c", &i2cDevice);
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

        ret += add("prompt", &prompt);

        // SUCCESS if ret == 0
        return static_cast<RetType>(ret);
    }

private:
    // Sensors
    MS5607 ms5607;
    BMP3XX bmp3xx;
    ADXL375 adxl375;
    LSM6DSL lsm6dsl;
    LIS3MDL lis3mdl;
    SHTC3 shtc3;
    TMP117 tmp117;

    // SPI Devices
    Wiznet wiznet;
//    W25Q w25q;

    HALI2CDevice i2cDevice;

    HALSPIDevice wiznetSPI;
    HALGPIODevice wiznetCS;
    HALGPIODevice wiznetLEDGPIO;
    HALGPIODevice wiznetReset;
//    LED wiznetLED;

    HALSPIDevice flashSPI;
    HALGPIODevice flashCS;

    HALGPIODevice ledOneGPIO;
//    LED ledOne;

    HALGPIODevice ledTwoGPIO;
//    LED ledTwo;

    HALUARTDevice prompt;
};

#endif //SENSOR_MODULE_SENSORMODULEDEVICEMAP_H
