/**
 * Defines all tasks a sensor module is capable of performing
 *
 * @author Aaron Chan
 */

#ifndef SENSOR_MODULE_DEVICE_TASK_H
#define SENSOR_MODULE_DEVICE_TASK_H

#include "return.h"
#include "SensorModuleDeviceMap.h"

extern SensorModuleDeviceMap deviceMap;

/***********************************/
/*********** Init Tasks ***********/
/*********************************/

/***********************************/
/********** Debug Tasks ***********/
/*********************************/
#ifdef DEBUG

RetType print_heartbeat_task(void *args);

RetType flash_led_task(void *args);

RetType wiz_send_test_task(void *) {
    return RET_ERROR;
}

RetType wiz_recv_test_task(void *);

#endif
/***********************************/
/********** Sensor Tasks **********/
/*********************************/

/**
 * Task for the MS5607 Barometric Pressure Sensor
 * @param args
 * @return Scheduler Status
 */
RetType ms5607_task(void *args) {
    static MS5607 *ms5607 = (MS5607 *) deviceMap.get("ms5607");
    static MS5607_DATA_STRUCT(ms5607_data);

    RESUME();

    RetType ret = CALL(ms5607->getData(&ms5607_data));
    if (RET_SUCCESS != ret) {
#ifdef DEBUG
        swprint("MS5607 Data Read Error");
#endif
        // TODO: LOGGING
    } else {
        // TODO: Add to data queue
    }

    RESET();
    return RET_SUCCESS;
}

/**
 * Task for the BMP388 Barometric Pressure Sensor
 * @param args
 * @return Scheduler Status
 */
RetType bmp388_task(void *args) {
    static BMP3XX *bmp3xx = (BMP3XX *) deviceMap.get("bmp388");
    static BMP3XX_DATA_STRUCT(bmp3xx_data);

    RESUME();

    RetType ret = CALL(bmp3xx->getData(&bmp3xx_data));
    if (RET_SUCCESS != ret) {
#ifdef DEBUG
        swprint("BMP388 Data Read Error");
#endif
        // TODO: LOGGING
    } else {
        // TODO: Add to data queue
    }

    RESET();
    return RET_SUCCESS;
}

/**
 * Task for the LSM6DSL IMU
 * @param args
 * @return Scheduler Status
 */
RetType lsm6dsl_task(void *args) {
    static LSM6DSL *lsm6dsl = (LSM6DSL *) deviceMap.get("lsm6dsl");
    static LSM6DSL_DATA_STRUCT(lsm6dsl_data);

    RESUME();

    RetType ret = CALL(lsm6dsl->getData(&lsm6dsl_data));
    if (RET_SUCCESS != ret) {
#ifdef DEBUG
        swprint("LSM6DSL Data Read Error");
#endif
    } else {

    }

    RESET();
    return RET_SUCCESS;
}

/**
 * Task for the ADXL375 Accelerometer
 * @param args
 * @return Scheduler Status
 */
RetType adxl375_task(void *args) {
    static ADXL375 *adxl375 = (ADXL375 *) deviceMap.get("adxl375");
    static ADXL375_DATA_STRUCT(adxl375_data);

    RESUME();

    RetType ret = CALL(adxl375->getData(&adxl375_data));
    if (RET_SUCCESS != ret) {
#ifdef DEBUG
        swprint("ADXL375 Data Read Error");
#endif
    } else {

    }

    RESET();
    return RET_SUCCESS;
}

/**
 * Task for the LIS3MDL Magnetometer
 * @param args
 * @return Scheduler Status
 */
RetType lis3mdl_task(void *args) {
    static LIS3MDL *lis3mdl = (LIS3MDL *) deviceMap.get("lis3mdl");
    static LIS3MDL_DATA_STRUCT(lis3mdl_data);

    RESUME();

    RetType ret = CALL(lis3mdl->getData(&lis3mdl_data));
    if (RET_SUCCESS != ret) {
#ifdef DEBUG
        swprint("LIS3MDL Data Read Error");
#endif
    } else {

    }

    RESET();
    return RET_SUCCESS;
}

/**
 * Task for the TMP117 Temperature Sensor
 * @param args
 * @return Scheduler Status
 */
RetType tmp117_task(void *args) {
    static TMP117 *tmp117 = (TMP117 *) deviceMap.get("tmp117");
    static TMP117_DATA_STRUCT(tmp117_data);

    RESUME();

    RetType ret = CALL(tmp117->getData(&tmp117_data));
    if (RET_SUCCESS != ret) {
#ifdef DEBUG
        swprint("TMP117 Data Read Error");
#endif
    } else {

    }

    RESET();
    return RET_SUCCESS;
}

/**
 * Task for the SHTC3 Temperature/Humidity Sensor
 * @param args
 * @return Scheduler Status
 */
RetType shtc3_task(void *args) {
    static SHTC3 *shtc3 = (SHTC3 *) deviceMap.get("shtc3");
    static SHTC3_DATA_STRUCT(shtc3_data);

    RESUME();

    RetType ret = CALL(shtc3->getData(&shtc3_data));
    if (RET_SUCCESS != ret) {
#ifdef DEBUG
        swprint("SHTC3 Data Read Error");
#endif
    } else {

    }

    RESET();
    return RET_SUCCESS;
}

/***********************************/
/********** Network Tasks *********/
/*********************************/


/***********************************/
/********** Polling Tasks *********/
/*********************************/


#endif //SENSOR_MODULE_DEVICE_TASK_H
