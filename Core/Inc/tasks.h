/**
 * Defines all tasks a sensor module is capable of performing
 *
 * @author Aaron Chan
 */

#ifndef SENSOR_MODULE_DEVICE_TASK_H
#define SENSOR_MODULE_DEVICE_TASK_H

#include "return.h"

/***********************************/
/*********** Init Tasks ***********/
/*********************************/
// TODO: Use the device map to initialize these eventually

RetType sensor_init(void* args);

RetType network_init(void* args);

/***********************************/
/********** Debug Tasks ***********/
/*********************************/
#ifdef DEBUG
RetType print_heartbeat_task(void* args);

RetType flash_led_task(void* args);

RetType wiz_send_test_task(void *);

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
RetType ms5607_task(void* args);

/**
 * Task for the BMP388 Barometric Pressure Sensor
 * @param args
 * @return Scheduler Status
 */
RetType bmp388_task(void *args);

/**
 * Task for the LSM6DSL IMU
 * @param args
 * @return Scheduler Status
 */
RetType lsm6dsl_task(void* args);

/**
 * Task for the ADXL375 Accelerometer
 * @param args
 * @return Scheduler Status
 */
RetType adxl375_task(void* args);

/**
 * Task for the LIS3MDL Magnetometer
 * @param args
 * @return Scheduler Status
 */
RetType lis3mdl_task(void* args);

/**
 * Task for the TMP117 Temperature Sensor
 * @param args
 * @return Scheduler Status
 */
RetType tmp117_task(void* args);

/**
 * Task for the SHTC3 Temperature/Humidity Sensor
 * @param args
 * @return Scheduler Status
 */
RetType shtc3_task(void* args);

/***********************************/
/********** Network Tasks *********/
/*********************************/


/***********************************/
/********** Polling Tasks *********/
/*********************************/


#endif //SENSOR_MODULE_DEVICE_TASK_H
