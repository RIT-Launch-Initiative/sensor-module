#include "tasks.h"
#include "SensorModuleDeviceMap.h"

#ifdef DEBUG
#include "device/peripherals/LED/LED.h"

RetType print_heartbeat_task(void*) {
	RESUME();
	static int i = 0;
	swprintf("Ping %d\n", i++);
	SLEEP(1000);
	RESET();
	return RET_SUCCESS;
}

RetType flash_led_task(void* args) {
	RESUME();
	led_flash_t* arg = ((led_flash_t*) params);
	LED* task_led = *(arg->led);

	if (NULL != task_led) {
		if ((arg -> on) && (arg->period - arg->on_time > 0)) {
			task_led->setState(LED_OFF);
			arg->on = false;
			SLEEP(arg->period - arg->on_time);
		} else if (arg->on_time > 0) {
			task_led->setState(LED_ON);
			arg->on = true;
			SLEEP(arg->on_time);
		}
	}
	RESET();
	return RET_SUCCESS;
}

#endif

RetType init_led_task(void*) {
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

RetType bmp_task(void *) {
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

    RetType ret = CALL(tmp117->readTempCelsius(&tmp_data.temperature));
    if (ret == RET_ERROR) {
        // CALL(uartDev->write((uint8_t *) "Failed to get TMP data\r\n", 9));
        RESET();
        return RET_SUCCESS;
    }

//    size_t size = sprintf(buffer, "TMP Temperature: %f C\r\n", data.temp);
    // CALL(uartDev->write((uint8_t *) buffer, size));
//    swprintf("TMP117\n\tTemperature: %3.2f C\n", tmp_data.temperature);

//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&tmp_data), sizeof(tmp_data), &addr));

    RESET();
    return RET_SUCCESS;
}

RetType adxl_task(void *) {
    RESUME();
    static ADXL375_DATA_STRUCT(adxl_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = adxl_data.id;

    RetType ret = CALL(adxl375->readXYZ(&adxl_data.x_accel, &adxl_data.y_accel, &adxl_data.z_accel));
    if (ret != RET_SUCCESS) {
//         CALL(uartDev->write((uint8_t *) "Failed to get ADXL data\r\n", 24)
        RESET();
        return RET_SUCCESS;
    }

    swprintf("ADXL375\n\tX-Axis: %d m/s^2\n\tY-Axis: %d m/s^2\n\tZ-Axis: %d m/s^2\n", adxl_data.x_accel, adxl_data.y_accel, adxl_data.z_accel);

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

    swprintf("LSM6DSL\n\tAccel:\n\t\tX: %3.2f m/s^2\n\t\tY: %3.2f m/s^2\n\t\tZ: %3.2f m/s^2\n",
            lsm_data.x_accel, lsm_data.y_accel, lsm_data.z_accel);
    swprintf("\tGyro:\n\t\tX: %3.2f dps\n\t\tY: %3.2f dps\n\t\tZ: %3.2f dps\n",
            lsm_data.x_gyro, lsm_data.y_gyro, lsm_data.z_gyro);
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

    swprintf("LIS3MDL\n\tX: %3.2f gauss\n\tY: %3.2f gauss\n\tZ: %3.2f gauss\n\tTemp: %3.2f C\n", lis_data.x_mag,
             lis_data.y_mag, lis_data.z_mag, lis_data.temperature);



    RESET();
    return RET_SUCCESS;
}

RetType ms5607_task(void *) {
    RESUME();

    static MS5607_DATA_STRUCT(ms5607_data);
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 10;
//    addr.port = ms5607_data.id;

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

    swprintf("MS5607:\n\tPressure: %.2f mBar\n\tTemperature: %.2f C\n\tAltitude: %f\n", ms5607_data.pressure,
             ms5607_data.temperature, ms5607->getAltitude(ms5607_data.pressure, ms5607_data.temperature));
//    ret = CALL(sock->send(reinterpret_cast<uint8_t *>(&ms5607_data), sizeof(ms5607_data), &addr));


    RESET();
    return RET_SUCCESS;
}

RetType shtc3_task(void *) {
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

static led_flash_t led1_flash = {.led = &ledOne, .on_time = 100, .period = 250};
static led_flash_t led2_flash = {.led = &ledTwo, .on_time = 100, .period = 250};
static led_flash_t wiz_flash = {.led = &wizLED, .on_time = 100, .period = 250};

RetType sensor_init_task(void *) {
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
    static ADXL375 adxl(*i2cDev);
    adxl375 = &adxl;
    tid_t adxl375TID = -1;
    RetType adxl375Ret = CALL(adxl375->init(0x1D));
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
    static LIS3MDL lis(*i2cDev);
    lis3mdl = &lis;
    tid_t lisTID = -1;
    RetType lis3mdlRet = CALL(lis3mdl->init(LIS3MDL_I2C_ADDR_PRIMARY));
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
    static SHTC3 sht(*i2cDev);
    shtc3 = &sht;
    tid_t shtTID = -1;
    RetType sht3mdlRet = CALL(shtc3->init(0x70));
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

    swprint("Initializing LSM6DSL\n");
    static LSM6DSL lsm(*i2cDev);
    lsm6dsl = &lsm;
    tid_t lsmTID = -1;
    RetType lsm6dslRet = CALL(lsm6dsl->init(LSM6DSL_I2C_ADDR_SECONDARY));
    if (lsm6dslRet != RET_ERROR) {
        lsmTID = sched_start(lsmTask, {}); // TODO: Causes no other I2C tasks to run

        if (-1 == lsmTID) {
			swprint("#RED#LSM6DSL task start failed\n");
        } else {
			swprint("#GRN#LSM6DSL task start OK\n");
        }
    } else {
		swprint("#RED#LSM6DSL init failed\n");
    }

    led1_flash.period = 1000;
    led2_flash.period = 1000;

    swprint("Finished initializing sensors\n");

    RESET();
    return RET_ERROR;
}

RetType wiz_recv_test_task(void *) {
    RESUME();
    static Packet packet = alloc::Packet<IPv4UDPSocket::MTU_NO_HEADERS - IPv4UDPSocket::HEADERS_SIZE, IPv4UDPSocket::HEADERS_SIZE>();
    static uint8_t *buff;

    RetType ret = CALL(w5500->recv_data(stack->get_eth(), packet));
    buff = packet.raw();

    RESET();
    return RET_SUCCESS;
}

RetType wiz_send_test_task(void *) {
    RESUME();
    static IPv4UDPSocket::addr_t addr;
    addr.ip[0] = 10;
    addr.ip[1] = 10;
    addr.ip[2] = 10;
    addr.ip[3] = 1;
    addr.port = 8000;

    static uint8_t buff[7] = {'L', 'a', 'u', 'n', 'c', 'h', '!'};
    RetType ret = CALL(sock->send(buff, 7, &addr));

    RESET();
    return RET_SUCCESS;
}

RetType net_stack_init(void *) {
    RESUME();

	sched_start(flash_led_task, &wiz_flash);

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


    swprint("Initializing W5500\n");
    RetType ret = CALL(wiznet.init(gateway_addr, subnet_mask, mac_addr, ip_addr));
    if (RET_SUCCESS != ret) {
		swprint("#RED#W5500 init failed\n");
        goto netStackInitDone;
    } else {
		swprint("#GRN#W5500 init OK\n");
    }


    swprint("Initializing network stack\n");
    ret = stack->init();
    if (RET_SUCCESS != ret) {
    	swprint("#RED#Net stack init failed");
        goto netStackInitDone;
    } else {
		swprint("#GRN#Net stack init OK\n");
    }

    swprint("Successfully initialized network interface\n");

    netStackInitDone:
    wiz_flash.period = 1000;
    RESET();
    return RET_ERROR; // Kill task
}