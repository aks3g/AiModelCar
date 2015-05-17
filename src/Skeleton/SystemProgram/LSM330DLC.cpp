/*
 * LSM330DLC.cpp
 *
 * Created: 2014/11/04 1:11:47
 *  Author: sazae7
 */ 


#include <avr/io.h>

#include <stdint.h>

#include <utils.h>
#include <i2c_driver.h>
#include <uart_driver.h>
#include <gpio_driver.h>

#include "ioconfig.h"

#include "LSM330DLC.h"


#define LSM330DLC_ACCEL_ADR		(0x19)
#define LSM330DLC_ANGUL_ADR		(0x6B)

#define LSM330DLC_CTRL_REG1_A		(0x20)
#define LSM330DLC_CTRL_REG2_A		(0x21)
#define LSM330DLC_CTRL_REG3_A		(0x22)
#define LSM330DLC_CTRL_REG4_A		(0x23)
#define LSM330DLC_CTRL_REG5_A		(0x24)
#define LSM330DLC_CTRL_REG6_A		(0x25)
#define LSM330DLC_REFERENCE_A		(0x26)
#define LSM330DLC_STATUS_REG_A		(0x27)
#define LSM330DLC_OUT_X_L_A			(0x28)
#define LSM330DLC_OUT_X_H_A			(0x29)
#define LSM330DLC_OUT_Y_L_A			(0x2A)
#define LSM330DLC_OUT_Y_H_A			(0x2B)
#define LSM330DLC_OUT_Z_L_A			(0x2C)
#define LSM330DLC_OUT_Z_H_A			(0x2D)
#define LSM330DLC_FIFO_CTRL_REG		(0x2E)
#define LSM330DLC_FIFO_SRC_REG		(0x2F)
#define LSM330DLC_INT1_CFG_A		(0x30)
#define LSM330DLC_INT1_SOURCE_A		(0x31)
#define LSM330DLC_INT_THS_A			(0x32)
#define LSM330DLC_INT_DURATION_A	(0x33)
#define LSM330DLC_INT2_CFG_A		(0x34)
#define LSM330DLC_INT2_SOURCE_A		(0x35)
#define LSM330DLC_INT2_THS_A		(0x36)
#define LSM330DLC_INT2_DURATION_A	(0x37)
#define LSM330DLC_CLICK_CFG_A		(0x38)
#define LSM330DLC_CLICK_SRC_A		(0x39)
#define LSM330DLC_CLICK_THS_A		(0x3A)
#define LSM330DLC_TIME_LIMIT_A		(0x3B)
#define LSM330DLC_TIME_LATENCY_A	(0x3C)
#define LSM330DLC_TIME_WINDOW_A		(0x3D)
#define LSM330DLC_ACT_THS			(0x3E)
#define LSM330DLC_ACT_DUR			(0x3F)


#define LSM330DLC_WHO_AM_I_G		(0x0F)
#define LSM330DLC_CTRL_REG1_G		(0x20)
#define LSM330DLC_CTRL_REG2_G		(0x21)
#define LSM330DLC_CTRL_REG3_G		(0x22)
#define LSM330DLC_CTRL_REG4_G		(0x23)
#define LSM330DLC_CTRL_REG5_G		(0x24)
#define LSM330DLC_REFERENCE_G		(0x25)
#define LSM330DLC_OUT_TEMP_G		(0x26)
#define LSM330DLC_STATUS_REG_G		(0x27)
#define LSM330DLC_OUT_X_L_G			(0x28)
#define LSM330DLC_OUT_X_H_G			(0x29)
#define LSM330DLC_OUT_Y_L_G			(0x2A)
#define LSM330DLC_OUT_Y_H_G			(0x2B)
#define LSM330DLC_OUT_Z_L_G			(0x2C)
#define LSM330DLC_OUT_Z_H_G			(0x2D)
#define LSM330DLC_FIFO_CTRL_REG_G	(0x2E)
#define LSM330DLC_FIFO_SRC_REG_G	(0x2F)
#define LSM330DLC_INT1_CFG_G		(0x30)
#define LSM330DLC_INT1_SRC_G		(0x31)
#define LSM330DLC_INT1_TSH_XH_G		(0x32)
#define LSM330DLC_INT1_TSH_XL_G		(0x33)
#define LSM330DLC_INT1_TSH_YH_G		(0x34)
#define LSM330DLC_INT1_TSH_YL_G		(0x35)
#define LSM330DLC_INT1_TSH_ZH_G		(0x36)
#define LSM330DLC_INT1_TSH_ZL_G		(0x37)
#define LSM330DLC_INT1_DURATION_G	(0x38)



int8_t initialize_lsm330dlc(void)
{
	int8_t ret = 0;
	
	uint8_t cmd[2] = {0x0F, 0x00};
	uint8_t res[1] = {0x00};
	
	//J 加速度設定
	cmd[0] = LSM330DLC_CTRL_REG2_A;
	cmd[1] = 0x00; //J フィルタ機能は使わない
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, cmd, 2, 0, 0);

	cmd[0] = LSM330DLC_CTRL_REG3_A;
	cmd[1] = 0x50; //J Data Ready Interrupt 1のみ使う
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, cmd, 2, 0, 0);

	cmd[0] = LSM330DLC_CTRL_REG4_A;
	cmd[1] = 0x38; //J FS1-FS0 = 10: +-16G, HR = 1: High Resolution
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, cmd, 2, 0, 0);

	cmd[0] = LSM330DLC_CTRL_REG5_A;
	cmd[1] = 0x00; //J 
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, cmd, 2, 0, 0);

	cmd[0] = LSM330DLC_CTRL_REG6_A;
	cmd[1] = 0x00; //J INT1 is Active Hi
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, cmd, 2, 0, 0);


	//J ジャイロ側設定
	cmd[0] = LSM330DLC_WHO_AM_I_G;
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, cmd, 1, res, 1);

	cmd[0] = LSM330DLC_CTRL_REG1_G;
	cmd[1] = 0xCF; //J 760Hz, cut off 100Hz, NormalMode, 3-axis Enabled
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, cmd, 2, 0, 0);
	
	cmd[0] = LSM330DLC_CTRL_REG2_G;
	cmd[1] = 0x23; //J Normal Mode, Hipass Filter Cut off Freq = 7.2Hz @ 760Hz
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, cmd, 2, 0, 0);
	
	cmd[0] = LSM330DLC_CTRL_REG3_G;
	cmd[1] = 0x00; //J No Interrupt
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, cmd, 2, 0, 0);
	
	cmd[0] = LSM330DLC_CTRL_REG4_G;
	cmd[1] = 0x30; //J Full Scale = 2000dps
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, cmd, 2, 0, 0);

	cmd[0] = LSM330DLC_CTRL_REG5_G;
	cmd[1] = 0x00; //J No FIFO
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, cmd, 2, 0, 0);


	//J 最後に加速度センサを有効にして割込み開始
	cmd[0] = LSM330DLC_CTRL_REG1_A;
	cmd[1] = 0x77; //J 400Hz / Enable xyz
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, cmd, 2, 0, 0);

	Disable_Int();
	SENSOR_DATA_SET data;
	lsm330dlc_readAllData(&data);
	Enable_Int();

	return ret;
}

int8_t lsm330dlc_readAllData(SENSOR_DATA_SET *data)
{
	uint8_t cmd[1] = {0};
	
	cmd[0] = LSM330DLC_OUT_X_L_A | 0x80;
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, cmd, 1, (uint8_t *)&data->acceleration, 6);

	cmd[0] = LSM330DLC_OUT_X_L_G | 0x80;
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, cmd, 1, (uint8_t *)&data->angularRate,  6);

	return 0;
}


static uint8_t sCmd[1] = {0};
int8_t lsm330dlc_readAccelDataWithInterrupt(SENSOR_DATA_SET *data, i2c_done_callback cb)
{
	sCmd[0] = LSM330DLC_OUT_X_L_A | 0x80;
	(void)i2c_txRxBytes(LSM330DLC_ACCEL_ADR, sCmd, 1, (uint8_t *)&data->acceleration, 6, cb);

	return 0;
}


int8_t lsm330dlc_readGyeoDataWithInterrupt(SENSOR_DATA_SET *data, i2c_done_callback cb)
{
	sCmd[0] = LSM330DLC_OUT_X_L_G | 0x80;
	(void)i2c_txRxBytes(LSM330DLC_ANGUL_ADR, sCmd, 1, (uint8_t *)&data->angularRate,  6, cb);


	return 0;
}

