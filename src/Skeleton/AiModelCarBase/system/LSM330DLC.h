/*
 * LSM330DLC.h
 *
 * Created: 2014/11/04 1:11:39
 *  Author: sazae7
 */ 


#ifndef LSM330DLC_H_
#define LSM330DLC_H_

#include <i2c_driver.h>

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} _3AXIS_uint16;

typedef struct {
	_3AXIS_uint16 acceleration;
	_3AXIS_uint16 angularRate;
} SENSOR_DATA_SET;

int8_t initialize_lsm330dlc(void);
int8_t lsm330dlc_readAllData(SENSOR_DATA_SET *data);
int8_t lsm330dlc_readAccelDataWithInterrupt(SENSOR_DATA_SET *data, i2c_done_callback cb);
int8_t lsm330dlc_readGyeoDataWithInterrupt (SENSOR_DATA_SET *data, i2c_done_callback cb);

#endif /* LSM330DLC_H_ */