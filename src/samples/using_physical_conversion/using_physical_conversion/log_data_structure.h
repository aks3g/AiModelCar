/*
 * log_data_structure.h
 *
 * Created: 2015/04/12 23:33:10
 *  Author: sazae7
 */ 


#ifndef LOG_DATA_STRUCTURE_H_
#define LOG_DATA_STRUCTURE_H_

/*---------------------------------------------------------------------------*/
typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} user3AXIS_uint16;

typedef struct {
	user3AXIS_uint16 acceleration;
	user3AXIS_uint16 angularRate;
} USER_SENSOR_DATA_SET;

typedef struct {
	uint32_t timeStamp;
	uint8_t  duty;
	uint16_t state;
	USER_SENSOR_DATA_SET data;
}USER_SENSOR_DATA_WITH_TIMESTAMP;




#endif /* LOG_DATA_STRUCTURE_H_ */