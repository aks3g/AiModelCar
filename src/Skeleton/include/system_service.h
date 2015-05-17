/*
 * system_service.h
 *
 * Created: 2014/11/23 21:28:56
 *  Author: sazae7
 */ 


#ifndef SYSTEM_SERVICE_H_
#define SYSTEM_SERVICE_H_

#include <stdint.h>

extern int main(void);

/*---------------------------------------------------------------------------*/
uint16_t systemServGetSystemBatteryVoltage(void);
uint16_t systemServGetMini4wdBatteryVoltage(void);

typedef enum USER_LED_INDEX_t
{
	USER_LED_0,
	USER_LED_1
} USER_LED_INDEX;


typedef enum USER_LED_MODE_t
{
	USER_LED_ON,
	USER_LED_OFF	
} USER_LED_MODE;

void systemServDebugLed(USER_LED_INDEX ledIndex, USER_LED_MODE on_off);


uint16_t systemServGetSensorDataFrequency(void);
uint16_t systemServGetTimerFrequency(void);


#endif /* SYSTEM_SERVICE_H_ */