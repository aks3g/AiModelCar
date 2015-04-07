/*
 * scratch_userland.cpp
 *
 * Created: 2014/11/23 23:59:56
 *  Author: sazae7
 */ 
#include <stdlib.h>
#include <string.h>

#include "userland.h"
#include "parameter.h"


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
//J Scratch用の定数定義
#define SCRATCH_DATA_REQUEST			0x01
#define SCRATCH_FW_VER					0x04

#define SCRATCH_INDEX_ID				(0x0F)
#define SCRATCH_SENSOR_BOARD_ID			(0x04)

#define SCRATCH_INDEX_RESISTANCE_D		(0x00)
#define SCRATCH_INDEX_RESISTANCE_C		(0x01)
#define SCRATCH_INDEX_RESISTANCE_B		(0x02)
#define SCRATCH_INDEX_BUTTON			(0x03)
#define SCRATCH_INDEX_RESISTANCE_A		(0x04)
#define SCRATCH_INDEX_LIGHT				(0x05)
#define SCRATCH_INDEX_SOUND				(0x06)
#define SCRATCH_INDEX_SLIDER			(0x07)


/*---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------*/
static uint16_t sSwitchPressed = 0xffff;

static USER_SENSOR_DATA_SET sSensorLog[2];
static uint8_t sSensorLogIndex = 0;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void sPackScratchData(uint8_t *buf, uint8_t index, uint16_t value);

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint32_t max_duty = 80;
uint32_t max_battery_voltage = 2900;
uint32_t min_battery_voltage = 2000;


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void initialize_userland(void)
{

	return;
}

/*---------------------------------------------------------------------------*/
void updateUserland(void)
{
	//J UART からの読み込みを行って、SCRATCH_DATA_REQUEST であればデータを返す
	if (uart_get_rxlen() > 0) {
		uint8_t cmd[1] = {0x00};
		uint32_t rlen = 0;

		uart_rx_n(cmd, 1, &rlen);
		if (cmd[0] == SCRATCH_DATA_REQUEST) {
			gpio_output_toggle(nDEBUG_LED1_PORT, nDEBUG_LED1_PIN);
			uint8_t packet[2];

			//J センサーボードである事を自己申告
			sPackScratchData(packet, SCRATCH_INDEX_ID, SCRATCH_SENSOR_BOARD_ID);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_RESISTANCE_D, 400);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_RESISTANCE_C, 300);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_RESISTANCE_B, 200);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_BUTTON, sSwitchPressed);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_RESISTANCE_A, 100);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_LIGHT, 0);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_SOUND, 100);
			uart_tx(packet, 2);

			sPackScratchData(packet, SCRATCH_INDEX_SLIDER, 0);
			uart_tx(packet, 2);
		} else {
			; // Discards.	
		}
	}
	
	return;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t onSensorInterrupt(void *sensor)
{
	sSensorLogIndex = 1-sSensorLogIndex;
	memcpy((void *)&sSensorLog[sSensorLogIndex], (void *)sensor, sizeof(USER_SENSOR_DATA_SET));

	return 0;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onTimerInterrupt(uint32_t tick)
{
	;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onSw0Pressed(void)
{
	sSwitchPressed = 0x0000;

	gpio_output_toggle(nDEBUG_LED0_PORT, nDEBUG_LED0_PIN);
}

/*---------------------------------------------------------------------------*/
void onSw0Repeat(void)
{
	
}

/*---------------------------------------------------------------------------*/
void onSw0Released(void)
{
	sSwitchPressed = 0xffff;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onSw1Pressed(void)
{

}

/*---------------------------------------------------------------------------*/
void onSw1Repeat(void)
{
	
}

/*---------------------------------------------------------------------------*/
void onSw1Released(void)
{

}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onSw2Pressed(void)
{

}

/*---------------------------------------------------------------------------*/
void onSw2Repeat(void)
{

}

/*---------------------------------------------------------------------------*/
void onSw2Released(void)
{

}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#define SCRATCH_VALID_VAL			(1<<7)
static void sPackScratchData(uint8_t *buf, uint8_t index, uint16_t value)
{
	if (buf == NULL) {
		return;
	}
	
	uint8_t upper = (uint8_t)((value & 0x0380) >> 7);
	uint8_t lower = (uint8_t)((value & 0x007f) >> 0);
	
	buf[0] = SCRATCH_VALID_VAL | (index << 3) | upper;
	buf[1] = lower;

	return;
}