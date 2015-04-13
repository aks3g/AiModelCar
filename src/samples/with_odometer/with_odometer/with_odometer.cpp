/*
 * with_odometer.cpp
 *
 * Created: 2015/04/13 0:39:29
 *  Author: sazae7
 */ 
#include <stdlib.h>
#include <string.h>

#include "userland.h"
#include "parameter.h"


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

typedef struct {
	uint32_t timeStamp;
	uint32_t distance;
	uint8_t  duty;
	USER_SENSOR_DATA_SET data;
}USER_SENSOR_DATA_WITH_TIMESTAMP;


/*---------------------------------------------------------------------------*/
volatile static uint8_t sRunning = 0;
volatile static uint8_t sLogging = 0;

volatile static uint32_t sTimeStamp = 0;

#define LOG_RECODE_SIZE (16)
static USER_SENSOR_DATA_WITH_TIMESTAMP sSensorLog[2][LOG_RECODE_SIZE];
static bool    sFlagSensorLogIsFull = false;
static uint8_t sActiveIndex = 0;
static uint8_t sWritePtr    = 0;

static FIL sLogFile;

volatile static int32_t sDistanceX = 0;
volatile static int32_t sDistanceY = 0;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static bool sGetFlagOnce(bool &flag);
static uint16_t sSensorDataToText(uint32_t timestamp, uint8_t duty, uint32_t distance, void *data, char *string, uint16_t len);


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint32_t max_duty = 80;
uint32_t max_battery_voltage = 2900;
uint32_t min_battery_voltage = 2000;

/*---------------------------------------------------------------------------*/
//J 名前の一致する変数に値を入れる箇所
static void sCheckAndStoreParam(char *name, int16_t val)
{
	//J Vattery が最大電圧である場合のDuty比
	if (strcmp("max_duty", name) == 0) {
		max_duty = (uint32_t)val;
	}
	//J Vatteryの最大電圧[mV]
	else if (strcmp("max_battery_voltage", name) == 0) {
		max_battery_voltage = (uint32_t)val;
	}
	//J Vatteryの最終電圧[mV]
	else if (strcmp("min_battery_voltage", name) == 0) {
		min_battery_voltage = (uint32_t)val;
	}

	return;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void initialize_userland(void)
{
	FIL paramFile;
	
	//J パラメータファイルからパラメータを読み込む
	char fileName[16] = "uparam.txt";
	int8_t ret = f_open(&paramFile, fileName, FA_READ);	
	if (ret == FR_OK) {
		char line[100];
		while (f_gets(line, 100, &paramFile) != NULL) {
			setupParams(line, sCheckAndStoreParam);
		}

		f_close(&paramFile);
	} else {
	}
	
	//J ユーザログファイルを開く
	char logFileName[16] = "ulog.txt";
	ret = f_open(&sLogFile, logFileName,  FA_WRITE | FA_OPEN_ALWAYS);
	//J 基本追記でやります
	if (ret == FR_OK) {
		UINT len = 0;
		f_lseek(&sLogFile, f_size(&sLogFile));
		f_write(&sLogFile, "-- Start Userland\n", 18, &len);
		f_sync(&sLogFile);
	} else {
	}

	return;	
}

/*---------------------------------------------------------------------------*/
#define SENSOR_RECODE_TEXT_LEN		(64)
static char sString[SENSOR_RECODE_TEXT_LEN * LOG_RECODE_SIZE + 1];

typedef enum {
	UART_WAIT_FOR_STX,
	UART_DATA_RECEIVING,
} UART_RECEIVE_STATE;

typedef union {
	uint8_t byte[16];
	struct{
		uint8_t header[4];
		int32_t x;
		int32_t y;
		uint8_t footer[4]; 
	} packet;
} DISTANCE_METER_PACKET;

static UART_RECEIVE_STATE sUartState = UART_WAIT_FOR_STX;
static uint8_t sUartReceiveLen = 0;
static DISTANCE_METER_PACKET sUartReceiveBuf;

void updateUserland(void)
{
	//J センサのバッファがいっぱいになっていた場合の処理
	if (sGetFlagOnce(sFlagSensorLogIsFull)) {
		char *str = sString;
		uint8_t idx = 1-sActiveIndex;
	
		for (int i=0 ; i<LOG_RECODE_SIZE ; ++i) {
			uint32_t len = sSensorDataToText(sSensorLog[idx][i].timeStamp, sSensorLog[idx][i].duty, sSensorLog[idx][i].distance, &sSensorLog[idx][i].data, str, SENSOR_RECODE_TEXT_LEN+1);
			str += len;
		}

		UINT len = 0;
		f_write(&sLogFile, (void *)sString, (str - sString), &len);
		f_sync(&sLogFile);
	}

	//J UARTからの距離計計測
	uint32_t len = uart_get_rxlen();
	while (len) {
		len--;
		
		uint8_t tmp = 0;
		uint32_t rlen = 0;
		(void)uart_rx_n(&tmp, 1, &rlen);
		(void)rlen;

		//J パケットの先頭(0x10)を探す
		if (sUartState == UART_WAIT_FOR_STX) {
			if (tmp == 0x08) {
				sUartState = UART_DATA_RECEIVING;
				
				sUartReceiveLen = 0;
				sUartReceiveBuf.byte[sUartReceiveLen++] = tmp;
			}
		}
		//J 見つけたら16バイト受信する
		else{
			sUartReceiveBuf.byte[sUartReceiveLen++] = tmp;
			if (sUartReceiveLen >= sizeof(DISTANCE_METER_PACKET)) {
				if (sUartReceiveBuf.byte[1]==0x02 && sUartReceiveBuf.byte[15]==0x03) {

					Disable_Int();
					sDistanceX += sUartReceiveBuf.packet.x;
					sDistanceY += sUartReceiveBuf.packet.y;
					Enable_Int();
				}
				
				sUartState = UART_WAIT_FOR_STX;
			}
		}
	}
			
	return;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t onSensorInterrupt(void *sensor)
{
	uint8_t duty = 0;

	if (sRunning) {
		duty =  max_duty;	
	}
	else {
		duty =  0;	
	}

	if (sLogging) {
		memcpy((void *)&sSensorLog[sActiveIndex][sWritePtr].data, (void *)sensor, sizeof(USER_SENSOR_DATA_SET));
		sSensorLog[sActiveIndex][sWritePtr].distance = sDistanceY;
		sSensorLog[sActiveIndex][sWritePtr].duty = duty;
		sSensorLog[sActiveIndex][sWritePtr++].timeStamp = sTimeStamp;

		if (sWritePtr >= LOG_RECODE_SIZE) {
			sActiveIndex = 1-sActiveIndex; //J 0と1をスワップ
			sWritePtr = 0;
			sFlagSensorLogIsFull = true;
		}
	}

	return duty;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onTimerInterrupt(uint32_t tick)
{
	sTimeStamp = tick;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onSw0Pressed(void)
{
	sDistanceY = 0;
	
	sRunning = 1 - sRunning;
	sLogging = 1 - sLogging;
	gpio_output_toggle(nDEBUG_LED0_PORT, nDEBUG_LED0_PIN);
}

/*---------------------------------------------------------------------------*/
void onSw0Repeat(void)
{
	
}

/*---------------------------------------------------------------------------*/
void onSw0Released(void)
{

}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onSw1Pressed(void)
{
	max_duty += 5;
	if (max_duty > 100)	max_duty = 100;
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
	if (max_duty < 5) {
		max_duty = 0;	
	} else {
		max_duty -= 5;	
	}
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
//J そのうちUTILか何かに引っ越す
static bool sGetFlagOnce(bool &flag)
{
	if (flag) {
		flag = false;
		return true;
	} else {
		return false;
	}
}

/*---------------------------------------------------------------------------*/
static char *sConvertToHexString32(char *buf, uint32_t data);
static char *sConvertToHexString16(char *buf, uint16_t data);
static char *sConvertToHexString8(char *buf, uint8_t data);

static uint16_t sSensorDataToText(uint32_t timestamp, uint8_t duty, uint32_t distance, void *data, char *string, uint16_t len)
{
	if (string == NULL || len == 0) {
		return SENSOR_RECODE_TEXT_LEN;
	}

	if (len <= SENSOR_RECODE_TEXT_LEN) {
		return -1; // TODO
	}

	USER_SENSOR_DATA_SET *sensorData = (USER_SENSOR_DATA_SET *)data;
	char *next = string;
	
	next = sConvertToHexString32(next, timestamp); *next = ','; next++;
	next = sConvertToHexString32(next, distance);  *next = ','; next++;
	next = sConvertToHexString8 (next, duty);      *next = ','; next++;
	next = sConvertToHexString16(next, sensorData->acceleration.x); *next = ',';  next++;
	next = sConvertToHexString16(next, sensorData->acceleration.y); *next = ',';  next++;
	next = sConvertToHexString16(next, sensorData->acceleration.z); *next = ',';  next++;
	next = sConvertToHexString16(next, sensorData->angularRate.x);  *next = ',';  next++;
	next = sConvertToHexString16(next, sensorData->angularRate.y);  *next = ',';  next++;
	next = sConvertToHexString16(next, sensorData->angularRate.z);  *next = '\n'; next++;

	return (uint16_t)(next - string);
}



/*---------------------------------------------------------------------------*/
inline uint8_t hex2ascii(uint8_t dat)
{
	dat = dat & 0x0F;
	
	if (dat<=9) {
		return dat - 0x0 + '0';
		} else {
		return dat - 0xA + 'A';
	}
}

/*---------------------------------------------------------------------------*/
static char *sConvertToHexString32(char *buf, uint32_t data)
{
	*buf++ = hex2ascii(data >> 28);
	*buf++ = hex2ascii(data >> 24);
	*buf++ = hex2ascii(data >> 20);
	*buf++ = hex2ascii(data >> 16);
	*buf++ = hex2ascii(data >> 12);
	*buf++ = hex2ascii(data >>  8);
	*buf++ = hex2ascii(data >>  4);
	*buf++ = hex2ascii(data >>  0);

	return buf;
}

/*---------------------------------------------------------------------------*/
static char *sConvertToHexString16(char *buf, uint16_t data)
{
	*buf++ = hex2ascii(data >> 12);
	*buf++ = hex2ascii(data >>  8);
	*buf++ = hex2ascii(data >>  4);
	*buf++ = hex2ascii(data >>  0);

	return buf;
}

/*---------------------------------------------------------------------------*/
static char *sConvertToHexString8(char *buf, uint8_t data)
{
	*buf++ = hex2ascii(data >>  4);
	*buf++ = hex2ascii(data >>  0);

	return buf;
}