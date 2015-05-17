/*
 * using_physical_conversion.cpp
 *
 * Created: 2015/04/12 23:30:25
 *  Author: sazae7
 */ 

#include <stdlib.h>
#include <string.h>

#include <userland.h>
#include <physical_conversion_support.h>
#include "log_data_structure.h"

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
volatile static uint8_t sRunning = 0;
volatile static uint8_t sLogging = 0;


/*---------------------------------------------------------------------------*/
volatile static uint32_t sTimeStamp = 0;

#define LOG_RECODE_SIZE (16)
static USER_SENSOR_DATA_WITH_TIMESTAMP sSensorLog[2][LOG_RECODE_SIZE];
static bool    sFlagSensorLogIsFull = false;
static uint8_t sActiveIndex = 0;
static uint8_t sWritePtr    = 0;

static FIL sLogFile;

static ATTITUDE_ESTIMATION_CTX sAttitudeEstimationCtx;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static bool sGetFlagOnce(bool &flag);
static uint16_t sSensorDataToText(uint32_t timestamp, uint8_t duty, uint16_t state, void *data, char *string, uint16_t len);



/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void initialize_userland(void)
{
	//J ユーザログファイルを開く
	char logFileName[16] = "ulog.txt";
	int ret = f_open(&sLogFile, logFileName,  FA_WRITE | FA_OPEN_ALWAYS);
	//J 基本追記でやります
	if (ret == FR_OK) {
		UINT len = 0;
		f_lseek(&sLogFile, f_size(&sLogFile));
		f_write(&sLogFile, "-- Start Userland\n", 18, &len);
		f_sync(&sLogFile);
		} else {
	}

	initialize_attitudeEstimation(&sAttitudeEstimationCtx);

	return;
}

/*---------------------------------------------------------------------------*/
#define SENSOR_RECODE_TEXT_LEN		(128)
static char sString[(SENSOR_RECODE_TEXT_LEN + 1)];

void updateUserland(void)
{
	//J センサのバッファがいっぱいになっていた場合の処理
	if (sGetFlagOnce(sFlagSensorLogIsFull)) {
		uint8_t idx = 1-sActiveIndex;
		char *str = sString;
		for (int i=0 ; i<LOG_RECODE_SIZE ; ++i) {
			uint32_t len = sSensorDataToText(sSensorLog[idx][i].timeStamp, sSensorLog[idx][i].duty, sSensorLog[idx][i].state, &sSensorLog[idx][i].data, str, SENSOR_RECODE_TEXT_LEN+1);
			f_write(&sLogFile, (void *)sString, len, &len);
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
		duty =  80;
	}
	else {
		duty =  0;
	}

	if (sLogging) {
		memcpy((void *)&sSensorLog[sActiveIndex][sWritePtr].data, (void *)sensor, sizeof(USER_SENSOR_DATA_SET));

		uint16_t state = attitudeEstimationUpdate(
								&sAttitudeEstimationCtx,
								sSensorLog[sActiveIndex][sWritePtr].data.acceleration.x,
								sSensorLog[sActiveIndex][sWritePtr].data.acceleration.y,
								sSensorLog[sActiveIndex][sWritePtr].data.acceleration.z,
								sSensorLog[sActiveIndex][sWritePtr].data.angularRate.x,
								sSensorLog[sActiveIndex][sWritePtr].data.angularRate.y,
								sSensorLog[sActiveIndex][sWritePtr].data.angularRate.z);

		sSensorLog[sActiveIndex][sWritePtr].duty  = duty;
		sSensorLog[sActiveIndex][sWritePtr].state = state;
		sSensorLog[sActiveIndex][sWritePtr++].timeStamp = sTimeStamp++;

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

}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void onSw0Pressed(void)
{
	sRunning   = 1 - sRunning;

	//J sRunning が 0 -> 1 に変化したタイミングでログを撮り始める
	if (sRunning == 1) {
		sLogging = 1;
		sTimeStamp = 0;
	}
	else {
		sLogging = 0;
	}

	f_sync(&sLogFile);

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

static uint16_t sSensorDataToText(uint32_t timestamp, uint8_t duty, uint16_t state, void *data, char *string, uint16_t len)
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
	next = sConvertToHexString8 (next, duty);      *next = ','; next++;
	next = sConvertToHexString16(next, state);     *next = ','; next++;
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