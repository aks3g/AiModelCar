/*
 * physical_conversion_support.h
 *
 * Created: 2015/05/15 22:36:16
 *  Author: sazae7
 */ 


#ifndef PHYSICAL_CONVERSION_SUPPORT_H_
#define PHYSICAL_CONVERSION_SUPPORT_H_

#include <stdint.h>

/*---------------------------------------------------------------------------*/
//J 物理量変換サポート
#define CONVERT_ACCELERATION_TO_mG(v)					(12 * (int16_t)(v/16))
#define CONVERT_GYRO_TO_mDEGREE_PER_SEC(v)				(70 * (int16_t)v)

#define CONVERT_mG_TO_DIGIT(v)							(((int32_t)v/12)*16)
#define CONVERT_mDEGREE_PER_SEC_TO_DIGIT(v)				((int32_t)v/70)

//J 物理量からわかる情報, センサからの生データを期待
#define MACHINE_IS_UPSIDE_DOWN(ax, ay, az, gx, gy,gz)	(az < CONVERT_mG_TO_DIGIT(-800))
#define MACHINE_IS_STABLE(ax, ay, az, gx, gy,gz)		(gx < CONVERT_mDEGREE_PER_SEC_TO_DIGIT(  25*1000))
#define MACHINE_IS_TURNING_RIGHT(ax, ay, az, gx, gy,gz)	(gz < CONVERT_mDEGREE_PER_SEC_TO_DIGIT(-100*1000))
#define MACHINE_IS_TURNING_LEFT(ax, ay, az, gx, gy,gz)	(gz > CONVERT_mDEGREE_PER_SEC_TO_DIGIT( 100*1000))
#define MACHINE_IS_GETTING_UP(ax, ay, az, gx, gy,gz)	(gy > CONVERT_mDEGREE_PER_SEC_TO_DIGIT( 100*1000))
#define MACHINE_IS_GETTING_DOWN(ax, ay, az, gx, gy,gz)	(gy < CONVERT_mDEGREE_PER_SEC_TO_DIGIT(-100*1000))
#define MACHINE_IS_LEVEL_OFF(ax, ay, az, gx, gy,gz)		((CONVERT_mG_TO_DIGIT(-100) < ax) && (ax < CONVERT_mG_TO_DIGIT( 100)))


//J 連続的にセンサーデータを与えることで状況を推定
#define MACHINE_STATUS_ENTER_TURN_RIGHT					(0x1 <<  0) //0x0001
#define MACHINE_STATUS_TURNING_RIGHT					(0x1 <<  1) //0x0002
#define MACHINE_STATUS_LEAVE_TURN_RIGHT					(0x1 <<  2) //0x0004
#define MACHINE_STATUS_ENTER_TURN_LEFT					(0x1 <<  3) //0x0008
#define MACHINE_STATUS_TURNING_LEFT						(0x1 <<  4) //0x0010
#define MACHINE_STATUS_LEAVE_TURN_LEFT					(0x1 <<  5) //0x0020
#define MACHINE_STATUS_ENTER_UPHILL						(0x1 <<  6) //0x0040
#define MACHINE_STATUS_RUNNING_ON_UPHILL				(0x1 <<  7) //0x0080
#define MACHINE_STATUS_LEAVE_UPHILL						(0x1 <<  8) //0x0100
#define MACHINE_STATUS_ENTER_DOWNHILL					(0x1 <<  9) //0x0200
#define MACHINE_STATUS_RUNNING_ON_DOWNHILL				(0x1 << 10) //0x0400
#define MACHINE_STATUS_LEAVE_DOWNHILL					(0x1 << 11) //0x0800

#define MACHINE_STATUS_STABLE							(0x1 << 12) //0x1000

#define ATTITUDE_ESTIMATION_NUM_HISTORY					(8)

typedef struct ATTITUDE_ESTIMATION_CTX_t {
	uint16_t state;

	uint8_t wptr;
	int16_t historyAx[ATTITUDE_ESTIMATION_NUM_HISTORY];
	int16_t historyAy[ATTITUDE_ESTIMATION_NUM_HISTORY];
	int16_t historyAz[ATTITUDE_ESTIMATION_NUM_HISTORY];
	int16_t historyGx[ATTITUDE_ESTIMATION_NUM_HISTORY];
	int16_t historyGy[ATTITUDE_ESTIMATION_NUM_HISTORY];
	int16_t historyGz[ATTITUDE_ESTIMATION_NUM_HISTORY];
} ATTITUDE_ESTIMATION_CTX;

void initialize_attitudeEstimation(ATTITUDE_ESTIMATION_CTX *ctx);
uint16_t attitudeEstimationUpdate(ATTITUDE_ESTIMATION_CTX *ctx, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);

#endif /* PHYSICAL_CONVERSION_SUPPORT_H_ */