/*
 * physical_conversion_support.cpp
 *
 * Created: 2015/05/15 5:28:03
 *  Author: owner
 */ 
#include <stdlib.h>
#include <string.h>

#include <physical_conversion_support.h>

/*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*/
int16_t sAverage(int16_t *data, uint8_t len);

/*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*/
void initialize_attitudeEstimation(ATTITUDE_ESTIMATION_CTX *ctx)
{
	memset(ctx, 0x00, sizeof(ATTITUDE_ESTIMATION_CTX));
}

/*-------------------------------------------------------------------------*/
uint16_t attitudeEstimationUpdate(ATTITUDE_ESTIMATION_CTX *ctx, int16_t _ax, int16_t _ay, int16_t _az, int16_t _gx, int16_t _gy, int16_t _gz)
{
	ctx->historyAx[ctx->wptr] = _ax;
	ctx->historyAy[ctx->wptr] = _ay;
	ctx->historyAz[ctx->wptr] = _az;
	ctx->historyGx[ctx->wptr] = _gx;
	ctx->historyGy[ctx->wptr] = _gy;
	ctx->historyGz[ctx->wptr] = _gz;

	ctx->wptr = (ctx->wptr+1) & (ATTITUDE_ESTIMATION_NUM_HISTORY-1);

	int16_t ax = sAverage(ctx->historyAx, ATTITUDE_ESTIMATION_NUM_HISTORY);
	int16_t ay = sAverage(ctx->historyAy, ATTITUDE_ESTIMATION_NUM_HISTORY);
	int16_t az = sAverage(ctx->historyAz, ATTITUDE_ESTIMATION_NUM_HISTORY);
	int16_t gx = sAverage(ctx->historyGx, ATTITUDE_ESTIMATION_NUM_HISTORY);
	int16_t gy = sAverage(ctx->historyGy, ATTITUDE_ESTIMATION_NUM_HISTORY);
	int16_t gz = sAverage(ctx->historyGz, ATTITUDE_ESTIMATION_NUM_HISTORY);
	
	
		
	//J 右回り関係の処理
	if (MACHINE_IS_TURNING_RIGHT(ax,ay,zy,gx,gy,gz)) {
		ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_RIGHT;

		//J 既に右に曲がっている最中であれば、Enterを落とす
		if (ctx->state & MACHINE_STATUS_TURNING_RIGHT) {
			ctx->state &= ~MACHINE_STATUS_ENTER_TURN_RIGHT;
		}
		else {
			ctx->state |= (MACHINE_STATUS_TURNING_RIGHT|MACHINE_STATUS_ENTER_TURN_RIGHT);
		}
	}
	else {
		ctx->state &= ~MACHINE_STATUS_ENTER_TURN_RIGHT;

		//J 右に曲がっている最中であれば、Leaveを立てる
		if (ctx->state & MACHINE_STATUS_TURNING_RIGHT) {
			ctx->state |= MACHINE_STATUS_LEAVE_TURN_RIGHT;
			
			ctx->state &=~MACHINE_STATUS_TURNING_RIGHT;
		}
		//J それ以外の場合、Leaveは常に落とす
		else {
			ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_RIGHT;
		}
	}
	
	//J 左回り関係の処理
	if (MACHINE_IS_TURNING_LEFT(ax,ay,zy,gx,gy,gz)) {
		ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_LEFT;

		//J 既に左に曲がっている最中であれば、Enterを落とす
		if (ctx->state & MACHINE_STATUS_TURNING_LEFT) {
			ctx->state &= ~MACHINE_STATUS_ENTER_TURN_LEFT;
		}
		else {
			ctx->state |= (MACHINE_STATUS_TURNING_LEFT|MACHINE_STATUS_ENTER_TURN_LEFT);
		}
	}
	else {
		ctx->state &= ~MACHINE_STATUS_ENTER_TURN_LEFT;

		//J 左に曲がっている最中であれば、Leaveを立てる
		if (ctx->state & MACHINE_STATUS_TURNING_LEFT) {
			ctx->state |= MACHINE_STATUS_LEAVE_TURN_LEFT;
			
			ctx->state &=~MACHINE_STATUS_TURNING_LEFT;
		}
		//J それ以外の場合、Leaveは常に落とす
		else {
			ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_LEFT;
		}
	}

	//J 登り処理
	ctx->state &=~MACHINE_STATUS_ENTER_UPHILL;
	ctx->state &=~MACHINE_STATUS_LEAVE_UPHILL;
	ctx->state &=~MACHINE_STATUS_ENTER_DOWNHILL;
	ctx->state &=~MACHINE_STATUS_LEAVE_DOWNHILL;

	if (ctx->state & MACHINE_STATUS_RUNNING_ON_UPHILL) {
		//J 加速度の値から見て、平坦になったと見たら折る
		if (MACHINE_IS_LEVEL_OFF(ax,ay,az,gx,gy,gz)) {
			ctx->state |= MACHINE_STATUS_LEAVE_UPHILL;			
			ctx->state &=~MACHINE_STATUS_RUNNING_ON_UPHILL;
		}
	}
	else if (0 == (ctx->state & MACHINE_STATUS_RUNNING_ON_DOWNHILL)){
		//J ジャイロの値から見て、登っていそうなら立てる
		if (MACHINE_IS_GETTING_UP(ax,ay,zy,gx,gy,gz)) {
			ctx->state |= (MACHINE_STATUS_RUNNING_ON_UPHILL|MACHINE_STATUS_ENTER_UPHILL);
		}
	}

	//J 下り処理
	if (ctx->state & MACHINE_STATUS_RUNNING_ON_DOWNHILL) {
		//J 加速度の値から見て、平坦になったと見たら折る
		if (MACHINE_IS_LEVEL_OFF(ax,ay,az,gx,gy,gz)) {
			ctx->state |= MACHINE_STATUS_LEAVE_DOWNHILL;
			ctx->state &=~MACHINE_STATUS_RUNNING_ON_DOWNHILL;
		}
	}
	else if (0 == (ctx->state & MACHINE_STATUS_RUNNING_ON_UPHILL)){
		//J ジャイロの値から見て、登っていそうなら立てる
		if (MACHINE_IS_GETTING_DOWN(ax,ay,zy,gx,gy,gz)) {
			ctx->state |= (MACHINE_STATUS_RUNNING_ON_DOWNHILL|MACHINE_STATUS_ENTER_DOWNHILL);
		}
	}

	return ctx->state;
}


/*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*/
int16_t sAverage(int16_t *data, uint8_t len)
{
	int16_t sum = 0;
	int i=0;
	for (i=0 ; i<len ; ++i) {
		sum += data[i];
	}
	
	return sum/len;
}