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
	
	
		
	//J �E���֌W�̏���
	if (MACHINE_IS_TURNING_RIGHT(ax,ay,zy,gx,gy,gz)) {
		ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_RIGHT;

		//J ���ɉE�ɋȂ����Ă���Œ��ł���΁AEnter�𗎂Ƃ�
		if (ctx->state & MACHINE_STATUS_TURNING_RIGHT) {
			ctx->state &= ~MACHINE_STATUS_ENTER_TURN_RIGHT;
		}
		else {
			ctx->state |= (MACHINE_STATUS_TURNING_RIGHT|MACHINE_STATUS_ENTER_TURN_RIGHT);
		}
	}
	else {
		ctx->state &= ~MACHINE_STATUS_ENTER_TURN_RIGHT;

		//J �E�ɋȂ����Ă���Œ��ł���΁ALeave�𗧂Ă�
		if (ctx->state & MACHINE_STATUS_TURNING_RIGHT) {
			ctx->state |= MACHINE_STATUS_LEAVE_TURN_RIGHT;
			
			ctx->state &=~MACHINE_STATUS_TURNING_RIGHT;
		}
		//J ����ȊO�̏ꍇ�ALeave�͏�ɗ��Ƃ�
		else {
			ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_RIGHT;
		}
	}
	
	//J �����֌W�̏���
	if (MACHINE_IS_TURNING_LEFT(ax,ay,zy,gx,gy,gz)) {
		ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_LEFT;

		//J ���ɍ��ɋȂ����Ă���Œ��ł���΁AEnter�𗎂Ƃ�
		if (ctx->state & MACHINE_STATUS_TURNING_LEFT) {
			ctx->state &= ~MACHINE_STATUS_ENTER_TURN_LEFT;
		}
		else {
			ctx->state |= (MACHINE_STATUS_TURNING_LEFT|MACHINE_STATUS_ENTER_TURN_LEFT);
		}
	}
	else {
		ctx->state &= ~MACHINE_STATUS_ENTER_TURN_LEFT;

		//J ���ɋȂ����Ă���Œ��ł���΁ALeave�𗧂Ă�
		if (ctx->state & MACHINE_STATUS_TURNING_LEFT) {
			ctx->state |= MACHINE_STATUS_LEAVE_TURN_LEFT;
			
			ctx->state &=~MACHINE_STATUS_TURNING_LEFT;
		}
		//J ����ȊO�̏ꍇ�ALeave�͏�ɗ��Ƃ�
		else {
			ctx->state &= ~MACHINE_STATUS_LEAVE_TURN_LEFT;
		}
	}

	//J �o�菈��
	ctx->state &=~MACHINE_STATUS_ENTER_UPHILL;
	ctx->state &=~MACHINE_STATUS_LEAVE_UPHILL;
	ctx->state &=~MACHINE_STATUS_ENTER_DOWNHILL;
	ctx->state &=~MACHINE_STATUS_LEAVE_DOWNHILL;

	if (ctx->state & MACHINE_STATUS_RUNNING_ON_UPHILL) {
		//J �����x�̒l���猩�āA���R�ɂȂ����ƌ�����܂�
		if (MACHINE_IS_LEVEL_OFF(ax,ay,az,gx,gy,gz)) {
			ctx->state |= MACHINE_STATUS_LEAVE_UPHILL;			
			ctx->state &=~MACHINE_STATUS_RUNNING_ON_UPHILL;
		}
	}
	else if (0 == (ctx->state & MACHINE_STATUS_RUNNING_ON_DOWNHILL)){
		//J �W���C���̒l���猩�āA�o���Ă������Ȃ痧�Ă�
		if (MACHINE_IS_GETTING_UP(ax,ay,zy,gx,gy,gz)) {
			ctx->state |= (MACHINE_STATUS_RUNNING_ON_UPHILL|MACHINE_STATUS_ENTER_UPHILL);
		}
	}

	//J ���菈��
	if (ctx->state & MACHINE_STATUS_RUNNING_ON_DOWNHILL) {
		//J �����x�̒l���猩�āA���R�ɂȂ����ƌ�����܂�
		if (MACHINE_IS_LEVEL_OFF(ax,ay,az,gx,gy,gz)) {
			ctx->state |= MACHINE_STATUS_LEAVE_DOWNHILL;
			ctx->state &=~MACHINE_STATUS_RUNNING_ON_DOWNHILL;
		}
	}
	else if (0 == (ctx->state & MACHINE_STATUS_RUNNING_ON_UPHILL)){
		//J �W���C���̒l���猩�āA�o���Ă������Ȃ痧�Ă�
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