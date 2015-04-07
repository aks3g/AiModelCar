/*
 * timer_driver.cpp
 *
 * Created: 2013/09/07 14:39:43
 *  Author: sazae7
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <xmegaE5/utils.h>
#include <xmegaE5/timer_driver.h>

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static timer_callback sCallback = NULL;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t initialize_tcc5_as_timerMilSec(uint32_t periodInMiliSec, uint32_t peripheralClock)
{
	uint16_t divMap[8] = {0, 1, 2, 4, 8, 64, 256, 1024};
	volatile uint32_t dividedClock = peripheralClock;

	/*J 指定された周期を出すにあたって、相応しいClock Divを見つける */
	volatile int i;
	for (i=1 ; i<8 ; ++i) {
		dividedClock = peripheralClock / divMap[i];

		//J 16bit のカウントで収まる範囲で動かす
		if ( ((dividedClock * periodInMiliSec) / (uint32_t)(1000)) < 0x10000) {
			break;
		} else {
			dividedClock = 0;
		}
	}

	/*J 条件を満たせなかったので死ぬ */
	if (dividedClock == 0) {
		return TIMER_CANNOT_CONFIGURATE;
	}

	/**/
	TCC5.CTRLA = i;	
	TCC5.CTRLB = TC45_WGMODE_NORMAL_gc;
	TCC5.CTRLC = 0;
	TCC5.CTRLD = TC45_EVACT_OFF_gc | TC45_EVSEL_OFF_gc;

	TCC5.INTCTRLA = TC45_ERRINTLVL_OFF_gc | TC45_OVFINTLVL_MED_gc;
	TCC5.INTCTRLB = TC45_CCDINTLVL_OFF_gc | TC45_CCCINTLVL_OFF_gc | TC45_CCBINTLVL_OFF_gc | TC45_CCAINTLVL_OFF_gc;
	TCC5.INTFLAGS = 1; //J オーバーフロー割込みを有効化
	
	TCC5.PER = (dividedClock * periodInMiliSec) / (1000L);
	
	return TIMER_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t initialize_tcc5_as_timerMicroSec(uint32_t periodInMicroSec, uint32_t peripheralClock)
{
	uint16_t divMap[8] = {0, 1, 2, 4, 8, 64, 256, 1024};
	volatile uint32_t dividedClock = peripheralClock;

	/*J 指定された周期を出すにあたって、相応しいClock Divを見つける */
	volatile int i;
	for (i=1 ; i<8 ; ++i) {
		dividedClock = peripheralClock / divMap[i];

		//J 16bit のカウントで収まる範囲で動かす
		if ( ((dividedClock * periodInMicroSec) / (uint32_t)(1000)) < 0x10000) {
			break;
		} else {
			dividedClock = 0;
		}
	}

	/*J 条件を満たせなかったので死ぬ */
	if (dividedClock == 0) {
		return TIMER_CANNOT_CONFIGURATE;
	}

	/**/
	TCC5.CTRLA = i;
	TCC5.CTRLB = TC45_WGMODE_NORMAL_gc;
	TCC5.CTRLC = 0;
	TCC5.CTRLD = TC45_EVACT_OFF_gc | TC45_EVSEL_OFF_gc;

	TCC5.INTCTRLA = TC45_ERRINTLVL_OFF_gc | TC45_OVFINTLVL_MED_gc;
	TCC5.INTCTRLB = TC45_CCDINTLVL_OFF_gc | TC45_CCCINTLVL_OFF_gc | TC45_CCBINTLVL_OFF_gc | TC45_CCAINTLVL_OFF_gc;
	TCC5.INTFLAGS = 1; //J オーバーフロー割込みを有効化

	TCC5.PER = (dividedClock * periodInMicroSec) / (1000L*1000L);

	return TIMER_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t tcc5_timer_registerCallback(timer_callback cb)
{
	if (sCallback == NULL) {
		sCallback = cb;
		return TIMER_OK;
	} else {
		return TIMER_CALLBACK_IS_ALREADY_REGISTERD;	
	}
}


/*---------------------------------------------------------------------------*/
static volatile uint32_t sFreerunCounter = 0;
uint32_t tcc5_timer_getCurrentCount(void)
{
	return sFreerunCounter;
}

/*---------------------------------------------------------------------------*/
ISR(TCC5_OVF_vect)
{
	/* 13.12.10 INTFLAGS ? Interrupt Flags register */
	TCC5.INTFLAGS = RTC_OVFIF_bm; // To Clear

	sFreerunCounter++;

	if (sCallback!=NULL)	{
		sCallback();
	}

	return;
}



/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static timer_callback sCallbackTcc4 = NULL;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t initialize_tcc4_as_timerMilSec(uint32_t periodInMiliSec, uint32_t peripheralClock)
{
	uint16_t divMap[8] = {0, 1, 2, 4, 8, 64, 256, 1024};
	volatile uint32_t dividedClock = peripheralClock;

	/*J 指定された周期を出すにあたって、相応しいClock Divを見つける */
	volatile int i;
	for (i=1 ; i<8 ; ++i) {
		dividedClock = peripheralClock / divMap[i];

		//J 16bit のカウントで収まる範囲で動かす
		if ( ((dividedClock * periodInMiliSec) / (uint32_t)(1000)) < 0x10000) {
			break;
		} else {
			dividedClock = 0;
		}
	}

	/*J 条件を満たせなかったので死ぬ */
	if (dividedClock == 0) {
		return TIMER_CANNOT_CONFIGURATE;
	}

	/**/
	TCC4.CTRLA = i;
	TCC4.CTRLB = TC45_WGMODE_NORMAL_gc;
	TCC4.CTRLC = 0;
	TCC4.CTRLD = TC45_EVACT_OFF_gc | TC45_EVSEL_OFF_gc;

	TCC4.INTCTRLA = TC45_ERRINTLVL_OFF_gc | TC45_OVFINTLVL_MED_gc;
	TCC4.INTCTRLB = TC45_CCDINTLVL_OFF_gc | TC45_CCCINTLVL_OFF_gc | TC45_CCBINTLVL_OFF_gc | TC45_CCAINTLVL_OFF_gc;
	TCC4.INTFLAGS = 1; //J オーバーフロー割込みを有効化

	TCC4.PER = (dividedClock * periodInMiliSec) / (1000L);

	return TIMER_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t initialize_tcc4_as_timerMicroSec(uint32_t periodInMicroSec, uint32_t peripheralClock)
{
	uint16_t divMap[8] = {0, 1, 2, 4, 8, 64, 256, 1024};
	volatile uint32_t dividedClock = peripheralClock;

	/*J 指定された周期を出すにあたって、相応しいClock Divを見つける */
	volatile int i;
	for (i=1 ; i<8 ; ++i) {
		dividedClock = peripheralClock / divMap[i];

		//J 16bit のカウントで収まる範囲で動かす
		if ( ((dividedClock * periodInMicroSec) / (uint32_t)(1000)) < 0x10000) {
			break;
		} else {
			dividedClock = 0;
		}
	}

	/*J 条件を満たせなかったので死ぬ */
	if (dividedClock == 0) {
		return TIMER_CANNOT_CONFIGURATE;
	}

	/**/
	TCC4.CTRLA = i;
	TCC4.CTRLB = TC45_WGMODE_NORMAL_gc;
	TCC4.CTRLC = 0;
	TCC4.CTRLD = TC45_EVACT_OFF_gc | TC45_EVSEL_OFF_gc;

	TCC4.INTCTRLA = TC45_ERRINTLVL_OFF_gc | TC45_OVFINTLVL_MED_gc;
	TCC4.INTCTRLB = TC45_CCDINTLVL_OFF_gc | TC45_CCCINTLVL_OFF_gc | TC45_CCBINTLVL_OFF_gc | TC45_CCAINTLVL_OFF_gc;
	TCC4.INTFLAGS = 1; //J オーバーフロー割込みを有効化

	TCC4.PER = (dividedClock * periodInMicroSec) / (1000L*1000L);

	return TIMER_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t tcc4_timer_registerCallback(timer_callback cb)
{
	if (sCallbackTcc4 == NULL) {
		sCallbackTcc4 = cb;
		return TIMER_OK;
	} else {
		return TIMER_CALLBACK_IS_ALREADY_REGISTERD;	
	}
}


/*---------------------------------------------------------------------------*/
static volatile uint32_t sFreerunCounterTcc4 = 0;
uint32_t tcc4_timer_getCurrentCount(void)
{
	return sFreerunCounterTcc4;
}

/*---------------------------------------------------------------------------*/
ISR(TCC4_OVF_vect)
{
	/* 13.12.10 INTFLAGS ? Interrupt Flags register */
	TCC4.INTFLAGS = RTC_OVFIF_bm; // To Clear

	sFreerunCounter++;

	if (sCallbackTcc4!=NULL)	{
		sCallbackTcc4();
	}

	return;
}