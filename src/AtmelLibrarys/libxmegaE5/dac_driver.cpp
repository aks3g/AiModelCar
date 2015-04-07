/*
 * dac_driver.cpp
 *
 * Created: 2014/02/23 1:50:35
 *  Author: sazae7
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <xmegaE5/utils.h>
#include <xmegaE5/dac_driver.h>

#include <xmegaE5/gpio_driver.h>

/*---------------------------------------------------------------------------*/
uint8_t initialize_dac(DAC_INIT_OPT *opt)
{
	uint8_t vref_select = 0;
	
	if (opt != NULL) {
		vref_select = (opt->vrefSelect << DAC_REFSEL_gp) & DAC_REFSEL_gm;
	} else {
		vref_select = DAC_REFSEL_AVCC_gc;
	}
	
	/* 25.10.1 CTRLA ? Control register A */
	//J DAC‚ð—LŒø‰»
	DACA_CTRLA = DAC_ENABLE_bm;
	
	/* 25.10.2 CTRLB ? Control register B */
	//J ‚Æ‚è‚ ‚¦‚¸—¼•û“®‚­‚æ‚¤‚É‚Í‚·‚é
	DACA_CTRLB = DAC_CHSEL_DUAL_gc;

	/* 25.10.3 CTRLC ? Control register C */
	DACA_CTRLC = vref_select;

	/* 25.10.4 EVCTRL ? Event Control register */
	DACA_EVCTRL = 0;


	return DAC_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t dac_enable(DAC_CH channel, uint8_t enable)
{
	uint8_t ch_bitmap = 0;
	
	if(channel == DAC_CH0) {
		ch_bitmap = DAC_CH0EN_bm;
	} else if (channel == DAC_CH1) {
		ch_bitmap = DAC_CH1EN_bm;		
	} else {
		return DAC_INVALID_CHANNEL;
	}
	
	if (enable) {
		DACA_CTRLA |= ch_bitmap;	
	} else {
		DACA_CTRLA &=~ch_bitmap;
	}
	
	return DAC_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t dac_setValue(DAC_CH channel, uint16_t val)
{
	if(channel == DAC_CH0) {
		/* 25.10.10 CH0DATAH ? Channel 0 Data register High */
		DACA_CH0DATA = val;
	} else if (channel == DAC_CH1) {
		/* 25.10.12 CH1DATAH ? Channel 1 Data register High */
		DACA_CH1DATA = val;
	} else {
		return DAC_INVALID_CHANNEL;
	}

	return DAC_OK;	
}


/*---------------------------------------------------------------------------*/
uint8_t dac_calibrat(DAC_CH channel, uint8_t gain, uint8_t offset)
{
	
	return DAC_OK;
}
