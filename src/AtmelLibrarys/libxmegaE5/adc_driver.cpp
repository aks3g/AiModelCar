/*
 * adc_driver.cpp
 *
 * Created: 2013/09/10 20:13:40
 *  Author: sazae7
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <xmegaE5/utils.h>
#include <xmegaE5/adc_driver.h>

#include <xmegaE5/gpio_driver.h>

static uint16_t sGndLevelGainX1 = 0;
static ADC_GAIN sGain = ADC_GAIN_1X;
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t adc_request(ADC_MODE mode, uint8_t bitResolution);


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t initialize_adcModule(uint32_t sps, uint32_t sysclkHz, ADC_MODE mode, uint8_t bitResolution, ADC_INIT_OPT *opt)
{
	uint32_t divTable[8] = {4, 8, 16, 32, 64, 128, 256, 512};

	//J 動作モードと、ビット解像度をチェック
	if ((mode != ADC_ONESHOT) && (mode != ADC_FREERUN) ) {
		return ADC_UNKNOWN_CAPTURE_MODE;
	}

	if ((bitResolution != 8) && (bitResolution != 12)) {
		return ADC_INVALID_BIT_RESOLUTION;
	}
	
	//J オプション処理
	uint8_t vref_bitmap = 0;
	if (opt == NULL) {
		//J デフォルトはこれ
		vref_bitmap = ADC_REFSEL_INTVCC_gc;
	} else {
		vref_bitmap = ADC_REFSEL_gm & (opt->vrefSelect << ADC_REFSEL_gp);
	}		
 	
	
	//J 適当なSAMPVALになるようなDIVを見つける
	//J ねらったようにならないので、SPSを適当に当てて、現物合わせで使う
	int i=0;
	uint32_t sampval = 0;
	for (i=0 ; i<8 ; ++i) {
		sampval = (sysclkHz / divTable[i]) / ((bitResolution/2) * sps);
		if (sampval <= ADC_SAMPVAL_gm) {
			break;			
		} else {
			sampval = 0xff;
		}
	}

	if (sampval > ADC_SAMPVAL_gm) {
		return ADC_INVALID_SAMPLING_RATE;
	}

	//J ADCのモードを変更
	uint8_t ret = adc_request(mode, bitResolution);
	if (ret != ADC_OK) {
		return ret;
	}

	/* 24.14.3 REFCTRL ? Reference Control register */
	ADCA.REFCTRL = (ADCA.REFCTRL & ~ADC_PRESCALER_gm) | vref_bitmap;

	/* 24.14.4 EVCTRL ? Event Control register */
	// I don't use event control.
	
	/* 24.14.5 PRESCALER ? Clock Prescaler register */
	ADCA.PRESCALER = (i << ADC_PRESCALER_gp);

	/* 24.14.8 SAMPCTRL ? Sampling Time Control register */
	ADCA.SAMPCTRL = (uint8_t)sampval;


	//J PAD GNDの状況を確認して、結果から引く(GAIN x1の場合のみ)
	/* 24.15.1 CTRL ? Control register */
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc | ((ADC_GAIN_1X << ADC_CH_GAIN_gp) & ADC_CH_GAIN_gm);

	/* 24.15.2 MUXCTRL ? MUX Control register */
	ADCA.CH0.MUXCTRL = (ADC_CH_MUXNEGL_GND_gc) | ADC_CH_MUXNEGL_INTGND_gc;

	/* 24.15.3 INTCTRL ? Interrupt Control register */
	//J ポーリングするので、IntlevelはOffで良い
	ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

	/* 24.14.1 CTRLA ? Control register A */
	ADCA.CTRLA |= ADC_START_bm | ADC_ENABLE_bm;

	/* 24.14.6 INTFLAGS ? Interrupt Flags register */
	while (ADCA.INTFLAGS == 0);
	ADCA.INTFLAGS = ADC_CH0IF_bm; // Clear

	sGndLevelGainX1 = (ADCA.CH0RES);


	return ADC_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t adc_selectChannel(ADC_CHANNEL ch, ADC_GAIN gain = ADC_GAIN_1X)
{
	if (ch < ADC_CH0 || ADC_CH15 < ch) {
		return ADC_INVALID_CHANNEL;
	}

	sGain = gain;

	/* 24.15.1 CTRL ? Control register */
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ((gain << ADC_CH_GAIN_gp) & ADC_CH_GAIN_gm);

	/* 24.15.2 MUXCTRL ? MUX Control register */
	ADCA.CH0.MUXCTRL = (ch << ADC_CH_MUXPOS_gp) | ADC_CH_MUXNEGL_INTGND_gc;

	/* 24.15.3 INTCTRL ? Interrupt Control register */
	//J ポーリングするので、IntlevelはOffで良い
	ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

	return ADC_OK;
}

/*---------------------------------------------------------------------------*/
uint16_t adc_grabOneShot(void)
{
	/* 24.14.1 CTRLA ? Control register A */
	ADCA.CTRLA |= ADC_START_bm | ADC_ENABLE_bm;

	/* 24.14.6 INTFLAGS ? Interrupt Flags register */
	while (ADCA.INTFLAGS == 0);
	ADCA.INTFLAGS = ADC_CH0IF_bm; // Clear	

	if (sGain == ADC_GAIN_1X) {
		uint16_t adc = (ADCA.CH0RES);
		return (adc > sGndLevelGainX1) ? (adc - sGndLevelGainX1) : (0);
	} else {
		return (ADCA.CH0RES);	
	}
}

/*---------------------------------------------------------------------------*/
static uint8_t adc_request(ADC_MODE mode, uint8_t bitResolution)
{
	//J 動作モードと、ビット解像度をチェック
	if ((mode != ADC_ONESHOT) && (mode != ADC_FREERUN) ) {
		return ADC_UNKNOWN_CAPTURE_MODE;
	}
	uint8_t freerun = (mode == ADC_FREERUN) ? ADC_FREERUN_bm : 0;

	if ((bitResolution != 8) && (bitResolution != 12)) {
		return ADC_INVALID_BIT_RESOLUTION;
	}
	uint8_t resolution = (bitResolution == 8) ? ADC_RESOLUTION_8BIT_gc : ADC_RESOLUTION_12BIT_gc;

	/* 24.14.1 CTRLA ? Control register A */
	ADCA.CTRLA = ADC_FLUSH_bm | ADC_ENABLE_bm;

	/* 24.14.2 CTRLB ? Control register B */
	ADCA.CTRLB = ADC_CURRLIMIT_NO_gc | freerun | resolution;

	return ADC_OK;
}
