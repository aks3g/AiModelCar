/*
 * pwm_driver.cpp
 *
 * Created: 2013/09/07 4:37:33
 *  Author: sazae7
 */ 

#include <avr/io.h>

#include <xmegaE5/utils.h>
#include <xmegaE5/pwm_driver.h>

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint16_t initialize_tcc4_as_pwm(uint32_t carrerFreqInHz, uint32_t peripheralClock, PWM_CH ch, uint8_t remap)
{
	uint16_t divMap[8] = {0, 1, 2, 4, 8, 64, 256, 1024};
	uint32_t dividedClock = peripheralClock;

	/*J OC4A */
	if (remap) {
		PORTC_REMAP |= BIT(ch);
	}

	/*J 指定されたキャリア周波数を出すにあたって、相応しいClock Divを見つける */
	int i;
	for (i=1 ; i<8 ; ++i) {
		dividedClock = peripheralClock / divMap[i];

		if (dividedClock / carrerFreqInHz < 0x10000) {
			break;
		} else {
			dividedClock = 0;
		}
	}

	/*J 条件を満たせなかったので死ぬ */
	if (dividedClock == 0) {
		return 0;
	}

	/* 13.12.1 CTRLA ? Control register A */
	TCC4_CTRLA = i;

	/* 13.12.2 CTRLB ? Control register B */
	/* WGMODE[2:0] = 0b011 */
	//	TCC4_CTRLB = XCL_TCMODE_PWM_gc; //BUG
	TCC4_CTRLB = 0x03;

	/* 13.12.3 CTRLC ? Control register C */
	/* POLA = 1 */
	/* CMPA = 1 : マッチ時にH出力 */
//	TCC4_CTRLC = TCC4_CTRLC | ((BIT(ch) << 4) | (BIT(ch)));
	TCC4_CTRLC = 0x00;

	/* 13.12.4 CTRLD ? Control register D */
	/* Event Source は使わない */
	TCC4_CTRLD = 0;

	/* 13.12.5 CTRLE ? Control register E */
	/* 13.12.16 CCxL ? Compare or Capture x register Low */
	/* 13.12.17 CCxH ? Compare or Capture x register High */
	/*J CCANIDE[1..0] = 0b01 : Output compare enabled  */
	switch (ch) {
	case TCCxA:
		TCC4_CCA = (0);
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCAMODE_gm) | TC4_CCAMODE0_bm;
		break;
	case TCCxB:
		TCC4_CCB = (0);
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCBMODE_gm) | TC4_CCBMODE0_bm;
		break;
	case TCCxC:
		TCC4_CCC = (0);
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCCMODE_gm) | TC4_CCCMODE0_bm;
		break;
	case TCCxD:
		TCC4_CCD = (0);
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCDMODE_gm) | TC4_CCDMODE0_bm;
		break;
	};

	/* 13.12.6 INTCTRLA ? Interrupt Control register A */
	/* 13.12.7 INTCTRLB ? Interrupt Control register B */
	/* 割込みは使わない */

	/* 13.12.14 PERL ? Period register Low */
	/* 13.12.15 PERH ? Period register High */
	TCC4_PER = (dividedClock / carrerFreqInHz);

	/* TOP値を返す */
	return (dividedClock / carrerFreqInHz);
}

/*---------------------------------------------------------------------------*/
void tcc4_pwm_start(void)
{
	/* 13.13.9 CTRLGCLR/CTRLGSET ? Control register G Clear/Set */
	/* STOPをゼロにする */
	//	TCC4_CTRLGCLR = TC4_STOP_bm; // BUG?
	TCC4_CTRLGCLR = 0x20;
}

/*---------------------------------------------------------------------------*/
void tcc4_pwm_stop(void)
{
	/* 13.13.9 CTRLGCLR/CTRLGSET ? Control register G Clear/Set */
	/* STOPを1にする */
	//	TCC4_CTRLGSET = TC4_STOP_bm; // BUG?
	TCC4_CTRLGSET = 0x20;		
}

/*---------------------------------------------------------------------------*/
void tcc4_pwm_setDuty(uint16_t duty, PWM_CH idx)
{
	switch (idx) {
	case TCCxA:
		TCC4_CCA = duty;
		break;
	case TCCxB:
		TCC4_CCB = duty;
		break;
	case TCCxC:
		TCC4_CCC = duty;
		break;
	case TCCxD:
		TCC4_CCD = duty;
		break;
	};
}

/*---------------------------------------------------------------------------*/
uint16_t tcc4_pwm_getDuty(PWM_CH idx)
{
	uint16_t duty = 0;
	switch (idx) {
	case TCCxA:
		duty = TCC4_CCA;
		break;
	case TCCxB:
		duty = TCC4_CCB;
		break;
	case TCCxC:
		duty = TCC4_CCC;
		break;
	case TCCxD:
		duty = TCC4_CCD;
		break;
	};
	
	return duty;
}

/*---------------------------------------------------------------------------*/
void tcc4_pwm_enable(PWM_CH ch, uint8_t enable)
{
	uint8_t mode = 0;
	
	switch (ch) {
	case TCCxA:
		mode = (enable) ? TC4_CCAMODE0_bm : 0;
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCAMODE_gm) | mode;
		break;
	case TCCxB:
		mode = (enable) ? TC4_CCBMODE0_bm : 0;
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCBMODE_gm) | mode;
		break;
	case TCCxC:
		mode = (enable) ? TC4_CCCMODE0_bm : 0;
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCCMODE_gm) | mode;
		break;
	case TCCxD:
		mode = (enable) ? TC4_CCDMODE0_bm : 0;
		TCC4_CTRLE = (TCC4_CTRLE & ~TC4_CCDMODE_gm) | mode;
		break;
	};

	return;
}
