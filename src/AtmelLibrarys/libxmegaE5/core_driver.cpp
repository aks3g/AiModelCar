/*
 * core_driver.cpp
 *
 * Created: 2013/09/12 19:46:02
 *  Author: sazae7
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <xmegaE5/utils.h>
#include <xmegaE5/core_driver.h>

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t  sSelectClockSource(CORE_CLK_SRC src)
{
	uint8_t clkSrc = 0;

	switch (src) {
	case CORE_SRC_RC2MHZ:
		clkSrc = CLK_SCLKSEL_RC2M_gc;
		break;
	case CORE_SRC_RC32MHZ:
		clkSrc = CLK_SCLKSEL_RC32M_gc;
		break;
	case CORE_SRC_RC32KHZ:
		clkSrc = CLK_SCLKSEL_RC32K_gc;
		break;
	case CORE_SRC_XTAL:
		clkSrc = CLK_SCLKSEL_XOSC_gc;
		break;
	case CORE_SRC_PLL:
		clkSrc = CLK_SCLKSEL_PLL_gc;
		break;
	case CORE_SRC_RC8MHZ:
		clkSrc = CLK_SCLKSEL_RC8M_gc;
		break;
	default:
		clkSrc = CLK_SCLKSEL_RC2M_gc;
		break;
	}

	return clkSrc;
}


/*---------------------------------------------------------------------------*/
static uint8_t  sClockSourceBitMask(CORE_CLK_SRC src)
{
	uint8_t bitmask = 0;

	switch (src) {
	case CORE_SRC_RC2MHZ:
		bitmask = OSC_RC2MEN_bm;
		break;
	case CORE_SRC_RC32MHZ:
		bitmask = OSC_RC32MEN_bm;
		break;
	case CORE_SRC_RC32KHZ:
		bitmask = OSC_RC32KEN_bm;
		break;
	case CORE_SRC_XTAL:
		bitmask = OSC_XOSCEN_bm;
		break;
	case CORE_SRC_PLL:
		bitmask = OSC_PLLEN_bm;
		break;
	case CORE_SRC_RC8MHZ:
		bitmask = OSC_RC8MEN_bm;
		break;
	default:
		bitmask = 0;
		break;
	}

	return bitmask;
}


/*---------------------------------------------------------------------------*/
static uint32_t sConfigePLL(CORE_PLL_OPTION *opt)
{
	return 0;
}


/*---------------------------------------------------------------------------*/
static uint32_t sConfigePreScaler(CORE_CLK_SRC src, uint32_t pllClkHz, CORE_PRESCALER_OPTION *opt)
{
	uint32_t sysClkHz = 0;

	//J ベースとなるクロックを選ぶ
	switch (src) {
	case CORE_SRC_RC2MHZ:
		sysClkHz = 2000000;
		break;
	case CORE_SRC_RC32MHZ:
		sysClkHz = 32000000;
		break;
	case CORE_SRC_RC32KHZ:
		sysClkHz = 32000;
		break;
	case CORE_SRC_XTAL:
		sysClkHz = 0; //TODO
		break;
	case CORE_SRC_PLL:
		sysClkHz = pllClkHz;
		break;
	case CORE_SRC_RC8MHZ:
		sysClkHz = 8000000;
		break;
	default:
		sysClkHz = 0;
		break;
	}
	
	//J プリスケーラーの設定を行う
	if (opt != 0) {
		//TODO
	}

	return sysClkHz;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint32_t initialize_clock(CORE_CLK_SRC src, CORE_PRESCALER_OPTION *prescalerOpt, CORE_PLL_OPTION *pllOpt)
{
	uint32_t ret = 0;
	uint32_t pllClkHz = 0;
	
	//J ビットマスク取得
	uint8_t bitmask = sClockSourceBitMask(src);
	if (bitmask == 0) {
		return 0;
	}

	//J PLLの初期化を行う。PLLをソースに指定しているが、PLLオプションがNULLの場合には死ぬ
	if (src == CORE_SRC_PLL) {
		pllClkHz = sConfigePLL(pllOpt);
		if (pllClkHz == 0) {
			return 0;
		}				
	}

	//J プリスケーラーの設定を行う
	ret = sConfigePreScaler(src, pllClkHz, prescalerOpt);
	if (ret == 0) {
		return 0;
	}
	
	//J クロック切り替え
	{
		uint8_t clkSrc = sSelectClockSource(src);
	
		/*J 内蔵の8MHz発信器を起動 */
		/* 7.10.1 CTRL ? Oscillator Control register */
		OSC.CTRL |= bitmask;

		/*J 一旦安定するまで待つ */
		while(0 == (OSC.STATUS & bitmask));

		/* 7.9.1 CTRL ? Control register */
		CPU_CCP  = CCP_IOREG_gc; // magic word
		CLK.CTRL = clkSrc;

		/*J 少し待つ */
		volatile long int wait = 1000;
		while(wait--);		
	}
	
	//J 不要なクロックモジュールの電源を落とす
	OSC_CTRL &= ~bitmask;
	
	return ret;
}