/*
 * AiMini4wdBase.cpp
 *
 * Created: 2014/10/29 1:07:56
 *  Author: sazae7
 */ 

#include <avr/io.h>

#include <stdint.h>
#include <string.h>

#include <utils.h>
#include <core_driver.h>
#include <timer_driver.h>
#include <gpio_driver.h>
#include <spi_driver.h>
#include <i2c_driver.h>
#include <adc_driver.h>
#include <uart_driver.h>

#include "ff.h"
#include "ioconfig.h"
#include "LSM330DLC.h"

#include "system_service.h"
#include "userland.h"

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint32_t gFreerunCount = 0;

/*---------------------------------------------------------------------------*/
static uint16_t sBusVoltage     = 0xFFFF;
static uint16_t sBatteryVoltage = 0xFFFF;

//J ��d��1V�ŁA12�r�b�g�𑜓x��O��, 10:1 �ɕ������ꂽ���m
//J X : 1[V] = Val : 4095
//J X = (1[V] x Val) / 4095
//J E = 11 x X
//J E = 11 x 1[V] x Val/4095
//J E[mV] = 11 x 1000 x val/4095
#define CONVERT_TO_MILI_VOLT(val)	(uint16_t)(((uint32_t)(val)*1000*11)/4095)

/*---------------------------------------------------------------------------*/
static void _sTimerCB(void);
static void _sSwPwmCB(void);
static void _sInt1Cb(void);
static void _sI2cAccelDoneCb(uint8_t status);
static void _sI2cGyroDoneCb (uint8_t status);

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static FATFS sDrive;

/*---------------------------------------------------------------------------*/
static uint8_t sSwPwmDuty = 0; //J �\�t�g�E�F�APWM�pDuty



/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
int main(void)
{
	uint8_t ret = 0;
	//------------------------------------------------------------------------
	//J ���Ӌ@��̏���������
	//J GPIO �̏�����
	//J Boot Loader Mode�ɓ��邩�ǂ������m�F���邽�߂ɍŏ���GPIO���K�v
	ret = init_gpio();
	if (ret != GPIO_OK) {
		;
	}
	
	//J �N������SW0�������ꂽ��Ԃ������ꍇ�ɂ�Boot Loader���[�h�ɓ����
	uint8_t sw0In = 0xFF;
	ret = gpio_input(nSW0_PORT, nSW0_PIN, &sw0In);
	if (ret == GPIO_OK) {
		//J �N������SW0��������Ă����Bootloder���[�h�ɓ���
		if (sw0In == 0) {
			//J ���̌�X�e�[�^�XLED��t���邱�Ƃ������̂ł����Œ����Ƃ�
			gpio_output(nSTATUS_LED_PORT, nSTATUS_LED_PIN, 0);
			
			//J Boot Loader Time�I
			asm("jmp	0x8000");
		}
	}

	//J Clock����̏�����
	uint32_t systemClock = initialize_clock(CORE_SRC_RC32MHZ, NULL, NULL);

	//J �����ݏ����̏�����
	gpio_configeInterrupt(nINT1_PORT, nINT1_PIN, INT_RISING, _sInt1Cb);

	//J ��^�C�}�[(5ms)
	ret = initialize_tcc5_as_timerMilSec(5, systemClock);
	if (ret == TIMER_OK) {
		tcc5_timer_registerCallback(_sTimerCB);
	}

	//J �\�t�g�E�F�APWM�p�̃^�C�}
	ret = initialize_tcc4_as_timerMicroSec(10, systemClock);
	if (ret == TIMER_OK) {
		tcc4_timer_registerCallback(_sSwPwmCB);
	}

	//J I2C ��400kHz
	ret = initialize_i2c(400000, systemClock, I2C_MASTER, NULL);
	if (ret != I2C_OK) {
		;//J ���蓾�Ȃ�
	}

	//J UART��������
	ret = initialize_uart(USART0_ON_PORTD_1, USER_UART_BAUDRATE, systemClock);
	if (ret != UART_OK) {
	}

	//J Battery �Ď��pADC�̋N��
	ADC_INIT_OPT adcOpt;
	adcOpt.vrefSelect = VREF_INTERNAL_1V;

	//J ����1V����d���Ƃ��Ď��
	ret = initialize_adcModule(10000, systemClock, ADC_ONESHOT, 12, &adcOpt);
	if (ret == ADC_OK) {
	}
	
	//------------------------------------------------------------------------
	//J ���̑�SW�I�ȏ���������

	//J �����ݗL��
	Enable_Int();

	//------------------------------------------------------------------------
	//J �����ݏ�����K�v�Ƃ���e�평��������
	
	//J SD�J�[�h�𓮂��悤�ɂ���
	ret = f_mount(&sDrive, "", 0);
	if (ret != FR_OK) {
		;
	}

	initialize_lsm330dlc();


	//J ���[�U�����h�̏�����
	initialize_userland();

	//J �ݒ肪��ʂ�I��������_�ŃX�e�[�^�XLED��_��
	gpio_output(nSTATUS_LED_PORT, nSTATUS_LED_PIN, 0);

	//------------------------------------------------------------------------
	//J ���C������
	while(1) {
		updateUserland();
	}
	
	//J �����ɂ͗��Ȃ��͂�
}

/*---------------------------------------------------------------------------*/
uint16_t systemServGetSystemBatteryVoltage(void)
{
	return sBatteryVoltage;
}


uint16_t systemServGetMini4wdBatteryVoltage(void)
{
	return sBusVoltage;
}

void systemServDebugLed(USER_LED_INDEX ledIndex, USER_LED_MODE on_off)
{
	if (ledIndex == USER_LED_0) {
		gpio_output(nDEBUG_LED0_PORT, nDEBUG_LED0_PIN, ((on_off == USER_LED_ON) ? 0 : 1));
	}
	else if (ledIndex == USER_LED_1) {
		gpio_output(nDEBUG_LED1_PORT, nDEBUG_LED1_PIN, ((on_off == USER_LED_ON) ? 0 : 1));
	}
}

uint16_t systemServGetSensorDataFrequency(void)
{
	return 400; //TODO
}


uint16_t systemServGetTimerFrequency(void)
{
	//J 5ms�̎����J�E���^�Ȃ̂�
	return 200; //TODO
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
typedef struct {
	uint8_t stat;
	uint8_t statCnt;
	uint8_t prebStableStat;
} SW_CONTEXT;

SW_CONTEXT sSw0Ctx = {0, 0, 0};
SW_CONTEXT sSw1Ctx = {0, 0, 0};
SW_CONTEXT sSw2Ctx = {0, 0, 0};


static uint8_t sChgStat   = 0;
static uint8_t sChgStatCnt= 0;

static uint16_t sLowBatteryLedCnt = 0;

/*---------------------------------------------------------------------------*/
static void _sTimerCB(void)
{
	gFreerunCount++;
	int ret;
	//-----------------------------------------------------------------------
	// �~�j�l�쑤�� �d�����擾
	ret = adc_selectChannel(ADC_CH_BUS_SENS, ADC_GAIN_1X);
	if (ret == ADC_OK) {
		sBusVoltage = adc_grabOneShot();
		sBusVoltage = CONVERT_TO_MILI_VOLT(sBusVoltage);
	}

	//-----------------------------------------------------------------------
	// Battery �d�����擾
	ret = adc_selectChannel(ADC_CH_BAT_SENS, ADC_GAIN_1X);
	if (ret == ADC_OK) {
		sBatteryVoltage = adc_grabOneShot();
		sBatteryVoltage = CONVERT_TO_MILI_VOLT(sBatteryVoltage);
	}

	//------------------------------------------------------------------------
	//J ���[�U�����h����Timer Callback��Ԃ�
	onTimerInterrupt(gFreerunCount);

	//------------------------------------------------------------------------
	//J CHG �̏�Ԃ��m�F���āA�[�d���Ȃ�LED��_����
	uint8_t chgStat = 0;
	(void)gpio_input(nBAT_CHG_PORT, nBAT_CHG_PIN, &chgStat);

	if (chgStat == sChgStat) {
		//J 200ms�Ď����āA���̊ԓ����X�e�[�g�Ȃ�ALED���X�V����
		if (sChgStatCnt < 200) {
			sChgStatCnt += 5;
		} else {
			//J �[�d��
			if (chgStat == 0) {
				gpio_output(nCHG_LED_PORT, nCHG_LED_PIN, 0);
			}
			//J ��[�d��
			else {
				//J Low Battery ����
				if (sBatteryVoltage < 3400 && sChgStat != 0) {
					if (sLowBatteryLedCnt) {
						sLowBatteryLedCnt--;
					} else {
						sLowBatteryLedCnt = 100; //500ms �Ńg�O��
						gpio_output_toggle(nCHG_LED_PORT, nCHG_LED_PIN);
					}
				} else {
					sLowBatteryLedCnt = 100;
					gpio_output(nCHG_LED_PORT, nCHG_LED_PIN, 1);
				}
			}
		}
	} else {
		sChgStat = chgStat;
		sChgStatCnt = 0;
	}

	//------------------------------------------------------------------------
	//J SWx �̏�Ԃ��m�F(�����ꂽ���A������Ă���Ԗ���A�����ꂽ���@��CB�j
	uint8_t swStat = 0;

	//J SW0 �̓���
	(void)gpio_input(nSW0_PORT, nSW0_PIN, &swStat);
	if (swStat == sSw0Ctx.stat) {
		//J 20ms�Ď����āA���̊ԓ����X�e�[�g�Ȃ�ALED���X�V����
		if (sSw0Ctx.statCnt < 20) {
			sSw0Ctx.statCnt += 5;
		} else {
			//J �����ꂽ���ƁA�b���ꂽ��
			if (sSw0Ctx.prebStableStat != sSw0Ctx.stat) {
				//J �����ꂽ��(1�񂾂�)
				if (sSw0Ctx.stat == 0) {
					onSw0Pressed();
				}
				//J �����ꂽ��(1�񂾂�)
				else {
					onSw0Released();
				}

				sSw0Ctx.prebStableStat = sSw0Ctx.stat;
			}
			//J �����ꑱ���Ă��鎞(���s�[�g)
			else if (sSw0Ctx.stat == 0){
				onSw0Repeat();
			}
		}
	} else {
		sSw0Ctx.stat = swStat;
		sSw0Ctx.statCnt = 0;
	}

	//J SW1 �̓���
	(void)gpio_input(nSW1_PORT, nSW1_PIN, &swStat);
	if (swStat == sSw1Ctx.stat) {
		//J 20ms�Ď����āA���̊ԓ����X�e�[�g�Ȃ�ALED���X�V����
		if (sSw1Ctx.statCnt < 20) {
			sSw1Ctx.statCnt += 5;
		} else {
			//J �����ꂽ���ƁA�b���ꂽ��
			if (sSw1Ctx.prebStableStat != sSw1Ctx.stat) {
				//J �����ꂽ��(1�񂾂�)
				if (sSw1Ctx.stat == 0) {
					onSw1Pressed();
				}
				//J �����ꂽ��(1�񂾂�)
				else {
					onSw1Released();
				}

				sSw1Ctx.prebStableStat = sSw1Ctx.stat;
			}
			//J �����ꑱ���Ă��鎞(���s�[�g)
			else if (sSw1Ctx.stat == 0){
				onSw1Repeat();
			}
		}
	} else {
		sSw1Ctx.stat = swStat;
		sSw1Ctx.statCnt = 0;
	}

	//J SW2 �̓���
	(void)gpio_input(nSW2_PORT, nSW2_PIN, &swStat);
	if (swStat == sSw2Ctx.stat) {
		//J 20ms�Ď����āA���̊ԓ����X�e�[�g�Ȃ�ALED���X�V����
		if (sSw2Ctx.statCnt < 20) {
			sSw2Ctx.statCnt += 5;
		} else {
			//J �����ꂽ���ƁA�b���ꂽ��
			if (sSw2Ctx.prebStableStat != sSw2Ctx.stat) {
				//J �����ꂽ��(1�񂾂�)
				if (sSw2Ctx.stat == 0) {
					onSw2Pressed();
				}
				//J �����ꂽ��(1�񂾂�)
				else {
					onSw2Released();
				}

				sSw2Ctx.prebStableStat = sSw2Ctx.stat;
			}
			//J �����ꑱ���Ă��鎞(���s�[�g)
			else if (sSw2Ctx.stat == 0){
				onSw2Repeat();
			}
		}
	} else {
		sSw2Ctx.stat = swStat;
		sSw2Ctx.statCnt = 0;
	}

	return;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
SENSOR_DATA_SET sTmpData;
static void _sInt1Cb(void)
{
	lsm330dlc_readAccelDataWithInterrupt(&sTmpData, _sI2cAccelDoneCb);

	return;
}

/*---------------------------------------------------------------------------*/
//J �����x���W���C���̏��Ŏ��
static void _sI2cAccelDoneCb(uint8_t status)
{
	if (status == I2C_OK) {
		lsm330dlc_readGyeoDataWithInterrupt(&sTmpData, _sI2cGyroDoneCb);
	}
}

static void _sI2cGyroDoneCb(uint8_t status)
{
	if (status == I2C_OK) {
		sSwPwmDuty = onSensorInterrupt((void *)&sTmpData);
	}	
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t sCountUp100Timer = 0;
static void _sSwPwmCB(void)
{
	sCountUp100Timer++;
		
	if (sCountUp100Timer == 100) {
		sCountUp100Timer = 0;
		
		//PWM -> PIO = H
		if (sSwPwmDuty != 0) {
			gpio_output(MOTOR_ON_PORT, MOTOR_ON_PIN, 1);
		}
	}

	if (sCountUp100Timer > sSwPwmDuty) {
		//PWM -> PIO = L
		gpio_output(MOTOR_ON_PORT, MOTOR_ON_PIN, 0);
	}
	
}
