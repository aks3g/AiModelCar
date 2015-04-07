/*
 * gpio_driver.cpp
 *
 * Created: 2013/09/08 23:10:51
 *  Author: sazae7
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "utils.h"
#include "gpio_driver.h"

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static PORT_t *checkParamsAndGetPort(GPIO_PORT port);
static uint8_t checkParamsAndGetPinMask(GPIO_PIN pin);
static register8_t *checkParamsAndGetPinControlReg(GPIO_PORT port, GPIO_PIN pin);


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t initialize_gpio(GPIO_PORT port, GPIO_PIN pin, GPIO_DIR dir, GPIO_MODE mode)
{
	/*J PortとPinが正しいか確認して、使える状態にする */
	PORT_t *pPort = checkParamsAndGetPort(port);
	if (pPort == NULL) {
		return GPIO_NOT_FOUND_THE_PORT;
	}

	uint8_t pinMask = checkParamsAndGetPinMask(pin);
	if (pinMask == 0) {
		return GPIO_NOT_FOUNT_THE_PIN;
	}

	/*J ポートの方向をチェック */
	if (dir != GPIO_OUT && dir != GPIO_IN) {
		return GPIO_INVALID_DIRECTION;
	}

	/*J モードの確認 */
	if (mode != GPIO_PUSH_PULL && mode != GPIO_OPEN_DRAIN) {
		return GPIO_INVALID_PIN_MODE;
	}

	/*J ここからは、基本的に安心して作業して良い*/
	if (dir == GPIO_OUT) {
		pPort->DIRSET = pinMask; 
 	} else if(dir == GPIO_IN) {
		pPort->DIRCLR = pinMask;		 
	}

	/*J ピンコントロールを引き抜く */
	register8_t *pinControlReg = checkParamsAndGetPinControlReg(port, pin);
	if (pinControlReg == NULL) {
		return GPIO_NOT_FOUNT_THE_PIN;
	}

	if (mode == GPIO_PUSH_PULL) {
		*pinControlReg = ((*pinControlReg) & ~PORT_OPC_gm) | PORT_OPC_TOTEM_gc;
	} else if (mode == GPIO_OPEN_DRAIN) {
		*pinControlReg = ((*pinControlReg) & ~PORT_OPC_gm) | PORT_OPC_WIREDAND_gc;		
	}

	return GPIO_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t gpio_output(GPIO_PORT port, GPIO_PIN pin, uint8_t out)
{
	/*J PortとPinが正しいか確認して、使える状態にする */
	PORT_t *pPort = checkParamsAndGetPort(port);
	if (pPort == NULL) {
		return GPIO_NOT_FOUND_THE_PORT;
	}

	uint8_t pinMask = checkParamsAndGetPinMask(pin);
	if (pinMask == 0) {
		return GPIO_NOT_FOUNT_THE_PIN;
	}
	
	if (out) {
		pPort->OUTSET = pinMask;
	} else {
		pPort->OUTCLR = pinMask;		
	}

	return GPIO_OK;
}

/*---------------------------------------------------------------------------*/
uint8_t gpio_output_toggle(GPIO_PORT port, GPIO_PIN pin)
{
	/*J PortとPinが正しいか確認して、使える状態にする */
	PORT_t *pPort = checkParamsAndGetPort(port);
	if (pPort == NULL) {
		return GPIO_NOT_FOUND_THE_PORT;
	}

	uint8_t pinMask = checkParamsAndGetPinMask(pin);
	if (pinMask == 0) {
		return GPIO_NOT_FOUNT_THE_PIN;
	}
	
	pPort->OUTTGL = pinMask;

	return GPIO_OK;
}

/*---------------------------------------------------------------------------*/
uint8_t gpio_input(GPIO_PORT port, GPIO_PIN pin, uint8_t *in)
{
	/*J PortとPinが正しいか確認して、使える状態にする */
	PORT_t *pPort = checkParamsAndGetPort(port);
	if (pPort == NULL) {
		return GPIO_NOT_FOUND_THE_PORT;
	}

	uint8_t pinMask = checkParamsAndGetPinMask(pin);
	if (pinMask == 0) {
		return GPIO_NOT_FOUNT_THE_PIN;
	}

	if (in != NULL ) {
		*in = (pPort->IN & pinMask);
	} else {
		return GPIO_NULLPTR;
	}	

	return GPIO_OK;
}



/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static gpio_interrupt_callback sCallback[GPIO_PORT_MAX][GPIO_PIN_MAX]
=
{
	{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
};

/*---------------------------------------------------------------------------*/
uint8_t gpio_configeInterrupt(GPIO_PORT port, GPIO_PIN pin, INTERRUPT_MODE mode, gpio_interrupt_callback callback)
{
	if (!(INT_BOTHE_EDGE <= mode && mode <=INT_LEVEL)) {
		return GPIO_INVALID_INTERRUPT_MODE;
	}

	if (callback == NULL) {
		return GPIO_INVALID_CALLBACK;
	}

	register8_t *pinCtrlReg = checkParamsAndGetPinControlReg(port, pin);
	if (pinCtrlReg == NULL) {
		return GPIO_NOT_FOUNT_THE_PIN;
	}

	uint8_t ret = initialize_gpio(port, pin, GPIO_IN, GPIO_PUSH_PULL);
	if (ret != GPIO_OK) {
		return ret;
	}

	PORT_t *pPort = checkParamsAndGetPort(port);
	if (pPort == NULL) {
		return GPIO_NOT_FOUND_THE_PORT;
	}

	uint8_t pinMask = checkParamsAndGetPinMask(pin);
	if (pinMask == 0) {
		return GPIO_NOT_FOUNT_THE_PIN;
	}

	/* 12.13.14 PINnCTRL ? Pin n Control register */
	*pinCtrlReg = (*pinCtrlReg & ~(PORT_ISC_gm)) | mode;
	sCallback[port][pin] = callback;

	/* 12.13.10 INTCTRL ? Interrupt Control register */
	pPort->INTCTRL = (pPort->INTCTRL & ~(PORT_INTLVL_gm)) | PORT_INTLVL_HI_gc;

	/* 12.13.11 INTMASK ? Interrupt Mask register */
	pPort->INTMASK = pinMask;

	/* 11.8.3 CTRL ? Control register */
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;

	return GPIO_OK;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static PORT_t *checkParamsAndGetPort(GPIO_PORT port)
{
	PORT_t *pPort = NULL;

	/*J Port を見つける */
	if (port == GPIO_PORTA) {
		pPort = &PORTA;
	} else if (port == GPIO_PORTC) {
		pPort = &PORTC;
	} else if (port == GPIO_PORTD) {
		pPort = &PORTD;
	} else if (port == GPIO_PORTR) {
		pPort = &PORTR;	
	}
	return pPort;
}

/*---------------------------------------------------------------------------*/
static uint8_t checkParamsAndGetPinMask(GPIO_PIN pin)
{
	/*J Pin番号を確認 */
	if (!(0<=pin && pin<GPIO_PIN_MAX)) {
		return 0;
	}

	return (1 << pin);	
}

/*---------------------------------------------------------------------------*/
static register8_t *checkParamsAndGetPinControlReg(GPIO_PORT port, GPIO_PIN pin)
{
	PORT_t *pPort = checkParamsAndGetPort(port);
	if (pPort == NULL) {
		return NULL;
	}

	switch (pin) {
	case GPIO_PIN0:
		return &(pPort->PIN0CTRL);
		break;
	case GPIO_PIN1:
		return &(pPort->PIN1CTRL);
		break;
	case GPIO_PIN2:
		return &(pPort->PIN2CTRL);
		break;
	case GPIO_PIN3:
		return &(pPort->PIN3CTRL);
		break;
	case GPIO_PIN4:
		return &(pPort->PIN4CTRL);
		break;
	case GPIO_PIN5:
		return &(pPort->PIN5CTRL);
		break;
	case GPIO_PIN6:
		return &(pPort->PIN6CTRL);
		break;
	case GPIO_PIN7:
		return &(pPort->PIN7CTRL);
		break;
	default:
		break;
	}

	return NULL;
}



/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
ISR(PORTA_INT_vect)
{
	uint8_t flg = PORTA.INTFLAGS;
	PORTA.INTFLAGS = 0xff;

	uint8_t i;	
	for (i=0 ; i<8 ; ++i) {
		if ((BIT(i) & flg) && sCallback[GPIO_PORTA][i] != NULL) {
			sCallback[GPIO_PORTA][i]();
		}
	}

	return;
}


/*---------------------------------------------------------------------------*/
ISR(PORTC_INT_vect)
{
	uint8_t flg = PORTC.INTFLAGS;
	PORTC.INTFLAGS = 0xff;

	uint8_t i;
	for (i=0 ; i<8 ; ++i) {
		if ((BIT(i) & flg) && sCallback[GPIO_PORTC][i] != NULL) {
			sCallback[GPIO_PORTC][i]();
		}
	}

	return;
}


/*---------------------------------------------------------------------------*/
ISR(PORTD_INT_vect)
{
	uint8_t flg = PORTD.INTFLAGS;
	PORTD.INTFLAGS = 0xff;

	uint8_t i;
	for (i=0 ; i<8 ; ++i) {
		if ((BIT(i) & flg) && sCallback[GPIO_PORTD][i] != NULL) {
			sCallback[GPIO_PORTD][i]();
		}
	}

	return;
}