/*
 * uart_driver.cpp
 *
 * Created: 2013/09/12 0:19:44
 *  Author: sazae7
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "utils.h"
#include "gpio_driver.h"
#include "uart_driver.h"

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#define UART_QUEUE_SIZE		(0x7F)
#define UART_QUEUE_BUF_SIZE	(0x80)
#define UART_QUEUE_MASK		(0x7F)

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
USART_t *sUsart = NULL;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
typedef struct {
	uint8_t wptr;
	uint8_t rptr;
	
	uint8_t buf[UART_QUEUE_BUF_SIZE];
} UART_QUEUE;


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static UART_QUEUE sTxQueue;
static UART_QUEUE sRxQueue;
static volatile uint8_t sTxWorking = 0;


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#define QUEUE_SIZE_OF(q)	((((q)->wptr + UART_QUEUE_BUF_SIZE) - (q)->rptr) &  UART_QUEUE_MASK)
#define QUEUE_FULL(q)		(QUEUE_SIZE_OF(q) == (UART_QUEUE_SIZE))
#define ABS(v)				((v)<0 ? -(v) : (v))

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t sEnqueue(UART_QUEUE *queue, uint8_t dat);
static uint8_t sEnqueueAll(UART_QUEUE *queue, uint8_t *dat, uint32_t len);
static uint8_t sDequeue(UART_QUEUE *queue, uint8_t *dat);
static uint32_t sPower(uint32_t base, uint32_t power);


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t initialize_uart(uint8_t usartType, uint32_t baudrate, uint32_t peripheralClock)
{
	if (sUsart != NULL) {
		return UART_IS_ALREADY_INITIALIZED;
	}

	/* どのポートを初期化するか確認 */
	if (usartType == USART0_ON_PORTC_0 || usartType == USART0_ON_PORTC_1) {
		sUsart = &USARTC0;		
	} else if (usartType == USART0_ON_PORTD_0 || usartType == USART0_ON_PORTD_1) {
		sUsart = &USARTD0;
	} else {
		return UART_INVALID_UART_TYPE;
	}
	
	/*J 送受信バッファを初期化 */
	sTxQueue.rptr = 0;
	sTxQueue.wptr = 0;
	sRxQueue.rptr = 0;
	sRxQueue.wptr = 0;
	
	/* J BaudRateに対応した設定を作る */
	/* Table 20-1. Equations for calculating Baud Rate Register setting. */
	volatile uint32_t baudrateBits = 0; /* 12bit value */
	volatile int8_t baudrateScaleFactor = -8; /* -7 to 7 */
	
	for (int8_t i=-7 ; i <= 7 ; ++i ){
		if (i<0) {
			baudrateBits = sPower(2, ABS(i)) * (((float)peripheralClock / (16.0*(float)baudrate)) - 1.0);
		} else {
			baudrateBits = ((float)peripheralClock / (float)sPower(2, ABS(i)) * (16.0*(float)baudrate)) - 1;
		}

		if (baudrateBits < 0x0fff) {
			baudrateScaleFactor = i;
			break;
		} else {
			
		}
	}

	/*J 指定されたBaudrateに設定できなかった */
	if (baudrateScaleFactor == -8) {
		return UART_INVALID_BAUDRATE;
	}

	/*J 20.18.7 BAUDCTRLA ? Baud Rate Control register A */
	/*J 20.18.8 BAUDCTRLB ? Baud Rate Control register B */
	sUsart->BAUDCTRLA = (uint8_t)(baudrateBits & 0xff);
	sUsart->BAUDCTRLB = (((uint8_t)baudrateScaleFactor) << 4) | (uint8_t)((baudrateBits>>8) & 0x0f);

	//J 安定するまで待つ
	volatile uint32_t i = 0;
	while(++i < 10000);


	/* 20.18.3 CTRLA ? Control register A */
	sUsart->CTRLA = USART_RXCINTLVL_HI_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;

	/* 20.18.4 CTRLB ? Control register B */
	sUsart->CTRLB = USART_RXEN_bm | USART_TXEN_bm;

	/* 20.18.5 CTRLC ? Control register C */
	sUsart->CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	
	/* 20.18.6 CTRLD ? Control register D */
	sUsart->CTRLD = 0x00;

	/*J ポートの設定 */
	if (usartType == USART0_ON_PORTC_0) {
		initialize_gpio(GPIO_PORTC, GPIO_PIN2, GPIO_IN, GPIO_PUSH_PULL);
		initialize_gpio(GPIO_PORTC, GPIO_PIN3, GPIO_OUT, GPIO_PUSH_PULL);
	}
	else if (usartType == USART0_ON_PORTC_1) {
		PORTC.REMAP |= 0x10;
		initialize_gpio(GPIO_PORTC, GPIO_PIN6, GPIO_IN, GPIO_PUSH_PULL);
		initialize_gpio(GPIO_PORTC, GPIO_PIN7, GPIO_OUT, GPIO_PUSH_PULL);
	}
	else if (usartType == USART0_ON_PORTD_0) {
		initialize_gpio(GPIO_PORTD, GPIO_PIN2, GPIO_IN, GPIO_PUSH_PULL);
		initialize_gpio(GPIO_PORTD, GPIO_PIN3, GPIO_OUT, GPIO_PUSH_PULL);
	}
	else if (usartType == USART0_ON_PORTD_1) {
		PORTD.REMAP |= 0x10;
		initialize_gpio(GPIO_PORTD, GPIO_PIN6, GPIO_IN, GPIO_PUSH_PULL);
		initialize_gpio(GPIO_PORTD, GPIO_PIN7, GPIO_OUT, GPIO_PUSH_PULL);
	}

	return UART_OK;
}

/*---------------------------------------------------------------------------*/
uint8_t uart_tx(uint8_t *dat, uint32_t len)
{
	if (sUsart == NULL) {
		return UART_INVALID_UART_TYPE;
	}
	
	Disable_Int();

	sEnqueueAll(&sTxQueue, dat, len);

	if ( !sTxWorking ) {
		sTxWorking = 1; 
		uint8_t d = 0;
		uint8_t ret = sDequeue(&sTxQueue, &d);
		if (ret == UART_OK) {
			sUsart->STATUS |= USART_TXCIF_bm; //Clear!
			sUsart->CTRLA  |= USART_TXCINTLVL_MED_gc;
//			sUsart->CTRLA  |= USART_DREINTLVL_HI_gc;

			sUsart->DATA = d;
		}		
	}

	Enable_Int();

	return UART_OK;
}

/*---------------------------------------------------------------------------*/
uint8_t uart_txNonBuffer(uint8_t *dat, uint32_t len)
{
	if (sUsart == NULL) {
		return UART_INVALID_UART_TYPE;
	}

	/*J 割込み駆動のTxが動いている間は処理を止める */
	while (sTxWorking);

	if (len == 0) {
		return UART_OK;
	}
	
	if (dat == NULL) {
		return UART_BUFFER_IS_NULL;
	}
	
	while (len--) {
		while ( !(sUsart->STATUS & USART_DREIF_bm) );
		
		sUsart->DATA = *dat++;
	}

	return UART_OK;
}

/*---------------------------------------------------------------------------*/
uint8_t uart_rx(uint8_t *dat, uint32_t *len)
{
	*len = QUEUE_SIZE_OF(&sRxQueue);
	
	for (uint32_t i=0 ; i<*len ; ++i) {
		uint8_t d=0;
		uint8_t ret = sDequeue(&sRxQueue, &d);
		if (ret != UART_OK) {
			*len = i;
			break;
		}

		*dat++ = d;
	}
	
	return UART_OK;	
}

/*---------------------------------------------------------------------------*/
uint8_t uart_rx_n(uint8_t *dat, uint32_t rlen, uint32_t *len)
{
	*len = QUEUE_SIZE_OF(&sRxQueue);
	*len = ((*len) > rlen) ? rlen : (*len); 
	
	for (uint32_t i=0 ; i<*len ; ++i) {
		uint8_t d=0;
		uint8_t ret = sDequeue(&sRxQueue, &d);
		if (ret != UART_OK) {
			*len = i;
			break;
		}

		*dat++ = d;
	}
	
	return UART_OK;
}

/*---------------------------------------------------------------------------*/
uint8_t uart_rxBlocking(uint8_t *dat, uint32_t rlen)
{
	while (uart_get_rxlen() < (volatile uint32_t)rlen);
	
	uint32_t len = 0;
	return uart_rx_n(dat, rlen, &len);
}


/*---------------------------------------------------------------------------*/
uint32_t uart_get_rxlen(void)
{
	return QUEUE_SIZE_OF(&sRxQueue);
}

/*---------------------------------------------------------------------------*/
static uint8_t sEnqueue(UART_QUEUE *queue, uint8_t dat)
{
	if (QUEUE_FULL(queue)) {
		return UART_BUFFER_FULL;
	}

	queue->buf[queue->wptr++] = dat;
	queue->wptr = queue->wptr & UART_QUEUE_MASK;

	return UART_OK;
}


/*---------------------------------------------------------------------------*/
static uint8_t sEnqueueAll(UART_QUEUE *queue, uint8_t *dat, uint32_t len)
{
	if ((UART_QUEUE_SIZE - QUEUE_SIZE_OF(queue)) < len) {
		return UART_NOT_ENOUGH_BUFFER;
	}

	for (uint32_t i=0 ; i<len ; ++i) {
		queue->buf[queue->wptr++] = *dat++;	
		queue->wptr = queue->wptr & UART_QUEUE_MASK;
	}
	
	return UART_OK;
}


/*---------------------------------------------------------------------------*/
static uint8_t sDequeue(UART_QUEUE *queue, uint8_t *dat)
{
	if (QUEUE_SIZE_OF(queue) == 0) {
		return UART_BUFFER_EMPTY;
	}

	*dat = queue->buf[queue->rptr++];
	queue->rptr = queue->rptr & UART_QUEUE_MASK;

	return UART_OK;
}


/*---------------------------------------------------------------------------*/
static uint32_t sPower(uint32_t base, uint32_t power)
{
	uint32_t ret = 1;

	for (uint32_t i=0 ; i<power ; ++i) {
		ret *= base;
	}
	
	return ret;
}


/*---------------------------------------------------------------------------*/
ISR(USARTC0_RXC_vect)
{
	//J 溢れたって、ねぇ・・・。
	sEnqueue(&sRxQueue, USARTC0.DATA);

	return;
}

/*---------------------------------------------------------------------------*/
ISR(USARTC0_TXC_vect)
{
	if (QUEUE_SIZE_OF(&sTxQueue) != 0) {
		uint8_t d=0;
		//J エラーが帰ったって、ねぇ・・・。
		sDequeue(&sTxQueue, &d);

		sTxWorking = 1;
		USARTC0.DATA = d;
	} else {
//		USARTC0.STATUS |= USART_TXCIF_bm; //Clear!
		USARTC0.CTRLA &= ~USART_TXCINTLVL_gm;
		
		sTxWorking = 0;
	}

	return;
}

/*---------------------------------------------------------------------------*/
ISR(USARTC0_DRE_vect)
{
	if (QUEUE_SIZE_OF(&sTxQueue) != 0) {
		uint8_t d=0;
		//J エラーが帰ったって、ねぇ・・・。
		sDequeue(&sTxQueue, &d);

		sTxWorking = 1;
		USARTC0.DATA = d;
	} else {
		USARTC0.CTRLA &= ~USART_DREINTLVL_gm;
		sTxWorking = 0;
	}

	return;
}


/*---------------------------------------------------------------------------*/
ISR(USARTD0_RXC_vect)
{
	//J 溢れたって、ねぇ・・・。
	sEnqueue(&sRxQueue, USARTD0.DATA);

	return;
}

/*---------------------------------------------------------------------------*/
ISR(USARTD0_TXC_vect)
{
	if (QUEUE_SIZE_OF(&sTxQueue) > 0) {
		uint8_t d=0;
		//J エラーが帰ったって、ねぇ・・・。
		sDequeue(&sTxQueue, &d);

		sTxWorking = 1;
		USARTD0.DATA = d;
	} else {
//		USARTD0.STATUS |= USART_TXCIF_bm; //Clear!
		USARTD0.CTRLA &= ~USART_TXCINTLVL_gm;
		
		sTxWorking = 0;
	}

	return;
}

/*---------------------------------------------------------------------------*/
ISR(USARTD0_DRE_vect)
{
	if (QUEUE_SIZE_OF(&sTxQueue) != 0) {
		uint8_t d=0;
		//J エラーが帰ったって、ねぇ・・・。
		sDequeue(&sTxQueue, &d);

		sTxWorking = 1;
		USARTD0.DATA = d;
	} else {
		USARTD0.CTRLA &= ~USART_DREINTLVL_gm;
		sTxWorking = 0;
	}

	return;
}
