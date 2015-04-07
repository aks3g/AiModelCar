/*
 * ioconfig.h
 *
 * Created: 2014/10/29 1:19:27
 *  Author: sazae7
 */ 


#ifndef IOCONFIG_H_
#define IOCONFIG_H_

#include <avr/io.h>
#include <gpio_driver.h>
#include <adc_driver.h>

/*****************************************************************************/
/* Port A */
/* PIN0 : AUX0 / GPIO Input */
#define AUX0_PORT					(GPIO_PORTA)
#define AUX0_PIN					(GPIO_PIN0)
#define AUX0_OUTPUT_DEFAULT			(1)
#define AUX0_IO_DIR					(GPIO_OUT)
#define AUX0_IO_MODE				(GPIO_PUSH_PULL)
#define ADC_CH_AUX0					(ADC_CH0)

/* PIN1 : AUX1 / GPIO Input */
#define AUX1_PORT					(GPIO_PORTA)
#define AUX1_PIN					(GPIO_PIN1)
#define AUX1_OUTPUT_DEFAULT			(1)
#define AUX1_IO_DIR					(GPIO_OUT)
#define AUX1_IO_MODE				(GPIO_PUSH_PULL)
#define ADC_CH_AUX1					(ADC_CH1)

/* PIN2 : AUX2 / GPIO Input */
#define AUX2_PORT					(GPIO_PORTA)
#define AUX2_PIN					(GPIO_PIN2)
#define AUX2_OUTPUT_DEFAULT			(1)
#define AUX2_IO_DIR					(GPIO_OUT)
#define AUX2_IO_MODE				(GPIO_PUSH_PULL)
#define ADC_CH_AUX2					(ADC_CH2)

/* PIN3 : AUX3 / GPIO Input */
#define AUX3_PORT					(GPIO_PORTA)
#define AUX3_PIN					(GPIO_PIN3)
#define AUX3_OUTPUT_DEFAULT			(1)
#define AUX3_IO_DIR					(GPIO_OUT)
#define AUX3_IO_MODE				(GPIO_PUSH_PULL)
#define ADC_CH_AUX3					(ADC_CH3)

/* PIN4 : Reserved / GPIO Output / Hi */
#define RSVD_PORT					(GPIO_PORTA)
#define RSVD_PIN					(GPIO_PIN4)
#define RSVD_OUTPUT_DEFAULT			(1)
#define RSVD_IO_DIR					(GPIO_OUT)
#define RSVD_IO_MODE				(GPIO_PUSH_PULL)

/* PIN5 : Bat sens / ADC */
#define BAT_SENS_PORT				(GPIO_PORTA)
#define BAT_SENS_PIN				(GPIO_PIN5)
#define BAT_SENS_OUTPUT_DEFAULT		(1)
#define BAT_SENS_IO_DIR				(GPIO_IN)
#define BAT_SENS_IO_MODE			(GPIO_PUSH_PULL)
#define ADC_CH_BAT_SENS				(ADC_CH5)

/* PIN6 : Bus sens / ADC */
#define BUS_SENS_PORT				(GPIO_PORTA)
#define BUS_SENS_PIN				(GPIO_PIN6)
#define BUS_SENS_OUTPUT_DEFAULT		(1)
#define BUS_SENS_IO_DIR				(GPIO_IN)
#define BUS_SENS_IO_MODE			(GPIO_PUSH_PULL)
#define ADC_CH_BUS_SENS				(ADC_CH6)

/* PIN7 : MOTOR_ON / GPIO Output / Lo */
#define MOTOR_ON_PORT				(GPIO_PORTA)
#define MOTOR_ON_PIN				(GPIO_PIN7)
#define MOTOR_ON_OUTPUT_DEFAULT		(0)
#define MOTOR_ON_IO_DIR				(GPIO_OUT)
#define MOTOR_ON_IO_MODE			(GPIO_PUSH_PULL)



/*****************************************************************************/
/* Port C */
/* PIN0 : SDA / I2C */
#define SDA_PORT					(GPIO_PORTC)
#define SDA_PIN						(GPIO_PIN0)
#define SDA_OUTPUT_DEFAULT			(1)
#define SDA_IO_DIR					(GPIO_OUT)
#define SDA_IO_MODE					(GPIO_OPEN_DRAIN)

/* PIN1 : SCL / I2C */
#define SCL_PORT					(GPIO_PORTC)
#define SCL_PIN						(GPIO_PIN1)
#define SCL_OUTPUT_DEFAULT			(1)
#define SCL_IO_DIR					(GPIO_OUT)
#define SCL_IO_MODE					(GPIO_PUSH_PULL)

/* PIN2 : INT1# / GPIO Input */
#define nINT1_PORT					(GPIO_PORTC)
#define nINT1_PIN					(GPIO_PIN2)
#define nINT1_OUTPUT_DEFAULT		(1)
#define nINT1_IO_DIR				(GPIO_IN)
#define nINT1_IO_MODE				(GPIO_PUSH_PULL)

/* PIN3 : INT2# / GPIO Input */
#define nINT2_PORT					(GPIO_PORTC)
#define nINT2_PIN					(GPIO_PIN3)
#define nINT2_OUTPUT_DEFAULT		(1)
#define nINT2_IO_DIR				(GPIO_IN)
#define nINT2_IO_MODE				(GPIO_PUSH_PULL)

/* PIN4 : SS# / SPI */
#define nSS_PORT					(GPIO_PORTC)
#define nSS_PIN						(GPIO_PIN4)
#define nSS_OUTPUT_DEFAULT			(1)
#define nSS_IO_DIR					(GPIO_OUT)
#define nSS_IO_MODE					(GPIO_PUSH_PULL)

/* PIN5 : SCLK / SPI */
#define SCLK_PORT					(GPIO_PORTC)
#define SCLK_PIN					(GPIO_PIN5)
#define SCLK_OUTPUT_DEFAULT			(0)
#define SCLK_IO_DIR					(GPIO_OUT)
#define SCLK_IO_MODE				(GPIO_PUSH_PULL)

/* PIN6 : MISO / SPI */
#define MISO_PORT					(GPIO_PORTC)
#define MISO_PIN					(GPIO_PIN6)
#define MISO_OUTPUT_DEFAULT			(0)
#define MISO_IO_DIR					(GPIO_IN)
#define MISO_IO_MODE				(GPIO_PUSH_PULL)

/* PIN7 : MOSI /SPI */
#define MOSI_PORT					(GPIO_PORTC)
#define MOSI_PIN					(GPIO_PIN7)
#define MOSI_OUTPUT_DEFAULT			(1)
#define MOSI_IO_DIR					(GPIO_OUT)
#define MOSI_IO_MODE				(GPIO_PUSH_PULL)


/*****************************************************************************/
/* Port D */
/* PIN0 : BAT_CHG# / GPIO Input */
#define nBAT_CHG_PORT				(GPIO_PORTD)
#define nBAT_CHG_PIN				(GPIO_PIN0)
#define nBAT_CHG_OUTPUT_DEFAULT		(1)
#define nBAT_CHG_IO_DIR				(GPIO_IN)
#define nBAT_CHG_IO_MODE			(GPIO_OPEN_DRAIN)

/* PIN1 : DEBUG_LED0# / GPIO Output */
#define nDEBUG_LED0_PORT			(GPIO_PORTD)
#define nDEBUG_LED0_PIN				(GPIO_PIN1)
#define nDEBUG_LED0_OUTPUT_DEFAULT	(1)
#define nDEBUG_LED0_IO_DIR			(GPIO_OUT)
#define nDEBUG_LED0_IO_MODE			(GPIO_PUSH_PULL)


/* PIN2 : CHG_LED1# / GPIO Output */
#define nCHG_LED_PORT				(GPIO_PORTD)
#define nCHG_LED_PIN				(GPIO_PIN2)
#define nCHG_LED_OUTPUT_DEFAULT		(1)
#define nCHG_LED_IO_DIR				(GPIO_OUT)
#define nCHG_LED_IO_MODE			(GPIO_PUSH_PULL)

/* PIN3 : STATUS_LED# / GPIO Output */
#define nSTATUS_LED_PORT			(GPIO_PORTD)
#define nSTATUS_LED_PIN				(GPIO_PIN3)
#define nSTATUS_LED_OUTPUT_DEFAULT	(1)
#define nSTATUS_LED_IO_DIR			(GPIO_OUT)
#define nSTATUS_LED_IO_MODE			(GPIO_PUSH_PULL)

/* PIN4 : DEBUG_LED1# / GPIO Output */
#define nDEBUG_LED1_PORT			(GPIO_PORTD)
#define nDEBUG_LED1_PIN				(GPIO_PIN4)
#define nDEBUG_LED1_OUTPUT_DEFAULT	(1)
#define nDEBUG_LED1_IO_DIR			(GPIO_OUT)
#define nDEBUG_LED1_IO_MODE			(GPIO_PUSH_PULL)

/* PIN5 : SW2# / GPIO Input */
#define nSW2_PORT					(GPIO_PORTD)
#define nSW2_PIN					(GPIO_PIN5)
#define nSW2_OUTPUT_DEFAULT			(0)
#define nSW2_IO_DIR					(GPIO_IN)
#define nSW2_IO_MODE				(GPIO_PUSH_PULL)

/* PIN6 : RXD / UART */
#define RXD_PORT					(GPIO_PORTD)
#define RXD_PIN						(GPIO_PIN6)
#define RXD_OUTPUT_DEFAULT			(0)
#define RXD_IO_DIR					(GPIO_IN)
#define RXD_IO_MODE					(GPIO_PUSH_PULL)

/* PIN7 : TXD / UART */
#define TXD_PORT					(GPIO_PORTD)
#define TXD_PIN						(GPIO_PIN7)
#define TXD_OUTPUT_DEFAULT			(1)
#define TXD_IO_DIR					(GPIO_OUT)
#define TXD_IO_MODE					(GPIO_PUSH_PULL)


/*****************************************************************************/
/* Port R */
/* PIN0 : SW1# / GPIO Input */
#define nSW1_PORT					(GPIO_PORTR)
#define nSW1_PIN					(GPIO_PIN0)
#define nSW1_OUTPUT_DEFAULT			(0)
#define nSW1_IO_DIR					(GPIO_IN)
#define nSW1_IO_MODE				(GPIO_PUSH_PULL)

/* PIN1 : SW0# / GPIO Input */
#define nSW0_PORT					(GPIO_PORTR)
#define nSW0_PIN					(GPIO_PIN1)
#define nSW0_OUTPUT_DEFAULT			(0)
#define nSW0_IO_DIR					(GPIO_IN)
#define nSW0_IO_MODE				(GPIO_PUSH_PULL)


int8_t init_gpio(void);

#endif /* IOCONFIG_H_ */