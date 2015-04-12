/*
 * ioconfig.cpp
 *
 * Created: 2014/10/29 1:22:21
 *  Author: sazae7
 */ 
#include <avr/io.h>

#include <stdint.h>

#include <gpio_driver.h>
#include "ioconfig.h"

int8_t init_gpio(void)
{
	/* Port Configuration */
	/* Port A */
	initialize_gpio(AUX0_PORT,        AUX0_PIN,        AUX0_IO_DIR,        AUX0_IO_MODE);
	initialize_gpio(AUX1_PORT,        AUX1_PIN,        AUX1_IO_DIR,        AUX1_IO_MODE);
	initialize_gpio(AUX2_PORT,        AUX2_PIN,        AUX2_IO_DIR,        AUX2_IO_MODE);
	initialize_gpio(AUX3_PORT,        AUX3_PIN,        AUX3_IO_DIR,        AUX3_IO_MODE);
	initialize_gpio(RSVD_PORT,        RSVD_PIN,        RSVD_IO_DIR,        RSVD_IO_MODE);
	initialize_gpio(BAT_SENS_PORT,    BAT_SENS_PIN,    BAT_SENS_IO_DIR,    BAT_SENS_IO_MODE);
	initialize_gpio(BUS_SENS_PORT,    BUS_SENS_PIN,    BUS_SENS_IO_DIR,    BUS_SENS_IO_MODE);
	initialize_gpio(MOTOR_ON_PORT,    MOTOR_ON_PIN,    MOTOR_ON_IO_DIR,    MOTOR_ON_IO_MODE);
	
	/* Port C */
	initialize_gpio(SDA_PORT,         SDA_PIN,         SDA_IO_DIR,         SDA_IO_MODE);
	initialize_gpio(SCL_PORT,         SCL_PIN,         SCL_IO_DIR,         SCL_IO_MODE);
	initialize_gpio(nINT1_PORT,       nINT1_PIN,       nINT1_IO_DIR,       nINT1_IO_MODE);
	initialize_gpio(nINT2_PORT,       nINT2_PIN,       nINT2_IO_DIR,       nINT2_IO_MODE);
	initialize_gpio(nSS_PORT,         nSS_PIN,         nSS_IO_DIR,         nSS_IO_MODE);
	initialize_gpio(SCLK_PORT,        SCLK_PIN,        SCLK_IO_DIR,        SCLK_IO_MODE);
	initialize_gpio(MISO_PORT,        MISO_PIN,        MISO_IO_DIR,        MISO_IO_MODE);
	initialize_gpio(MOSI_PORT,        MOSI_PIN,        MOSI_IO_DIR,        MOSI_IO_MODE);

	/* Port D */
	initialize_gpio(nBAT_CHG_PORT,    nBAT_CHG_PIN,    nBAT_CHG_IO_DIR,    nBAT_CHG_IO_MODE);
	initialize_gpio(nSTATUS_LED_PORT, nSTATUS_LED_PIN, nSTATUS_LED_IO_DIR, nSTATUS_LED_IO_MODE);
	initialize_gpio(nDEBUG_LED1_PORT, nDEBUG_LED1_PIN, nDEBUG_LED1_IO_DIR, nDEBUG_LED1_IO_MODE);
	initialize_gpio(nDEBUG_LED0_PORT, nDEBUG_LED0_PIN, nDEBUG_LED0_IO_DIR, nDEBUG_LED0_IO_MODE);
	initialize_gpio(nCHG_LED_PORT,    nCHG_LED_PIN,    nCHG_LED_IO_DIR,    nCHG_LED_IO_MODE);
	initialize_gpio(nSW2_PORT,        nSW2_PIN,        nSW2_IO_DIR,        nSW2_IO_MODE);
	initialize_gpio(RXD_PORT,         RXD_PIN,         RXD_IO_DIR,         RXD_IO_MODE);
	initialize_gpio(TXD_PORT,         TXD_PIN,         TXD_IO_DIR,         TXD_IO_MODE);

	/* Port R */
	initialize_gpio(nSW1_PORT,        nSW1_PIN,        nSW1_IO_DIR,        nSW1_IO_MODE);
	initialize_gpio(nSW0_PORT,        nSW0_PIN,        nSW0_IO_DIR,        nSW0_IO_MODE);

	/** Default Output Level */
	/* Port A */
	gpio_output(AUX0_PORT,        AUX0_PIN,        AUX0_OUTPUT_DEFAULT);
	gpio_output(AUX1_PORT,        AUX1_PIN,        AUX1_OUTPUT_DEFAULT);
	gpio_output(AUX2_PORT,        AUX2_PIN,        AUX2_OUTPUT_DEFAULT);
	gpio_output(AUX3_PORT,        AUX3_PIN,        AUX3_OUTPUT_DEFAULT);
	gpio_output(RSVD_PORT,        RSVD_PIN,        RSVD_OUTPUT_DEFAULT);
	gpio_output(BAT_SENS_PORT,    BAT_SENS_PIN,    BAT_SENS_OUTPUT_DEFAULT);
	gpio_output(BUS_SENS_PORT,    BUS_SENS_PIN,    BUS_SENS_OUTPUT_DEFAULT);
	gpio_output(MOTOR_ON_PORT,    MOTOR_ON_PIN,    MOTOR_ON_OUTPUT_DEFAULT);
	
	/* Port C */
	gpio_output(SDA_PORT,         SDA_PIN,         SDA_OUTPUT_DEFAULT);
	gpio_output(SCL_PORT,         SCL_PIN,         SCL_OUTPUT_DEFAULT);
	gpio_output(nINT1_PORT,       nINT1_PIN,       nINT1_OUTPUT_DEFAULT);
	gpio_output(nINT2_PORT,       nINT2_PIN,       nINT2_OUTPUT_DEFAULT);
	gpio_output(nSS_PORT,         nSS_PIN,         nSS_OUTPUT_DEFAULT);
	gpio_output(SCLK_PORT,        SCLK_PIN,        SCLK_OUTPUT_DEFAULT);
	gpio_output(MISO_PORT,        MISO_PIN,        MISO_OUTPUT_DEFAULT);
	gpio_output(MOSI_PORT,        MOSI_PIN,        MOSI_OUTPUT_DEFAULT);

	/* Port D */
	gpio_output(nBAT_CHG_PORT,    nBAT_CHG_PIN,    nBAT_CHG_OUTPUT_DEFAULT);
	gpio_output(nSTATUS_LED_PORT, nSTATUS_LED_PIN, nSTATUS_LED_OUTPUT_DEFAULT);
	gpio_output(nDEBUG_LED1_PORT, nDEBUG_LED1_PIN, nDEBUG_LED1_OUTPUT_DEFAULT);
	gpio_output(nDEBUG_LED0_PORT, nDEBUG_LED0_PIN, nDEBUG_LED0_OUTPUT_DEFAULT);
	gpio_output(nCHG_LED_PORT,    nCHG_LED_PIN,    nCHG_LED_OUTPUT_DEFAULT);
	gpio_output(nSW2_PORT,        nSW2_PIN,        nSW2_OUTPUT_DEFAULT);
	gpio_output(RXD_PORT,         RXD_PIN,         RXD_OUTPUT_DEFAULT);
	gpio_output(TXD_PORT,         TXD_PIN,         TXD_OUTPUT_DEFAULT);

	/* Port R */
	gpio_output(nSW1_PORT,        nSW1_PIN,        nSW1_OUTPUT_DEFAULT);
	gpio_output(nSW0_PORT,        nSW0_PIN,        nSW0_OUTPUT_DEFAULT);


	// TODO
	//J MISO‚ðPullUP ‰B‚Ø‚¢‚µ‚½‚¢
	PORTC.PIN6CTRL = (PORTC.PIN6CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;

	return 0;
}