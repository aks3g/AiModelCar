/*
 * spi_driver.cpp
 *
 * Created: 2014/08/21 21:42:05
 *  Author: sazae7
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <xmegaE5/utils.h>
#include <xmegaE5/gpio_driver.h>
#include <xmegaE5/spi_driver.h>

// inline uint8_t spi_txrx_pio(uint8_t data);


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t initialize_spi_master(SPI_TYPE type, SPI_TRANSFER_MODE mode, SPI_DATA_ORDER order, SPI_CLK_SELECT clk)
{
	// 19.9.1 CTRL ? Control register
	// 7   CLK2X    - Double speed mode enable
	// 6   ENABLE   - Function Enable
	// 5   DORD     - Data Order
	// 4   MASTER   - Master/Slave selection
	// 3:2 MODE     - SPI Transfer mode(Group 0-3)
	// 1:0 PRESCALER- Clk prescaler

	//J クロック設定
	// Table 19-4. Relationship between SCK and the peripheral clock (ClkPER) frequency.
	uint8_t clkSetting = 0;
	switch (clk) {
	case SPI_CLK_DIV_2:
		clkSetting = (SPI_CLK2X_bm) | (SPI_PRESCALER_DIV4_gc);
		break;
	case SPI_CLK_DIV_4:
		clkSetting = (0x00)         | (SPI_PRESCALER_DIV4_gc);
		break;
	case SPI_CLK_DIV_8:
		clkSetting = (SPI_CLK2X_bm) | (SPI_PRESCALER_DIV16_gc);
		break;
	case SPI_CLK_DIV_16:
		clkSetting = (0x00)         | (SPI_PRESCALER_DIV16_gc);
		break;
	case SPI_CLK_DIV_32:
		clkSetting = (SPI_CLK2X_bm) | (SPI_PRESCALER_DIV64_gc);
		break;
	case SPI_CLK_DIV_64:
		clkSetting = (0x00)         | (SPI_PRESCALER_DIV64_gc);
		break;
	case SPI_CLK_DIV_128:
		clkSetting = (0x00)         | (SPI_PRESCALER_DIV128_gc);
		break;
	default:
		return SPI_INVALID_CLK;
	}

	//J モード設定
	uint8_t spiMode = 0;
	switch (mode) {
	case SPI_TRANSFER_MODE_0:
		spiMode = SPI_MODE_0_gc;
		break;
	case SPI_TRANSFER_MODE_1:
		spiMode = SPI_MODE_1_gc;
		break;
	case SPI_TRANSFER_MODE_2:
		spiMode = SPI_MODE_2_gc;
		break;
	case SPI_TRANSFER_MODE_3:
		spiMode = SPI_MODE_3_gc;
		break;
	default:
		return SPI_INVALID_MODE;
	}

	//J データ方向設定
	uint8_t dataOrder = 0;
	switch (order) {
	case SPI_DATA_ORDER_LSB_FIRST:
	case SPI_DATA_ORDER_MSB_FIRST:
		dataOrder = (order << SPI_DORD_bp) & SPI_DORD_bm;
		break;
	default:
		return SPI_INVALID_ORDER;
	}


	// 19.9.5 CTRLB ? Control register B
	// 7:6 BUFMODE  - OFF (00)
	// 5:3 reserved - 
	// 2   SSD      - Slave Select Disable
	// 1:0 reserved -
	uint8_t ctrlb = 0;
	
	//J 3線と4線の切り替え
	switch (type) {
	case SPI_TYPE_3WIRE:
	case SPI_TYPE_4WIRE:
		ctrlb |= (type << SPI_SSD_bp) & SPI_SSD_bm;
		break;
	default:
		return SPI_INVALID_TYPE;
	}


//	SPIC.CTRLB   = ctrlb;
	SPIC.CTRLB   = 0;
	SPIC.INTCTRL = SPI_INTLVL_OFF_gc | 0xF0;
	SPIC.CTRL    = SPI_ENABLE_bm | SPI_MASTER_bm | clkSetting | dataOrder | spiMode;
	
	return SPI_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t spi_tx(uint8_t data)
{
/*
	uint8_t input = 0xff;
	gpio_input(GPIO_PORTC, GPIO_PIN4, &input);
	if (input) {
		spi_txrx_pio(data);
		return SPI_OK;
	}
	
	gpio_output(GPIO_PORTD, GPIO_PIN1, 0);
*/
	//J データ書込み -> IF ビットが落ちる
	SPIC.DATA = data;

	//J コントローラの状態を確認
	// 19.9.3 STATUS . Status register
	// 7 IF - Interrupt Flag.
	while (!(SPIC.STATUS & SPI_IF_bm));
	
	return SPI_OK;
}


/*---------------------------------------------------------------------------*/
uint8_t spi_rx(void)
{
/*
	uint8_t input = 0xff;
	gpio_input(GPIO_PORTC, GPIO_PIN4, &input);
	if (input) {
		int ret =  spi_txrx_pio(0xFF);		
		return ret;
	}
*/

	//J データ書込み -> IF ビットが落ちる
	SPIC.DATA = 0xFF;

	//J コントローラの状態を確認
	// 19.9.3 STATUS . Status register
	// 7 IF - Interrupt Flag.
	while (!(SPIC.STATUS & SPI_IF_bm));

	//J 通信完了時にデータレジスタにはMISOからのバイナリが入るはず
	return SPIC.DATA;
}


/*---------------------------------------------------------------------------*/
uint8_t spi_txrx_pio(uint8_t data)
{
	uint8_t ctrl = SPIC.CTRL;
	uint8_t ret = 0;
	uint8_t input = 0xff;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);

	SPIC.CTRL = 0;

	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x80));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x80;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);
	
	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x40));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x40;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);

	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x20));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x20;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);

	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x10));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x10;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);

	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x08));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x08;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);

	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x04));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x04;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);

	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x02));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x02;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);

	gpio_output(GPIO_PORTC, GPIO_PIN7, (data & 0x01));
	gpio_input (GPIO_PORTC, GPIO_PIN6, &input);
	if (input) ret |= 0x01;
	gpio_output(GPIO_PORTC, GPIO_PIN5, 1);
	gpio_output(GPIO_PORTC, GPIO_PIN5, 0);
	
	SPIC.CTRL = ctrl;

	return ret;
}


