/*
 * bootloader_xmegaE5.cpp
 *
 * Created: 2014/11/15 13:23:14
 *  Author: sazae7
 */ 


#include <avr/io.h>
#include <util/crc16.h>

#include "avr_bootloader.h"
#include "flash_operation.h"

// ---------------------------------------------------------------------------
static uint8_t  eraseEepromAll(void);

static uint8_t  blockLoad(uint16_t size, uint8_t type, uint32_t *address);
static uint8_t  blockRead(uint16_t size, uint8_t type, uint32_t *address);

static uint8_t  readEepromByte (uint32_t address);
static uint8_t  writeEepromByte(uint32_t address, uint8_t val);

static uint8_t  getCrc16Block(uint8_t section, uint16_t *crc_out);


// ---------------------------------------------------------------------------
// Work Buffer
unsigned char buffer[SPM_PAGESIZE];



// ---------------------------------------------------------------------------
static void sendByte(uint8_t dat)
{
	while ( !(USARTD0.STATUS & USART_DREIF_bm) );
		
	USARTD0.DATA = dat;
	
	return;
}

static uint8_t getByte(void)
{
	while ( !(USARTD0.STATUS & USART_RXCIF_bm) );

	return USARTD0.DATA;
}

static uint16_t getWord(void)
{
	uint8_t hByte;
	uint8_t lByte;
	
	while ( !(USARTD0.STATUS & USART_RXCIF_bm) );
	hByte =  USARTD0.DATA;

	while ( !(USARTD0.STATUS & USART_RXCIF_bm) );
	lByte =  USARTD0.DATA;

	return ((uint16_t)hByte) << 8 | lByte;
}

static uint32_t get3Bytes(void)
{
	uint8_t hByte;
	uint8_t mByte;
	uint8_t lByte;
	
	while ( !(USARTD0.STATUS & USART_RXCIF_bm) );
	hByte =  USARTD0.DATA;
	
	while ( !(USARTD0.STATUS & USART_RXCIF_bm) );
	mByte =  USARTD0.DATA;

	while ( !(USARTD0.STATUS & USART_RXCIF_bm) );
	lByte =  USARTD0.DATA;

	return ((uint32_t)hByte) << 16 | ((uint32_t)mByte) << 8 | lByte;
}



// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
#define UART_BSEL_VALUE				(1047)
#define UART_BSCALE_VALUE			(-6)

// ---------------------------------------------------------------------------
static void initUart(void)
{
	/*J 20.18.7 BAUDCTRLA ? Baud Rate Control register A */
	/*J 20.18.8 BAUDCTRLB ? Baud Rate Control register B */
	USARTD0.BAUDCTRLA = (uint8_t)(UART_BSEL_VALUE & 0xff);
	USARTD0.BAUDCTRLB = (uint8_t)(((uint8_t)UART_BSCALE_VALUE << 4) & 0xff) | (uint8_t)((uint8_t)(UART_BSEL_VALUE>>8) & 0x0f);

	//J 安定するまで待つ
	volatile uint32_t i = 0;
	while(++i < 10000);


	/* 20.18.3 CTRLA ? Control register A */
	USARTD0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;

	/* 20.18.4 CTRLB ? Control register B */
	USARTD0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

	/* 20.18.5 CTRLC ? Control register C */
	USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	
	/* 20.18.6 CTRLD ? Control register D */
	USARTD0.CTRLD = 0x00;
		
	//J Port Setting
	PORTD.REMAP |= 0x10;
	PORTD.DIRSET = (1 << 7);	//J TXはピン7
	PORTD.DIRCLR = (1 << 6);	//J RXはピン6
	
	return;
}

// ---------------------------------------------------------------------------
int main(void)
{
	//J クロック設定
	OSC.CTRL |= OSC_RC32MEN_bm; // turn on 32 MHz oscillator
	while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { }; // wait for it to start
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
	
	//J UARTの設定
	initUart();

	uint8_t inBootloader = 1;
	uint32_t address = 0;
	uint8_t buf[3];
	uint8_t response[7];
	uint16_t writeVal16 = 0;

	//J UARTからデータ読み込んでは書く　の繰り返し
    while (inBootloader) {
		uint8_t  cmd = getByte();

		switch (cmd) {
		// -------------------------------------------------------------------
		// Bootloader command
		case CMD_ENTER_PROGRAMMING_MODE:
		case CMD_LEAVE_PROGRAMMING_MODE:
			{
				sendByte('\r'); // Send Ack
			}
			break;
		case CMD_SET_LED:
		case CMD_CLEAR_LED:
		case CMD_SELECT_DEVICE_TYPE:
			{
				//J 1Byte受信する -> 捨てる
				buf[0] = getByte();

				sendByte('\r'); // Send Ack
			}
			break;
		case CMD_EXIT_BOOTLOADER:
			{
				//J Exitする方法を考えるTODO
				inBootloader = 0;
				sendByte('\r'); // Send Ack
				volatile uint32_t wait = 100000;
				while (wait--);
				
			}
			break;

		// -------------------------------------------------------------------
		// Access to information
		case CMD_AUTO_INCREMENT_ADDRESS:
			{
				sendByte('Y'); // Send YES
			}
			break;
		case CMD_CHECK_BLOCK_SUPPORT:
			{
				sendByte('Y'); // Send YES
				
				response[0] = (SPM_PAGESIZE >> 8) & 0xff;
				response[1] = (SPM_PAGESIZE >> 0) & 0xff;

				sendByte(response[0]);
				sendByte(response[1]);
			}
			break;
		case CMD_RETURN_PROGRAMMER_TYPE:
			{
				sendByte('S');
			}
			break;
		case CMD_RETURN_SUPPORTED_DEVICE_CODES:
			{
				// sendByte(123);
				sendByte(0);
			}
			break;
		case CMD_RETURN_SOFTWARE_IDENTIFIER:
			{
				sendByte('A');
				sendByte('k');
				sendByte('s');
				sendByte('B');
				sendByte('o');
				sendByte('o');
				sendByte('t');
			}
			break;
		case CMD_RETURN_SOFTWARE_VERSION:
			{
				sendByte('0' + AKS_BOOT_VERSION_MAJOR);
				sendByte('0' + AKS_BOOT_VERSION_MINOR);
			}
			break;
		case CMD_READ_SIGNATURE_BYTES:
			{
				sendByte(SIGNATURE_2);
				sendByte(SIGNATURE_1);
				sendByte(SIGNATURE_0);
			}
			break;
		
		// -------------------------------------------------------------------
		// Addressing
		//J 2Byte Addressの読込
		case CMD_SET_ADDRESS:
			{
				//J 2Byte受信する
				address = getWord();
				
				sendByte('\r'); // Send Ack
			}
			break;
		//J 3Byte Addressの読込
		case CMD_SET_EXT_ADDRESS:
			{
				//J 3Byte受信する
				address = get3Bytes();

				sendByte('\r'); // Send Ack
			}
			break;

		// -------------------------------------------------------------------
		//J Acccess to program memory
		case CMD_READ_PROGRAM_MEMORY:
			{
				uint16_t val = readFlashWord((address << 1));
				address++;
				
				sendByte((val >> 8) & 0xff);
				sendByte((val >> 0) & 0xff);
			}
			break;
		case CMD_WRITE_PROGRAM_MEMORY_LOW_BYTE:
			{
				//J 1Byte受信する
				writeVal16 = getByte();
				
				sendByte('\r'); // Send Ack
			}
			break;
		case CMD_WRITE_PROGRAM_MEMORY_HIGH_BYTE:
			{
				//J 1Byte受信する
				writeVal16 |= (uint16_t)(getByte()) << 8;
				
				writeFlashWord(address<<1, writeVal16);
				address++;

				sendByte('\r'); // Send Ack
			}
			break;		
		case CMD_ISSUE_PAGE_WRITE:
			{
				if (address >= (APP_SECTION_SIZE >> 1)) {
					sendByte('?'); // Send ERROR
				} else {
					writeFlashApplicationPageStart(address<<1);

					sendByte('\r'); // Send Ack
				}
			}
			break;

		// -------------------------------------------------------------------
		// Chip Erase
		case CMD_CHIP_ERASE:
			{
				// Erase Application Section
				eraseFrashOnApplicationSection();
				waitForDoneFlashOperation();
				
				// Erase EEPROM
				eraseEepromAll();
				
				sendByte('\r'); // Send Ack
			}
			break;
			
		// -------------------------------------------------------------------
		// Block Access
		case CMD_START_BLOCK_LOAD:
			{
				//J 2Byte受信する
				uint16_t blockSize = getWord();
				
				//J 1Byte受信する
				uint8_t memoryType = getByte();
				
				//J Block loadを実行
				uint8_t ret = blockLoad(blockSize, memoryType, &address);
				
				//J Ackを返す
				sendByte(ret);
			}
			break;
		case CMD_START_BLOCK_READ:
			{
				//J 2Byte受信する
				uint16_t blockSize = getWord();
				
				//J 1Byte受信する
				uint8_t memoryType = getByte();
				
				//J Block loadを実行
				(void)blockRead(blockSize, memoryType, &address);
			}
			break;
		
		// -------------------------------------------------------------------
		// EEPROM Access
		case CMD_WRITE_EEPROM_BYTE:
			{
				//J 1Byte受信する
				buf[0] = getByte();
				writeEepromByte(address++, buf[0]);
				
				sendByte('\r'); // Send Ack
			}
			break;
		case CMD_READ_EEPROM_BYTE:
			{
				response[0] = readEepromByte(address++);				
				sendByte(response[0]);
			}
			break;
		
		// -------------------------------------------------------------------
		// Lock and Fuse Bits
		case CMD_WRITE_LOCK_BITS:
			{
				buf[0] = getByte();
				writeLockBit(buf[0]);

				sendByte('\r'); // Send Ack
			}
			break;
		case CMD_READ_LOCK_BITS:
			{
				response[0] = readLockBit();				
				sendByte(response[0]);
			}
			break;
		case CMD_READ_LOW_FUSE_BITS:
			{
				response[0] = readFuseByte(0);
				sendByte(response[0]);
			}
			break;
		case CMD_READ_HIGH_FUSE_BITS:
			{
				response[0] = readFuseByte(1);
				sendByte(response[0]);
			}
			break;
		case CMD_READ_EXTENDED_FUSE_BITS:
			{
				response[0] = readFuseByte(2);
				sendByte(response[0]);
			}
			break;
		
		// -------------------------------------------------------------------
		// CRC Support
		case CMD_CRC:
			{
				//J どのセクションのCRCを計算するかを調べる
				buf[0] = getByte();
				writeLockBit(buf[0]);
				
				//J CRCを計算
				uint16_t crc = 0;
				uint8_t ret = getCrc16Block(buf[0], &crc);
				if (ret == 0) {
					sendByte((crc >> 8) & 0xff);
					sendByte((crc >> 0) & 0xff);
				} else {
					sendByte('?'); // Send ERROR
					continue;
				}
			}
			break;

		// -------------------------------------------------------------------
		case CMD_I2C_AUTO_NEGOTIATION_START:
		case CMD_I2C_AUTO_NEGOTIATION_DONE:
		default:
			{
				sendByte('?'); // Send ERROR
			}
			break;
		case CMD_SYNC:
			{
				// DO Nothing ?
			}
			break;
		}
    }
	
	CCP = CCP_IOREG_gc;
	RST.CTRL = RST_SWRST_bm;
	while(1);
	
	// END
	return 1;
}


// ---------------------------------------------------------------------------
// NVM call From Internet...
static inline void NVM_EXEC(void)
{
	void *z = (void *)&NVM_CTRLA;
	
	__asm__ volatile("out %[ccp], %[ioreg]"  "\n\t"
	"st z, %[cmdex]"
	:
	: [ccp] "I" (_SFR_IO_ADDR(CCP)),
	[ioreg] "d" (CCP_IOREG_gc),
	[cmdex] "r" (NVM_CMDEX_bm),
	[z] "z" (z)
	);
}

// ---------------------------------------------------------------------------
static uint8_t  eraseEepromAll(void)
{
	//J Wait For NVM Process
	while (NVM.STATUS & NVM_NVMBUSY_bm);

	NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
	NVM_EXEC();
	
	return 0;
}


// ---------------------------------------------------------------------------
static void flushEepromNVMBuffer(void)
{
	//J Wait For NVM Process
	while (NVM.STATUS & NVM_NVMBUSY_bm);

	if ((NVM.STATUS & NVM_EELOAD_bm) != 0) {
		NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
		NVM_EXEC();
	}
}


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
#define NVM_CMD_READ_EEPROM							(0x06)
#define NVM_CMD_LOAD_EEPROM_BUFFER					(0x33)

// ---------------------------------------------------------------------------
static uint8_t  readEepromByte (uint32_t address)
{
	//J Wait For NVM Process
	while (NVM.STATUS & NVM_NVMBUSY_bm);

	NVM.ADDR0 = (address >> 8) & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0;

	NVM.CMD = NVM_CMD_READ_EEPROM;
	NVM_EXEC();

	return NVM.DATA0;
}

// ---------------------------------------------------------------------------
static uint8_t  writeEepromByte(uint32_t address, uint8_t val)
{
	flushEepromNVMBuffer();
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER;

	NVM.ADDR0 = (address >> 0) & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0;

	NVM.DATA0 = val;

	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
	NVM_EXEC();

	return 0;
}


// ---------------------------------------------------------------------------
static uint16_t writeEepromBlock(uint16_t addr, const uint8_t *src, uint16_t len)
{
	uint8_t  byte_addr = addr % EEPROM_PAGE_SIZE;
	uint16_t page_addr = addr - byte_addr;
	uint16_t cnt = 0;

	flushEepromNVMBuffer();

	//J Wait For NVM Process
	while (NVM.STATUS & NVM_NVMBUSY_bm);

	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER;

	//J 結局1Byteづつ書く
	NVM.ADDR1 = 0;
	NVM.ADDR2 = 0;
	while (len > 0) {
		NVM.ADDR0 = byte_addr;
		
		NVM.DATA0 = *(src++);
		
		byte_addr++;
		len--;
		
		if (len == 0 || byte_addr >= EEPROM_PAGE_SIZE) {
			NVM.ADDR0 = page_addr & 0xFF;
			NVM.ADDR1 = (page_addr >> 8) & 0x1F;
			
			NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
			NVM_EXEC();
			
			page_addr += EEPROM_PAGE_SIZE;
			byte_addr = 0;
			
			//J Wait For NVM Process
			while (NVM.STATUS & NVM_NVMBUSY_bm);

			NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER;
		}
		cnt++;
	}
	
	return cnt;
}

// ---------------------------------------------------------------------------
static uint16_t readEepromBlock(uint16_t address, uint8_t *dst, uint16_t len)
{
	uint16_t cnt = 0;

	NVM.ADDR2 = 0;

	//J Wait For NVM Process
	while (NVM.STATUS & NVM_NVMBUSY_bm);

	while (len > 0) {
		NVM.ADDR0 = (address >> 0) & 0xFF;
		NVM.ADDR1 = (address >> 8) & 0x1F;

		NVM.CMD = NVM_CMD_READ_EEPROM;
		NVM_EXEC();
		
		*(dst++) = NVM.DATA0; address++;
		
		len--; cnt++;
	}
	return cnt;
}


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
static uint8_t  blockLoad(uint16_t size, uint8_t type, uint32_t *address)
{
	uint16_t tempaddress = 0;
        
	//J バッファいっぱい受信(ただしSize分まで)
	for (uint16_t i = 0; i < SPM_PAGESIZE; i++) {
		char c = 0xff;
		if (i < size) {
			c = getByte();
		}
		buffer[i] = c;
	}

	// EEPROM のケース
	if (type == MEM_TYPE_EEPROM) {
		writeEepromBlock(*address, buffer, size);
		*address += size;
	        
		return '\r'; // Send ACK
	}
	//J フラッシュのケース
	else if (type == MEM_TYPE_FLASH) {
		// NOTE: For flash programming, 'address' is given in words.
	    tempaddress = (*address) << 1;  // Store address in page.
		*address += size >> 1;
	        
		programFlashPageWithErase(tempaddress, buffer, SPM_PAGESIZE, true);

		return '\r'; // Send ACK
	}
	else if (type == MEM_TYPE_USERSIG) {
		// NOTE: For flash programming, 'address' is given in words.
		tempaddress = (*address) << 1;  // Store address in page.
		*address += size >> 1;

		loadPageToFlashPageBuffer((uint16_t)buffer);
		eraseUserSignature();
		waitForDoneFlashOperation();
		writeUserSignature();
		waitForDoneFlashOperation();

		return '\r'; // Send ACK
	} else {
		// ?
	}

	//J 良く分からん時
	return '?'; // Send Error
}

// ---------------------------------------------------------------------------
static uint8_t  blockRead(uint16_t size, uint8_t type, uint32_t *address)
{
	int offset = 0;
	int size2  = size;
        
	//J EEPROM の場合
	if (type == MEM_TYPE_EEPROM) {
		
		readEepromBlock(*address, buffer, size2);
		*address += size2;
	}
	//J Flash の場合
	else if (type == MEM_TYPE_FLASH) {
		*address <<= 1;

		do {
			buffer[offset++] = readFlashByte(*address);

			waitForDoneFlashOperation();
			*address += 1;
			size2--;
		} while (size2);
		
		*address >>= 1;
	}
	//J ユーザ識別子の場合
	else if (type == MEM_TYPE_USERSIG) {
		*address <<= 1;

		do {
			buffer[offset++] = readUserSignature(*address);

			waitForDoneFlashOperation();
			*address += 1;
			size2--;
		} while (size2);
		
		*address >>= 1;
	}
	//J 製品識別子の場合
	else if (type == MEM_TYPE_PRODSIG) {
		*address <<= 1;

		do {
			buffer[offset++] = readCalibrationInfo(*address);

			waitForDoneFlashOperation();
			*address += 1;
			size2--;
		} while (size2);
		
		*address >>= 1;
	}	
	else {
		return -1;
	}


	//J データを数だけ送信
	for (uint16_t i = 0; i < size; i++) {
		sendByte(buffer[i]);
	}

	return 0;
}


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
static uint16_t crc16_block(uint32_t start, uint32_t length);


// ---------------------------------------------------------------------------
static uint8_t getCrc16Block(uint8_t section, uint16_t *crc_out)
{
	uint32_t start = 0;
	uint32_t length = 0;
	uint8_t  ret = 0;

	switch (section)
	{
	case CRC_SECTION_FLASH:
		start  = PROGMEM_START;
		length = PROGMEM_SIZE;
		break;
	case CRC_SECTION_APPLICATION:
		start  = APP_SECTION_START;
		length = APP_SECTION_SIZE;
		break;
	case CRC_SECTION_BOOT:
		start  = BOOT_SECTION_START;
		length = BOOT_SECTION_SIZE;
		break;
	default:
		ret = 'c'; // Continue
	}

	*crc_out = crc16_block(start, length);

	return ret;
}


// ---------------------------------------------------------------------------
static uint16_t crc16_block(uint32_t start, uint32_t length)
{
	uint16_t crc = 0;

	int bc = SPM_PAGESIZE;	
	for ( ; length > 0; length--) {
		if (bc == SPM_PAGESIZE) {
			readFlashPage(buffer, start);
			start += SPM_PAGESIZE;
			bc = 0;
		}
		
		crc = _crc16_update(crc, buffer[bc]);		
		bc++;
	}
	
	return crc;
}