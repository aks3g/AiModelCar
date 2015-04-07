/*
 * avr_bootloader.h
 *
 * Created: 2014/11/16 20:51:53
 *  Author: sazae7
 */ 


#ifndef AVR_BOOTLOADER_H_
#define AVR_BOOTLOADER_H_
// -------------------------------------------------------------------
#define AKS_BOOT_VERSION_MAJOR				0
#define AKS_BOOT_VERSION_MINOR				1

#define AKS_BOOT_SIGNATURE0					0x0A
#define AKS_BOOT_SIGNATURE1					0x0A
#define AKS_BOOT_SIGNATURE2					0x0A



// -------------------------------------------------------------------
// Memory types to Block Access
#define MEM_TYPE_EEPROM						('E')
#define MEM_TYPE_FLASH						('F')
#define MEM_TYPE_USERSIG					('U')
#define MEM_TYPE_PRODSIG					('P')

// CRC Section Selection
#define CRC_SECTION_FLASH					('F')
#define CRC_SECTION_APPLICATION				('A')
#define CRC_SECTION_BOOT					('B')
#define CRC_SECTION_APP						('a')
#define CRC_SECTION_APP_TEMP				('t')

// -------------------------------------------------------------------
//J Bootloader Command 
#define CMD_ENTER_PROGRAMMING_MODE			('P')
#define CMD_LEAVE_PROGRAMMING_MODE			('L')
#define CMD_SET_LED							('x')
#define CMD_CLEAR_LED						('y')
#define CMD_SELECT_DEVICE_TYPE				('T')
#define CMD_EXIT_BOOTLOADER					('E')

//J Access to Information
#define CMD_AUTO_INCREMENT_ADDRESS			('a')
#define CMD_CHECK_AUTO_INCREMENT			('a')	//J Extra
#define CMD_READ_SIGNATURE_BYTES			('s')
#define CMD_RETURN_PROGRAMMER_TYPE			('p')
#define CMD_RETURN_SOFTWARE_IDENTIFIER		('S')
#define CMD_RETURN_SOFTWARE_VERSION			('V')
#define CMD_RETURN_SUPPORTED_DEVICE_CODES	('t')
#define CMD_CHECK_BLOCK_SUPPORT				('b')

//J Addressing
#define CMD_SET_ADDRESS						('A')
#define CMD_SET_EXT_ADDRESS					('H')	//J Extra

//J Program Memory Access
#define CMD_READ_PROGRAM_MEMORY				('R')
#define CMD_WRITE_PROGRAM_MEMORY_LOW_BYTE	('c')
#define CMD_WRITE_PROGRAM_MEMORY_HIGH_BYTE	('C')
#define CMD_ISSUE_PAGE_WRITE				('m')

//J EEPROM Access
#define CMD_WRITE_EEPROM_BYTE				('D')	//J Alias
#define CMD_READ_EEPROM_BYTE				('d')	//J Alias
#define CMD_WRITE_DATA_MEMORY				('D')
#define CMD_READ_DATA_MEMORY				('d')

//J Block Accesss
#define CMD_START_BLOCK_FLASH_LOAD			('B')
#define CMD_START_BLOCK_EEPROM_LOAD			('B')
#define CMD_START_BLOCK_LOAD				('B')

#define CMD_START_BLOCK_FLASH_READ			('g')
#define CMD_START_BLOCK_EEPROM_READ			('g')
#define CMD_START_BLOCK_READ				('g')

//J Chip Erase
#define CMD_CHIP_ERASE						('e')

//J Lock and Fuse Bits
#define CMD_READ_LOCK_BITS					('r')
#define CMD_WRITE_LOCK_BITS					('l')
#define CMD_READ_LOW_FUSE_BITS				('F')
#define CMD_READ_HIGH_FUSE_BITS				('N')
#define CMD_READ_EXTENDED_FUSE_BITS			('Q')

//J CRC Check
#define CMD_CRC								('h')

//J I2C Address Autonegotiation Commands
#define CMD_I2C_AUTO_NEGOTIATION_START		('@')
#define CMD_I2C_AUTO_NEGOTIATION_DONE		('#')

//J CMD SYNC
#define CMD_SYNC							(0x1b)


#endif /* AVR_BOOTLOADER_H_ */