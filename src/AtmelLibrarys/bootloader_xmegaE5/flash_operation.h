/*
 * flash_operation.h
 *
 * Created: 2014/11/18 1:41:18
 *  Author: sazae7
 */ 
#include <avr/io.h>

#ifndef FLASH_OPERATION_H_
#define FLASH_OPERATION_H_

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

// ---------------------------------------------------------------------------
// Assembler
// ---------------------------------------------------------------------------
uint8_t  readFlashByte (uint32_t address);
uint16_t readFlashWord (uint32_t address);
void     writeFlashWord(uint16_t address, uint16_t val);
void     writeFlashApplicationPageStart(uint32_t address);

//J BufferÇÃÉTÉCÉYÇÕ128Byteå≈íËÇ≈Ç∑
void     loadPageToFlashPageBuffer(uint16_t bufAddr);
void     writeApplicationPageFromFlashPageBuffer(uint32_t address);
void     readFlashPage(uint8_t *data, uint32_t address);


void     eraseUserSignature(void);
void     writeUserSignature(void);
uint8_t  readUserSignature(uint16_t address);
uint8_t  readCalibrationInfo(uint16_t address);


// ---------------------------------------------------------------------------
uint8_t  writeLockBit(uint8_t lock);
uint8_t  readLockBit(void);
uint8_t  readFuseByte(uint8_t idx);

void     eraseFrashOnApplicationSection(void);

// ---------------------------------------------------------------------------
void	 waitForDoneFlashOperation(void);



// ---------------------------------------------------------------------------
// C++
// ---------------------------------------------------------------------------
void     programFlashPageWithErase(uint32_t pageAddress, uint8_t *buf, uint16_t bufSize, bool doErase);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* FLASH_OPERATION_H_ */