
/*
 * flash_operation.s
 *
 * Created: 2014/11/18 1:40:57
 *  Author: sazae7
 */ 

#include <avr/io.h>

#define CCP_SPM										(0x9D)
#define CCP_IOREG									(0xD8)

#define NVM_CMD_NO_OPERATION						(0x00)
#define NVM_CMD_READ_USER_SIG_ROW					(0x01)
#define NVM_CMD_READ_CALIB_ROW						(0x02)
#define NVM_CMD_READ_FUSES							(0x07)
#define NVM_CMD_WRITE_LOCK_BITS						(0x08)
#define NVM_CMD_ERASE_USER_SIG_ROW					(0x18)
#define NVM_CMD_WRITE_USER_SIG_ROW					(0x1A)
#define NVM_CMD_ERASE_APP							(0x20)
#define NVM_CMD_ERASE_APP_PAGE						(0x22)
#define NVM_CMD_LOAD_FLASH_BUFFER					(0x23)
#define NVM_CMD_WRITE_APP_PAGE						(0x24)
#define NVM_CMD_ERASE_WRITE_APP_PAGE				(0x25)
#define NVM_CMD_ERASE_FLASH_BUFFER					(0x26)
#define NVM_CMD_ERASE_BOOT_PAGE						(0x2A)
#define NVM_CMD_ERASE_FLASH_PAGE					(0x2B)
#define NVM_CMD_WRITE_BOOT_PAGE						(0x2C)
#define NVM_CMD_ERASE_WRITE_BOOT_PAGE				(0x2D)
#define NVM_CMD_WRITE_FLASH_PAGE					(0x2E)
#define NVM_CMD_ERASE_WRITE_FLASH_PAGE				(0x2F)
#define NVM_CMD_ERASE_EEPROM						(0x30)
#define NVM_CMD_ERASE_EEPROM_PAGE					(0x32)
#define NVM_CMD_WRITE_EEPROM_PAGE					(0x34)
#define NVM_CMD_ERASE_WRITE_EEPROM_PAGE				(0x35)
#define NVM_CMD_ERASE_EEPROM_BUFFER					(0x36)
#define NVM_CMD_APP_CRC								(0x38)
#define NVM_CMD_BOOT_CRC							(0x39)
#define NVM_CMD_FLASH_RANGE_CRC						(0x3A)
#define NVM_CMD_CHIP_ERASE							(0x40)
#define NVM_CMD_READ_NVM							(0x43)
#define NVM_CMD_WRITE_FUSE							(0x4C)
#define NVM_CMD_ERASE_BOOT							(0x68)
#define NVM_CMD_FLASH_CRC							(0x78)




;-----------------------------------------------------------------------------
; readFlashWord
;  arg
;    - uint32_t address : r22 - 25
;  ret
;    - uint8_t 
.section .text
.global readFlashByte

readFlashByte: 
	in		r19, RAMPZ								; Save RAMPZ
	out		RAMPZ, r24								; RAMPZ <- address[23:16]
	movw	ZL, r22									; ZL    <- address[15:00]
	elpm	r24, Z+									; Z     -> r24
	out		RAMPZ, r19								; Restore RAMPZ
	ret


; readFlashWord
;  arg
;    - uint32_t address : r22 - 25
;  ret
;    - uint16_t 
.section .text
.global readFlashWord

readFlashWord: 
	in		r19, RAMPZ								; Save RAMPZ
	out		RAMPZ, r24								; RAMPZ <- address[23:16]
	movw	ZL, r22									; ZL    <- address[15:00]
	elpm	r24, Z+									; Z     -> r24, Z++
	elpm	r25, Z									; Z     -> r25
	out		RAMPZ, r19								; Restore RAMPZ
	ret


; writeFlashWord
;  arg
;    - uint16_t address : r24 - r25
;    - uint16_t val     : r22 - r23
;  ret
;    - void
.section .text
.global writeFlashWord

writeFlashWord:
	in		r19, RAMPZ								; Save RAMPZ
	movw	r0, r22									; R0:R1 <- val[15:00]
	ldi		r20, NVM_CMD_LOAD_FLASH_BUFFER			; R20   <- LOAD FLASH BUFFER CMD
	jmp		__commonSPM								; Jump to SPM code.
	

; writeFlashApplicationPageStart
;  arg
;    - uint32_t address : r22 - 25
;  ret
;    - void
.section .text
.global writeFlashApplicationPageStart

writeFlashApplicationPageStart:
	in		r19, RAMPZ								; Save RAMPZ
	out		RAMPZ, r24								; RAMPZ   <- address[23:16]
	movw	r24, r22								; r24:r25 <- address[15:00]
	ldi		r20, NVM_CMD_WRITE_APP_PAGE				; R20     <- NVM_CMD_WRITE_APP_PAGE
	jmp		__commonSPM								; Jump to SPM code.

	

; loadPageToFlashPageBuffer
;  arg
;    - uint16_t bufAddr : R24 - R25
;  ret
;    - void
.section .text
.global loadPageToFlashPageBuffer

loadPageToFlashPageBuffer:
	clr		ZL										; Z <- 0 , Z pointer is zero
	clr		ZH										;
	out		RAMPX, r1								; Push RAMPX pointer.
	movw	XL, r24									; X <- data buffer address.
	ldi 	r20, NVM_CMD_LOAD_FLASH_BUFFER			; R20 <- NVM_CMD_LOAD_FLASH_BUFFER
	sts		NVM_CMD, r20							; NVM_CMD <- R20

	; Set Constant Value
	ldi		r21, ((APP_SECTION_PAGE_SIZE/2)&0xFF)	; R21 <- Count of Write Size in word
	ldi		r18, CCP_SPM							; R18 <- SPM Unlock Key

writeFlashPageCount_Loop:
	ld		r0, X+									; R0 <- *(bufAddr++)
	ld		r1, X+									; R0 <- *(bufAddr++)
	sts		CCP, r18								; SPM Unlock
	spm												; Self-program.
	adiw	ZL, 2									; Z <- Z + 2 , Dest address increment

	dec		r21										; Decrement Count of Write
	brne	writeFlashPageCount_Loop				; Jump non zero

	clr		r1										; Clear R1 for GCC _zero_reg_
	ret

; writeApplicationPageFromFlashPageBuffer
;  arg
;    - uint32_t address		: r24 - r22 J有効なのは24ビット
;  ret
;    - void
.section .text
.global writeApplicationPageFromFlashPageBuffer

writeApplicationPageFromFlashPageBuffer:
	in		r19, RAMPZ								; Save RAMPZ
	out		RAMPZ, r24								; 
	movw	r24, r22								;
	ldi		r20, NVM_CMD_ERASE_WRITE_APP_PAGE		; R20 <- NVM_CMD_ERASE_WRITE_APP_PAGE
	jmp		__commonSPM								; Jump to SPM code.


; readFlashPage
;  arg
;    - uint32_t address		: r23 - r20
;    - uint16_t data_ptr	: r25 - r24
;  ret
;    - void
.section .text
.global readFlashPage

readFlashPage:
	in		r19, RAMPZ								; Save RAMPZ reg.
	out		RAMPZ, r22								; RAMPZ <- R22 (Address[23:16])
	movw	ZL, r20									; Z     <- R21:R20 (Address[15:0])

	out		RAMPX, r1								; ?
	movw	XL, r24									; X     <- R25:R24 (data pointer)

	ldi		r20, NVM_CMD_NO_OPERATION				; R20   <- NVM_CMD_NOP
	sts		NVM_CMD, r20							; LPM reads Flash.

	ldi		r21, ((APP_SECTION_PAGE_SIZE)&0xFF)		; R21   <- inital byte count

readFlashPage_Loop:
	elpm	r24, Z+									; R25:R24 <- *Z++ Jフラッシュの内容読込
	elpm	r25, Z+
	st		X+, r24									; *X++    <- R24
	st		X+, r25									; *X++    <- R25

	dec		r21										; R21--
	brne	readFlashPage_Loop						; if R21 != 0 then jump to readFlashPage_Loop

	out	RAMPZ, r19									; Restore RAMPZ reg.
	ret





; waitForDoneFlashOperation
;  arg
;    - void
;  ret
;    - void
.section .text
.global waitForDoneFlashOperation

waitForDoneFlashOperation:
	lds		r18, NVM_STATUS							; R18 <- NVM_STATUS
	sbrc	r18, NVM_NVMBUSY_bp						; Check NVM_NVMBUSY_bp
	rjmp	waitForDoneFlashOperation				; Jump to waitForDoneFlashOperation
	clr		r18										; R18 <= 0
	sts		NVM_CMD, r18							; NVM_CMD <- R18
	ret


; eraseFrashOnApplicationSection
;  arg
;    - void
;  ret
;    - void
.section .text
.global eraseFrashOnApplicationSection

eraseFrashOnApplicationSection:
	in		r19, RAMPZ								; Save RAMPZ
	clr		r24										; R24 <= 0
	clr		r25										; R25 <= 0
	out		RAMPZ, r24								; RAMPZ <- R24
	ldi		r20, NVM_CMD_ERASE_APP					; R20 <- NVM_CMD_ERASE_APP
	jmp		__commonSPM								; Jump to SPM code.



;-----------------------------------------------------------------------------
; eraseUserSignature
;  arg
;    - void
;  ret
;    - void
.section .text
.global eraseUserSignature

eraseUserSignature:
	in		r19, RAMPZ								; Save RAMPZ
	ldi		r20, NVM_CMD_ERASE_USER_SIG_ROW			; R20 <- NVM_CMD_ERASE_USER_SIG_ROW
	jmp		__commonSPM								; Jump to SPM code.


; writeUserSignature
;  arg
;    - void
;  ret
;    - void
.section .text
.global writeUserSignature

writeUserSignature:
	in		r19, RAMPZ								; Save RAMPZ
	ldi		r20, NVM_CMD_WRITE_USER_SIG_ROW			; R20 <- NVM_CMD_WRITE_USER_SIG_ROW
	jmp		__commonSPM								; Jump to SPM code.



; readUserSignature
;  arg
;    - uint16_t address			: R24 - R25
;  ret
;    - uint8_t
.section .text
.global readUserSignature

readUserSignature:
;	in		r19, RAMPZ								; Save RAMPZ
	ldi		r20, NVM_CMD_READ_USER_SIG_ROW			; R20 <- NVM_CMD_ERASE_USER_SIG_ROW
	jmp		__commonLPM								; Jump to SPM code.


;-----------------------------------------------------------------------------
; readCalibrationInfo
;  arg
;    - uint16_t address			: R24 - R25
;  ret
;    - uint8_t
.section .text
.global readCalibrationInfo

readCalibrationInfo:
;	in		r19, RAMPZ								; Save RAMPZ
	ldi		r20, NVM_CMD_READ_CALIB_ROW				; R20 <- NVM_CMD_ERASE_USER_SIG_ROW
	jmp		__commonLPM								; Jump to SPM code.



;-----------------------------------------------------------------------------
; writeLockBit
;  arg
;    - uint8_t lock				: R24
;  ret
;    - void
.section .text
.global writeLockBit

writeLockBit:
	sts		NVM_DATA0, r24							; NVM_DATA0 <- r24(lock)
	ldi		r20, NVM_CMD_WRITE_LOCK_BITS			; R20 <- NVM_CMD_WRITE_LOCK_BITS
	rjmp	__commonCMD


; readLockBit
;  arg
;    - void
;  ret
;    - uint8_t lock				: R24
.section .text
.global readLockBit

readLockBit:
	lds		r24, NVM_LOCKBITS						; R20 <- NVM_CMD_WRITE_LOCK_BITS
	ret


; readFuseByte
;  arg
;    - uint8_t index			: R24
;  ret
;    - uint8_t lock				: R24
.section .text
.global readFuseByte

readFuseByte:
	sts		NVM_ADDR0, r24							; NVM_ADDR0 <- R24
	clr		r24
	sts		NVM_ADDR1, r24							; Fill zero to upper address
	sts		NVM_ADDR2, r24
	ldi		r20, NVM_CMD_READ_FUSES
	rcall	__commonCMD_return32bits
	movw	r24, r22								; R24 <- R22
	ret




;-----------------------------------------------------------------------------
; __commonSPM
;  arg
;    - Input to SPM Command		: r0 - r1
;    - Low Byte of Z pointer	: r24 - r25   e.g Address
;    - NVM Command				; r20
;  ret
;    - void
.section .text

__commonSPM:
	movw	ZL, r24									; Z   <- R25:R24
	sts		NVM_CMD, r20							; Load NVM Command to NVM_CMD reg.
	ldi		r18, CCP_SPM							; r18 = SPM Protection Code
	sts		CCP, r18								; CCP <- r18
	spm												; Self programming
	clr		r1										; _zero_reg_
	out		RAMPZ, r19								; Restore RAMPZ
	ret		


; __commonLPM
;  arg
;    - Low Byte of Z pointer	: r24 - r25   e.g Address
;    - NVM Command				; r20
;  ret
;    - uint8_t					; r24
.section .text

__commonLPM:
	movw	ZL, r24									; Z   <- R25:R24
	sts		NVM_CMD, r20							; Load NVM Command to NVM_CMD reg.
	lpm		r24, Z									; R24 <- Z (戻り値設定)
;	out		RAMPZ, r19								; Restore RAMPZ
	ret		


; __commonCMD_return32bits
;  arg
;    - uint8_t					: R20
;  ret
;    - uint32_t					: R22 - R25
.section .text

__commonCMD_return32bits:
	sts		NVM_CMD, r20							; NVM_CMD <- r20
	ldi		r18, CCP_IOREG							; r18 <- CCP IOREG Unlock Key
	ldi		r19, NVM_CMDEX_bm						; r19 <- NVM Command EX
	sts		CCP, r18								; CCP IOREG Unlock
	sts		NVM_CTRLA, r19							; Issue NVM Command Ex
	lds		r22, NVM_DATA0							; packing 32bit value
	lds		r23, NVM_DATA1
	lds		r24, NVM_DATA2
	clr		r25
	ret

; __commonCMD
;  arg
;    - uint8_t					: R20
;  ret
;    - void
.section .text

__commonCMD:
	sts		NVM_CMD, r20							; NVM_CMD <- r20
	ldi		r18, CCP_IOREG							; r18 <- CCP IOREG Unlock Key
	ldi		r19, NVM_CMDEX_bm						; r19 <- NVM Command EX
	sts		CCP, r18								; CCP IOREG Unlock
	sts		NVM_CTRLA, r19							; Issue NVM Command Ex
	clr		r1										; _zero_reg_
	ret