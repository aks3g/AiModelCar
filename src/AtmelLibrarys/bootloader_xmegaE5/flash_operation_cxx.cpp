/*
 * flash_operation.cpp
 *
 * Created: 2014/11/20 8:58:30
 *  Author: sazae7
 */ 

#include "flash_operation.h"

void programFlashPageWithErase(uint32_t pageAddress, uint8_t *buf, uint16_t bufSize, bool doErase)
{
	(void)doErase;
	(void)bufSize;

	//J �y�[�W�o�b�t�@�Ƀf�[�^���i�[
	//J Buffer Size��APP_SECTION_PAGE_SIZE�ɌŒ�
	loadPageToFlashPageBuffer((uint16_t)buf);

	//J �y�[�W�o�b�t�@����t���b�V���Ƀf�[�^��������
	writeApplicationPageFromFlashPageBuffer(pageAddress);
	
	waitForDoneFlashOperation();

	return;
}