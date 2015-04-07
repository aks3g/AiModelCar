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

	//J ページバッファにデータを格納
	//J Buffer SizeはAPP_SECTION_PAGE_SIZEに固定
	loadPageToFlashPageBuffer((uint16_t)buf);

	//J ページバッファからフラッシュにデータを書込み
	writeApplicationPageFromFlashPageBuffer(pageAddress);
	
	waitForDoneFlashOperation();

	return;
}