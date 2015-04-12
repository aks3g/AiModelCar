/*
 * parameter.cpp
 *
 * Created: 2014/08/23 2:40:25
 *  Author: sazae7
 */ 
#include <stdlib.h>
#include <string.h>

#include <utils.h>
#include <gpio_driver.h>

#include "parameter.h"

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static char *_sSkipFrontSpace(char *str);
static char *_sRemoveNextSpace(char *str);
static uint8_t _sParseParamNameValuePair(char *str, char **name, int16_t *val);

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
//J ParameterName = Value
//J という形式でお願いします。
uint8_t setupParams(char *line, parameterCallback cb)
{
	if (line[0] == '#') {
		return 0;
	}
	
	if (cb == NULL) {
		return -1; //TODO
	}
	
	int16_t val = 0;
	char *name = NULL;
	uint8_t ret = _sParseParamNameValuePair(line, &name, &val);
	if (ret == 0) {
		cb(name, val);
	}

	return 0;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static char *_sSkipFrontSpace(char *str)
{
	//J 半角スペースとタブの間ポインタを進める
	while (*str == ' ' || *str == '\t') {
		str++;
	}
	
	return str;
}

/*---------------------------------------------------------------------------*/
static char *_sRemoveNextSpace(char *str)
{
	//J 半角スペースとタブで無い間ポインタを進める
	while (*str != ' ' && *str != '\t') {
		str++;
	}
	
	//J ヌル文字に置き換え
	*str = '\0';
	
	//J ヌル文字の1個あとを返す
	return str + 1;
}

/*---------------------------------------------------------------------------*/
static uint8_t _sParseParamNameValuePair(char *str, char **name, int16_t *val)
{
	char *ptr = str;

	//J 先頭の空白を消す
	ptr = _sSkipFrontSpace(ptr);
	
	//J パラメータ名を渡す
	*name = str;
	
	//J 次の空白をヌル文字に置き換える
	ptr = _sRemoveNextSpace(ptr);
	ptr = _sSkipFrontSpace(ptr);
	
	//J イコールを飛ばす
	while (*ptr++ == '=') {
	}
	ptr = _sSkipFrontSpace(ptr);

	//J 値が作れればOK
	if (ptr[0] != '\0') {
		*val = strtol(ptr, NULL, 10);
		return 0;
	} else {
		*name = NULL;
		*val  = 0;
		return -1;
	}

	return -1;
}

