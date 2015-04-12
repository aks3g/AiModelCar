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
//J �Ƃ����`���ł��肢���܂��B
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
	//J ���p�X�y�[�X�ƃ^�u�̊ԃ|�C���^��i�߂�
	while (*str == ' ' || *str == '\t') {
		str++;
	}
	
	return str;
}

/*---------------------------------------------------------------------------*/
static char *_sRemoveNextSpace(char *str)
{
	//J ���p�X�y�[�X�ƃ^�u�Ŗ����ԃ|�C���^��i�߂�
	while (*str != ' ' && *str != '\t') {
		str++;
	}
	
	//J �k�������ɒu������
	*str = '\0';
	
	//J �k��������1���Ƃ�Ԃ�
	return str + 1;
}

/*---------------------------------------------------------------------------*/
static uint8_t _sParseParamNameValuePair(char *str, char **name, int16_t *val)
{
	char *ptr = str;

	//J �擪�̋󔒂�����
	ptr = _sSkipFrontSpace(ptr);
	
	//J �p�����[�^����n��
	*name = str;
	
	//J ���̋󔒂��k�������ɒu��������
	ptr = _sRemoveNextSpace(ptr);
	ptr = _sSkipFrontSpace(ptr);
	
	//J �C�R�[�����΂�
	while (*ptr++ == '=') {
	}
	ptr = _sSkipFrontSpace(ptr);

	//J �l�������OK
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

