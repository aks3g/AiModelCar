/*
 * parameter.h
 *
 * Created: 2014/08/23 2:40:39
 *  Author: sazae7
 */ 


#ifndef PARAMERTER_H_
#define PARAMERTER_H_


/*---------------------------------------------------------------------------*/
//J �p�����[�^�Ǎ��p�R�[���o�b�N�i1�p�����[�^���ɌĂ΂��j
typedef void (*parameterCallback)(char *name, int16_t val);

/*---------------------------------------------------------------------------*/
//J �p�����[�^�ǂݍ���
uint8_t setupParams(char *line, parameterCallback cb);

#endif /* PARAMERTER_H_ */