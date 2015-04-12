/*
 * parameter.h
 *
 * Created: 2014/08/23 2:40:39
 *  Author: sazae7
 */ 


#ifndef PARAMERTER_H_
#define PARAMERTER_H_


/*---------------------------------------------------------------------------*/
//J パラメータ読込用コールバック（1パラメータ毎に呼ばれる）
typedef void (*parameterCallback)(char *name, int16_t val);

/*---------------------------------------------------------------------------*/
//J パラメータ読み込み
uint8_t setupParams(char *line, parameterCallback cb);

#endif /* PARAMERTER_H_ */