/*
 * userland.h
 *
 * Created: 2014/11/04 22:36:16
 *  Author: sazae7
 */ 


#ifndef USERLAND_H_
#define USERLAND_H_

#include <utils.h>
#include <core_driver.h>
#include <timer_driver.h>
#include <gpio_driver.h>
#include <spi_driver.h>
#include <i2c_driver.h>
#include <adc_driver.h>
#include <uart_driver.h>

#include "ff.h"
#include "ioconfig.h"

/*---------------------------------------------------------------------------*/
//J UART ��Baudrate�ݒ�
#define USER_UART_BAUDRATE				(57600)

/*---------------------------------------------------------------------------*/
//J ���[�U�����h�����̏�����
void initialize_userland(void);

/*---------------------------------------------------------------------------*/
//J Main�֐��őS�͂ŌĂ΂�鏈��
void updateUserland(void);

/*---------------------------------------------------------------------------*/
//J �Z���T�f�[�^�̓ǂݍ��݂��I��閈�Ɏ��s����鏈��
//J Duty���Ԃ��Ɣ��f�����
uint8_t onSensorInterrupt(void *sensor);

/*---------------------------------------------------------------------------*/
//J Timer�����݂Ŏ��s����鏈��
void onTimerInterrupt(uint32_t tick);

/*---------------------------------------------------------------------------*/
//J SW�������ꂽ���A������Ă���ԃ��s�[�g�ASW�������ꂽ���@�̏���
void onSw0Pressed(void);
void onSw0Repeat(void);
void onSw0Released(void);

void onSw1Pressed(void);
void onSw1Repeat(void);
void onSw1Released(void);

void onSw2Pressed(void);
void onSw2Repeat(void);
void onSw2Released(void);

#endif /* USERLAND_H_ */