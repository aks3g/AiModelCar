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
//J UART のBaudrate設定
#define USER_UART_BAUDRATE				(57600)

/*---------------------------------------------------------------------------*/
//J ユーザランド処理の初期化
void initialize_userland(void);

/*---------------------------------------------------------------------------*/
//J Main関数で全力で呼ばれる処理
void updateUserland(void);

/*---------------------------------------------------------------------------*/
//J センサデータの読み込みが終わる毎に実行される処理
//J Duty比を返すと反映される
uint8_t onSensorInterrupt(void *sensor);

/*---------------------------------------------------------------------------*/
//J Timer割込みで実行される処理
void onTimerInterrupt(uint32_t tick);

/*---------------------------------------------------------------------------*/
//J SWが押された時、押されている間リピート、SWが放された時　の処理
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