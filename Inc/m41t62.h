/*
 * m41t62.h
 *
 *  Created on: Jan 31, 2017
 *      Author: gudozhni
 */

#ifndef M41T62_H_
#define M41T62_H_

#include "stm32l4xx_hal.h"
#include "wotch.h"

int m41t62_init(void);
int m41t62_readall(uint8_t *buf);
int m41t62_writeall(uint8_t *buf);
void m41t62_set_time(wotch_struct *wotch);
void m41t62_read_time(wotch_struct *wotch);
void m41t62_SQW(uint8_t state);
void m41t62_Alarm(uint8_t state);
#endif /* M41T62_H_ */
