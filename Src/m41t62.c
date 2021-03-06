/*
 * m41t62.c
 *
 *  Created on: Jan 31, 2017
 *      Author: gudozhni
 */

#include "m41t62.h"
#include "wotch.h"
#include <string.h>

#define SubSEC_REG 0x00
#define SEC_REG 0x01 //MSbit is STop bit, keep it 0
#define MIN_REG 0x02 //MSbit is OscFaultIntEN
#define HOUR_REG 0x03 //keep D7,D6 at 00
#define DAY_REG 0x04	// first 4 bits define SQW freq, set to 1Hz with 0xF0
#define DATE_REG 0x05 //keep D7,D6 at 00
#define CENT_MONTH_REG 0x06
#define YEAR_REG 0x07
#define CALIB_REG 0x08
#define WDG_REG 0x09
#define ALARM_MONTH_REG 0x0A
#define ALARM_DATE_REG 0x0B
#define ALARM_HOUR_REG 0x0C
#define ALARM_MIN_REG 0x0D
#define ALARM_SEC_REG 0x0E
#define FLAG_REG 0x0F




extern I2C_HandleTypeDef hi2c1;

uint8_t m41_regs[16]={0x01, 0x00, //sec
							0x00, //min
							0x00, //hour
							0xf0, //day + sqw freq
							0x00, //date
							0x00, //month
							0x00, //year
							0x80, //cal
							0x00, //WD
							0x00, //alarm month 010aaaaa
							0x00, //alarm date
							0x00, //alarm hour
							0x00, //alarm min
							0x00, //alarm sec
							0x00};//flags

int m41t62_init(void)
{
	int err = 0;
	uint8_t tmp[2];
	uint8_t m41_buf[16];
	memset(m41_buf,0x00,16);
	tmp[0] = 0x01;
	tmp[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, tmp, 2, 1000);
	tmp[0] = 0x01;
	tmp[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0,  m41_regs, 15, 1000);
	HAL_Delay(1000);

	return err;
};

int m41t62_readall(uint8_t *buf)
{
	int err = 0;
	HAL_StatusTypeDef stat;
	uint8_t tmp=0x00;

	stat = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tmp, 1, 1000);

	stat = HAL_I2C_Master_Receive(&hi2c1, 0xD1, buf, 16,1000);

	return err;
};

int m41t62_writeall(uint8_t *buf){
	int err = 0;
	uint8_t tmp=0x00;

	//HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tmp, 1, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0,  buf, 8, 1000);

	return err;
};

void m41t62_reset_int()
{

};
void m41t62_SQW(uint8_t state)
{
	uint8_t tmp[2];
	uint8_t tmp1;
	tmp[0]= 0x0A;
	tmp1 = 0x0A;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tmp1, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0xD1, &tmp1, 1, 1000);
	if (state)
		tmp[1] = 0x40 | tmp1;
	else
		tmp[1] = 0xBF & tmp1;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, tmp, 2,1000);
};
void m41t62_Alarm(uint8_t state)
{
	uint8_t tmp[2];
	uint8_t tmp1;
	tmp[0]= 0x0A;
	tmp1 = 0x0A;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tmp1, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0xD1, &tmp1, 1, 1000);
	if (state)
		tmp[1] = 0x80 | tmp1;
	else
		tmp[1] = 0x7F & tmp1;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, tmp, 2,1000);
};

void m41t62_read_time(wotch_struct * wotch){

	m41t62_readall(m41_regs);
	wotch->cur_time.bcd10hours = (m41_regs[HOUR_REG]>>4)&0x0F;
	wotch->cur_time.bcd1hours = (m41_regs[HOUR_REG])&0x0F;
	wotch->cur_time.bcd10minutes = (m41_regs[MIN_REG]>>4)&0x07;
	wotch->cur_time.bcd1minutes = (m41_regs[MIN_REG])&0x0F;
	wotch->cur_time.bcd10seconds = (m41_regs[SEC_REG]>>4)&0x07;
	wotch->cur_time.bcd1seconds = (m41_regs[SEC_REG])&0x0F;
	wotch->cur_time.bcdWday = (m41_regs[DAY_REG])&0x07;
	wotch->cur_time.bcd10date = (m41_regs[DATE_REG]>>4)&0x03;
	wotch->cur_time.bcd1date = (m41_regs[DATE_REG])&0x0F;
	wotch->cur_time.bcd10months = (m41_regs[CENT_MONTH_REG]>>4)&0x01;
	wotch->cur_time.bcd1months = (m41_regs[CENT_MONTH_REG])&0x0F;
	wotch->cur_time.bcd10years = (m41_regs[YEAR_REG]>>4)&0x0F;
	wotch->cur_time.bcd1years = (m41_regs[YEAR_REG])&0x0F;

	wotch->cur_time.seconds = 10*(wotch->cur_time.bcd10seconds)+wotch->cur_time.bcd1seconds;
	wotch->cur_time.minutes = 10*(wotch->cur_time.bcd10minutes)+wotch->cur_time.bcd1minutes;
	wotch->cur_time.hours = 10*(wotch->cur_time.bcd10hours)+wotch->cur_time.bcd1hours;
	wotch->cur_time.date = 10*(wotch->cur_time.bcd10date)+wotch->cur_time.bcd1date;
	wotch->cur_time.month = 10*(wotch->cur_time.bcd10months)+wotch->cur_time.bcd1months;
	wotch->cur_time.year = 2000 + 10*(wotch->cur_time.bcd10years)+wotch->cur_time.bcd1years;

};

void m41t62_set_time(wotch_struct *wotch){
	uint8_t m41_buf[16];
	memset(m41_buf,0x00,16);

	m41_buf[SEC_REG] = ((uint8_t)(wotch->cur_time.bcd10seconds<<4)+wotch->cur_time.bcd1seconds)&0x7F;
	m41_buf[MIN_REG] = ((uint8_t)(wotch->cur_time.bcd10minutes<<4)+wotch->cur_time.bcd1minutes)&0x7F;
	m41_buf[HOUR_REG] = ((uint8_t)(wotch->cur_time.bcd10hours<<4)+wotch->cur_time.bcd1hours)&0x3F;
	m41_buf[DATE_REG] = ((uint8_t)(wotch->cur_time.bcd10date<<4)+wotch->cur_time.bcd1date)&0x3F;
	m41_buf[CENT_MONTH_REG] = ((uint8_t)(wotch->cur_time.bcd10months<<4)+wotch->cur_time.bcd1months)&0x1F;
	m41_buf[YEAR_REG] = ((uint8_t)(wotch->cur_time.bcd10years<<4)+wotch->cur_time.bcd1years);

	m41_buf[0]=0x01;
	m41_buf[4]=0xf0; //always write 0xf.

	 m41t62_writeall(m41_buf);

};
