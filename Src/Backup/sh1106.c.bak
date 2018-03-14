/*
 * SH1106.c
 *
 *  Created on: Jan 26, 2017
 *      Author: pups
 */
#include "wotch.h"
#include "SH1106.h"

uint8_t lcd_page;
extern SPI_HandleTypeDef hspi1;
extern wotch_struct wotch1;


int SH1106_init(void)
{
	int err = 0;
	GPIOB->BRR = LCD_RST_PIN;
	HAL_Delay(10);
	GPIOB->BSRR = LCD_RST_PIN;
	GPIOB->BSRR = LCD_CS_PIN;

	HAL_Delay(10);
	SH1106_command(0xC8);
	SH1106_command(0xA1);
	SH1106_command(0xAF);
	HAL_Delay(100);

	return err;
};


int SH1106_command(uint8_t cmd)
{
	int err = 0;

	GPIOB->BRR = LCD_CD_PIN|LCD_CS_PIN;
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 500);
	GPIOB->BSRR = LCD_CD_PIN|LCD_CS_PIN;
	return err;
};

int SH1106_write_frame()
{	int err = 0;
	lcd_page = 0;
	while (wotch1.writing_lcd);
	SH1106_command(0x02);
	SH1106_command(0x10);
	SH1106_command(0xB0);
	GPIOB->BSRR = LCD_CD_PIN;
	GPIOB->BRR = LCD_CS_PIN;
	//HAL_Delay(1);
	wotch1.writing_lcd = 1;
	HAL_SPI_Transmit_DMA(&hspi1, wotch1.lcd_frame_buffer, (uint16_t)128);


//	GPIOB->BRR = LCD_CD_PIN;
//	GPIOB->BSRR = LCD_CS_PIN;
	return err;
};

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	lcd_page++;
	if (lcd_page < 8)
	{
		GPIOB->BSRR = LCD_CS_PIN;
		SH1106_command(0x02);
		SH1106_command(0x10);
		SH1106_command(0xB0+lcd_page);
		GPIOB->BSRR = LCD_CD_PIN;
		GPIOB->BRR = LCD_CS_PIN;

		HAL_SPI_Transmit_DMA(&hspi1, wotch1.lcd_frame_buffer+128*lcd_page, (uint16_t)128);
	}
	else
		wotch1.writing_lcd = 0;

};
