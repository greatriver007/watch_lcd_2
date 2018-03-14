/*
 * sh1106.h
 *
 *  Created on: Jan 26, 2017
 *      Author: pups
 */

#ifndef SH1106_H_
#define SH1106_H_

#include "stm32l4xx_hal.h"

#define SH1106_SET_CONTRAST 0x81 //followed by a value 0x00-0xff, 0x80 default
#define SH1106_DISPLAY_ALL_NORMAL 0xA4
#define SH1106_DISPLAY_ALL_ON 0xA5
#define SH1106_NONINVERT_DISPLAY 0xA6
#define SH1106_INVERT_DISPLAY 0xA7
#define SH1106_DISPLAY_OFF 0xAE
#define SH1106_DISPLAY_ON 0xAF

#define SH1106_SET_DISPLAY_OFFSET 0xD3
#define SH1106_SET_COM_PINS 0xDA

#define SH1106_SET_VCOM_DETECT 0xDB

#define SH1106_SETDISPLAYCLOCKDIV 0xD5
#define SH1106_SETPRECHARGE 0xD9

#define SH1106_SET_MULTIPLEX 0xA8

#define SH1106_SET_COLUMN_LOW 0x00 // 4 lsb's  is the low part of the address
#define SH1106_SET_COLUMN_HIGH 0x10 // 4 lsb's  is the low part of the address

#define SH1106_SETSTARTLINE 0x40  // 6 lsb's is the address

#define SH1106_MEMORYMODE 0x20
#define SH1106_COLUMNADDR 0x21
#define SH1106_PAGEADDR   0x22

#define SH1106_COMSCANINC 0xC0
#define SH1106_COMSCANDEC 0xC8

#define SH1106_SEGREMAP 0xA0

#define SH1106_CHARGEPUMP 0x8D

#define SH1106_EXTERNALVCC 0x1
#define SH1106_SWITCHCAPVCC 0x2

#define LCD_CS_PIN 0x0800  // all from GPIOB
#define LCD_CD_PIN 0x0004
#define LCD_RST_PIN 0x0400

#define FRAME_SIZE 1024

int SH1106_init(void);
int SH1106_command(uint8_t cmd);
int SH1106_write_frame(void);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);


#endif /* SH1106_H_ */
