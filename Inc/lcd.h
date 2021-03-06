/*
 * lcd.h
 *
 *  Created on: Feb 5, 2017
 *      Author: gudozhni
 */

#ifndef LCD_H_
#define LCD_H_
#include "sh1106.h"
#include "menu.h"

//#define
void lcd_put_digit(uint8_t dig, uint8_t pos, wotch_struct *wotch);
void lcd_put_digit1(uint8_t dig, uint8_t pos, wotch_struct *wotch);

void lcd_update_time(wotch_struct *wotch);
void lcd_update_time1(wotch_struct *wotch);
void lcd_write_menu_line(wotch_struct *wotch,MENU_line *ln,uint8_t position);
void lcd_write_char(wotch_struct *wotch,uint8_t pos_line, uint8_t pos_str, char symb);
void lcd_write_float(wotch_struct *wotch,float number, uint8_t xposition, uint8_t yposition);
void lcd_menu_arrow(wotch_struct * wotch,uint8_t pos);
void lcd_draw_sec_bar(wotch_struct *wotch);
void lcd_put_time_str(wotch_struct *wotch);
void lcd_put_date_str(wotch_struct *wotch);
void lcd_put_date(wotch_struct *wotch);
void lcd_clear(wotch_struct * wotch);
#endif /* LCD_H_ */
