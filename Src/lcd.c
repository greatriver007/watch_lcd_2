/*
 * lcd.c
 *
 *  Created on: Feb 5, 2017
 *      Author: gudozhni
 */
#include "wotch.h"
#include "lcd.h"
#include "digits.h"
#include "digits1.h"
#include "menu.h"
#include "font7x5.h"

MENU_line mon = {7, "Monday,"};
MENU_line tue = {8, "Tuesday,"};
MENU_line wed = {10, "Wednesday,"};
MENU_line thu = {9, "Thursday,"};
MENU_line fri = {7, "Friday,"};
MENU_line sat = {9, "Saturday,"};
MENU_line sun = {7, "Sunday,"};
MENU_line * DAY_line[8] = {NULL, &(mon), &(tue), &(wed), &(thu), &(fri), &(sat), &(sun)};

void lcd_put_digit(uint8_t dig, uint8_t pos, wotch_struct *wotch)
{
	for (uint8_t i=0;i<28;i++)
	{
		if (pos < 2)
		{
			wotch->lcd_frame_buffer[129+i+28*pos] = digits[dig*112+i];
			wotch->lcd_frame_buffer[257+i+28*pos] = digits[dig*112+28+i];
			wotch->lcd_frame_buffer[385+i+28*pos] = digits[dig*112+56+i];
			wotch->lcd_frame_buffer[513+i+28*pos] = digits[dig*112+84+i];
		}
		else
		{
			wotch->lcd_frame_buffer[143+i+28*pos] = digits[dig*112+i];
			wotch->lcd_frame_buffer[271+i+28*pos] = digits[dig*112+28+i];
			wotch->lcd_frame_buffer[399+i+28*pos] = digits[dig*112+56+i];
			wotch->lcd_frame_buffer[527+i+28*pos] = digits[dig*112+84+i];
		};
	};
};

void lcd_put_digit1(uint8_t dig, uint8_t pos, wotch_struct *wotch)
{
	for (uint8_t i=0;i<17;i++)
	{
		if (pos < 2)
		{
			wotch->lcd_frame_buffer[129+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+i];
			wotch->lcd_frame_buffer[257+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+17+i];
			wotch->lcd_frame_buffer[385+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+34+i];
			wotch->lcd_frame_buffer[513+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+51+i];
		}
		else
		{
			wotch->lcd_frame_buffer[143+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+i];
			wotch->lcd_frame_buffer[271+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+17+i];
			wotch->lcd_frame_buffer[399+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+34+i];
			wotch->lcd_frame_buffer[527+i+17*pos] = digits_1[dig*DIGIT_SIZE_1+51+i];
		};
	};
};

void lcd_update_time(wotch_struct *wotch)
{
	for (uint8_t i=0;i<14;i++)
	{
	wotch->lcd_frame_buffer[185+i] = digits[1120+i];
	wotch->lcd_frame_buffer[313+i] = digits[1134+i];
	wotch->lcd_frame_buffer[441+i] = digits[1148+i];
	wotch->lcd_frame_buffer[569+i] = digits[1162+i];
	};
	wotch->prev_time.bcd10hours = wotch->cur_time.bcd10hours;
	lcd_put_digit(wotch->cur_time.bcd10hours,0,wotch);
	wotch->prev_time.bcd1hours = wotch->cur_time.bcd1hours;
	lcd_put_digit(wotch->cur_time.bcd1hours,1,wotch);
	wotch->prev_time.bcd10minutes = wotch->cur_time.bcd10minutes;
	lcd_put_digit(wotch->cur_time.bcd10minutes,2,wotch);
	wotch->prev_time.bcd1minutes = wotch->cur_time.bcd1minutes;
	lcd_put_digit(wotch->cur_time.bcd1minutes,3,wotch);
};

void lcd_update_time1(wotch_struct *wotch)
{
	for (uint8_t i=0;i<5;i++)
	{
	wotch->lcd_frame_buffer[185+i] = digits_1[680+i];
	wotch->lcd_frame_buffer[313+i] = digits_1[685+i];
	wotch->lcd_frame_buffer[441+i] = digits_1[690+i];
	wotch->lcd_frame_buffer[569+i] = digits_1[695+i];
	};
	wotch->prev_time.bcd10hours = wotch->cur_time.bcd10hours;
	lcd_put_digit1(wotch->cur_time.bcd10hours,0,wotch);
	wotch->prev_time.bcd1hours = wotch->cur_time.bcd1hours;
	lcd_put_digit1(wotch->cur_time.bcd1hours,1,wotch);
	wotch->prev_time.bcd10minutes = wotch->cur_time.bcd10minutes;
	lcd_put_digit1(wotch->cur_time.bcd10minutes,2,wotch);
	wotch->prev_time.bcd1minutes = wotch->cur_time.bcd1minutes;
	lcd_put_digit1(wotch->cur_time.bcd1minutes,3,wotch);
};

void lcd_write_menu_line(wotch_struct *wotch, MENU_line *ln,uint8_t position){
	uint16_t temp = 128*(position+1)+8;

	if (ln == NULL)
	{
		for (uint8_t i = 0; i<120;i++)
		{
			wotch->lcd_frame_buffer[temp+i]=0x00;
		};
	}
	else
	{
		for (uint8_t i = 0; i<ln->length;i++)
		{
			uint8_t symb = ln->str[i] - 0x20;
			for(uint8_t j = 0; j < 5; j++)
			{
				wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
			};
			wotch->lcd_frame_buffer[temp+5]=0x00;
			temp+=6;
		};
	}

};

void lcd_write_char(wotch_struct *wotch,uint8_t pos_line, uint8_t pos_str, char symb){
	uint16_t temp = 128*(pos_line)+6*(pos_str);

	uint8_t symb1 = symb - 0x20;
	for(uint8_t j = 0; j < 5; j++)
	{
		wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb1+j];
	};
	wotch->lcd_frame_buffer[temp+5]=0x00;

};


void lcd_write_float(wotch_struct *wotch, float number, uint8_t xposition, uint8_t yposition){
	uint8_t temp;
	uint16_t ind = 128*(yposition+1)+6*xposition;
	temp = (uint8_t)number;

	for(uint8_t j = 0; j < 5; j++)
	{
		wotch->lcd_frame_buffer[ind+j]=font7x5[5*(temp+16)+j];
	};
	wotch->lcd_frame_buffer[ind+5]=0x00;
	ind+=6;

	number = 10.0*(number - (float)temp);
	temp = (uint8_t)number;

	for(uint8_t j = 0; j < 5; j++)
	{
		wotch->lcd_frame_buffer[ind+j]=font7x5[5*(14)+j];
	};
	wotch->lcd_frame_buffer[ind+5]=0x00;
	ind+=6;

	for(uint8_t j = 0; j < 5; j++)
	{
		wotch->lcd_frame_buffer[ind+j]=font7x5[5*(temp+16)+j];
	};
	wotch->lcd_frame_buffer[ind+5]=0x00;
	ind+=6;

	number = 10.0*(number - (float)temp);
	temp = (uint8_t)number;
	for(uint8_t j = 0; j < 5; j++)
	{
		wotch->lcd_frame_buffer[ind+j]=font7x5[5*(temp+16)+j];
	};
	wotch->lcd_frame_buffer[ind+5]=0x00;

};

void lcd_menu_arrow(wotch_struct *wotch,uint8_t pos){

	for (uint8_t i=0;i<8;i++)
	{
		if (i==pos+1)
		{
			for (uint8_t j = 0; j < 5; j++)
			wotch->lcd_frame_buffer[128*i+j] = font7x5[150+j];
		}
		else
		{
			for (uint8_t j = 0; j < 8; j++)
			wotch->lcd_frame_buffer[128*i+j] = 0x00;
		}
	}
};

void lcd_draw_sec_bar(wotch_struct *wotch)
{
	uint8_t sec = 0;
	sec = wotch->cur_time.seconds;
	wotch->lcd_frame_buffer[640+2]=0xff;
	wotch->lcd_frame_buffer[640+125]=0xff;
	wotch->lcd_frame_buffer[640+3]=0x81;
	wotch->lcd_frame_buffer[640+124]=0x81;
	for (uint8_t i = 4; i<124; i++)
	{
		if (i<2*sec+4)
			wotch->lcd_frame_buffer[640+i]=0xBD;
		else
			wotch->lcd_frame_buffer[640+i]=0x81;
	};
};
void lcd_put_time_str(wotch_struct *wotch)
{
	uint8_t pos = wotch->current_menu->current_choice;
	uint16_t temp = 3*128+40;
	uint8_t symb = wotch->cur_time.bcd10hours + 0x10;
	for(uint8_t j = 0; j < 5; j++)
	{
		if (pos==0)
			wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
		else
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
	};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd1hours + 0x10;
	for(uint8_t j = 0; j < 5; j++)
	{
		if (pos==1)
			wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
		else
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
	};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = ':' - 0x20;
		for(uint8_t j = 0; j < 5; j++)
		{
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
		};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd10minutes + 0x10;
	for(uint8_t j = 0; j < 5; j++)
	{
		if (pos==2)
			wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
		else
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
	};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd1minutes + 0x10;
		for(uint8_t j = 0; j < 5; j++)
		{
			if (pos==3)
				wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
			else
				wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
		};
	wotch->lcd_frame_buffer[temp+5]=0x00;
};

void lcd_put_date_str(wotch_struct *wotch)
{
	uint8_t pos = wotch->current_menu->current_choice;
	uint16_t temp = 3*128+40;
	uint8_t symb = wotch->cur_time.bcd10date + 0x10;
	for(uint8_t j = 0; j < 5; j++)
	{
		if (pos==0)
			wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
		else
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
	};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd1date + 0x10;
	for(uint8_t j = 0; j < 5; j++)
	{
		if (pos==1)
			wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
		else
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
	};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = ':' - 0x20;
		for(uint8_t j = 0; j < 5; j++)
		{
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
		};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd10months + 0x10;
	for(uint8_t j = 0; j < 5; j++)
	{
		if (pos==2)
			wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
		else
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
	};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd1months + 0x10;
		for(uint8_t j = 0; j < 5; j++)
		{
			if (pos==3)
				wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
			else
				wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
		};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = ':' - 0x20;
		for(uint8_t j = 0; j < 5; j++)
		{
			wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
		};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd10years + 0x10;
		for(uint8_t j = 0; j < 5; j++)
		{
			if (pos==4)
				wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
			else
				wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
		};
	wotch->lcd_frame_buffer[temp+5]=0x00;
	temp+=6;
	symb = wotch->cur_time.bcd1years + 0x10;
		for(uint8_t j = 0; j < 5; j++)
		{
			if (pos==5)
				wotch->lcd_frame_buffer[temp+j]= ~font7x5[5*symb+j];
			else
				wotch->lcd_frame_buffer[temp+j]=font7x5[5*symb+j];
		};
	wotch->lcd_frame_buffer[temp+5]=0x00;
};

void lcd_put_date(wotch_struct *wotch)
{
	lcd_write_menu_line(wotch, (DAY_line[wotch->cur_time.bcdWday]), 0);

};

void lcd_clear(wotch_struct *wotch)
{
	memset(wotch->lcd_frame_buffer,0,1024);
};
