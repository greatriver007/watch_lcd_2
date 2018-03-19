/*
 * menu.h
 *
 *  Created on: Feb 10, 2017
 *      Author: gudozhni
 */

#ifndef MENU_H_
#define MENU_H_

#include "stm32l4xx_hal.h"
#include <string.h>
#include "wotch.h"

typedef struct MENU_line{
	uint8_t length;
	char str[];
}MENU_line;

typedef struct MENU{
	void (*fptr)();
	uint8_t length;
	uint8_t current_choice;
	struct MENU* members[7];
	struct MENU* parent;
	MENU_line* title;

}MENU;


void menu_manager(wotch_struct *wotch);
void show_menu(wotch_struct *wotch);
void menu_next(wotch_struct *wotch);
void menu_prev(wotch_struct *wotch);
void menu_init(wotch_struct *wotch);
void show_main_screen(wotch_struct *wotch);
void show_dfu_message(wotch_struct *wotch);
void show_status(wotch_struct *wotch);
void menu_set_time(wotch_struct * wotch);
void menu_set_date(wotch_struct * wotch);
void menu_set_tilt(wotch_struct * wotch);
void menu_set_tap(wotch_struct * wotch);
void menu_set_wum(wotch_struct * wotch);
void menu_set_buf(wotch_struct * wotch);
void test(void);
#endif /* MENU_H_ */
