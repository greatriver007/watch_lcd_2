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

extern MENU mainscreen;
extern MENU main_menu;
extern MENU settings_menu;
extern MENU set_time_menu,time_menu,date_menu;
extern MENU acc_set_menu,tilt_set_menu,tap_set_menu,wum_set_menu,buf_set_menu;
extern MENU dfu_menu;
extern MENU apps_menu, accel_tap_test,accel_tilt_test;
extern MENU status_menu;

extern MENU_line BAT_msg;
extern MENU_line DFU_msg;
extern MENU_line STATUS_msg;

void show_menu(wotch_struct *wotch);
void menu_next(wotch_struct *wotch);
void menu_prev(wotch_struct *wotch);
void show_main_screen(wotch_struct *wotch);
void test(void);
#endif /* MENU_H_ */
