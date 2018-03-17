/*
 * menu.c
 *
 *  Created on: Feb 10, 2017
 *      Author: gudozhni
 */

#include "menu.h"
#include "wotch.h"
#include "lcd.h"
extern wotch_struct wotch1;

MENU main_menu;
MENU settings_menu;
MENU set_time_menu;
MENU acc_set_menu;
MENU set2_menu;
MENU item1_menu;
MENU item2_menu;
MENU item3_menu;
MENU item4_menu;
MENU dfu_menu;
MENU item6_menu;
MENU apps_menu;

MENU_line DFU_msg = {17, "Press and Hold..."};
MENU_line BAT_msg = {9, "Battery: "};
MENU_line main_line = {4, "Main"};

MENU_line set_line = {8,"Settings"};
MENU_line set_time_line = {8,"Set time"};
MENU_line time_line = {4,"Time"};
MENU_line date_line = {4,"Date"};

MENU_line apps_line = {4,"Apps"};
MENU_line accel_tap_test_line = {15,"Accel. tap test"};
MENU_line accel_tilt_test_line = {16,"Accel. tilt test"};
MENU_line acc_settings_line = {13,"Acc. settings"};


MENU_line alarm_line = {6,"Alarms"};
MENU_line item2_line = {9,"MainItem2"};
MENU_line item3_line = {9,"MainItem3"};
MENU_line item4_line = {9,"MainItem4"};
MENU_line DFU_line = {15,"Firmware update"};
MENU_line STATUS_line = {6,"Status"};

MENU time_menu = { &menu_set_time,4,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&set_time_menu,&time_line};
MENU date_menu = { &menu_set_date,6,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&set_time_menu,&date_line};
MENU set_time_menu = { &show_menu,2,0,{&time_menu,&date_menu,NULL,NULL,NULL,NULL,NULL},&settings_menu,&set_time_line};
MENU acc_set_menu = { &show_menu,0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&settings_menu,&acc_settings_line};

MENU settings_menu = { &show_menu,2,0,{&set_time_menu,&acc_set_menu,NULL,NULL,NULL,NULL,NULL},&main_menu,&set_line};
MENU accel_tap_test = {&run_tap_test, 0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&apps_menu,&accel_tap_test_line};
MENU accel_tilt_test = {&run_tilt_test, 0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&apps_menu,&accel_tilt_test_line};
MENU apps_menu = {&show_menu,3,0,{&accel_tap_test,&accel_tilt_test,NULL,NULL,NULL,NULL,NULL},&main_menu,&apps_line}; // Applications menu
MENU alarms_menu = { &show_menu,2,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&main_menu,&alarm_line}; // Alarms menu
MENU item2_menu = { &show_menu,0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&main_menu,&item2_line};
MENU item3_menu = { &show_menu,0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&main_menu,&item3_line};
MENU item4_menu = { &show_menu,0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&main_menu,&item4_line};
MENU dfu_menu = { &show_dfu_message,0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&main_menu,&DFU_line};
MENU status_menu = { &show_status,0,0,{NULL,NULL,NULL,NULL,NULL,NULL,NULL},&main_menu,&STATUS_line};

MENU main_menu = {&show_menu,4,0,{&settings_menu, &apps_menu,&dfu_menu,&status_menu},NULL,&main_line};

void show_menu(wotch_struct *wotch){

	uint8_t k;
	lcd_clear(wotch);
	for (k = 0;k<wotch->current_menu->length;k++)
		lcd_write_menu_line(wotch,wotch->current_menu->members[k]->title,k);

	lcd_menu_arrow(wotch,wotch->current_menu->current_choice);

};

void menu_next(wotch_struct *wotch){
	uint8_t temp=0;
	temp = wotch->current_menu->length - 1;
	if (wotch->current_menu->current_choice == temp)
		wotch->current_menu->current_choice = 0;
	else
		wotch->current_menu->current_choice += 1;
	lcd_menu_arrow(wotch,wotch->current_menu->current_choice);
};

void menu_prev(wotch_struct *wotch){
	uint8_t temp=0;
	temp = wotch->current_menu->length - 1;
	if (wotch->current_menu->current_choice == 0)
		wotch->current_menu->current_choice = temp;
	else
		wotch->current_menu->current_choice -= 1;
	lcd_menu_arrow(wotch,wotch->current_menu->current_choice);
};

void menu_init(wotch_struct *wotch){

	wotch->current_menu = &main_menu;
};

void test(void){
	show_menu(&wotch1);
	wotch1.current_menu = &main_menu;
};

void show_main_screen(wotch_struct *wotch)
{
	//read current time to update the display
	wotch->read_time(wotch);

	//put the digits and seconds bar
	lcd_clear(wotch);
	lcd_put_date(wotch);
	lcd_update_time(wotch);
	lcd_draw_sec_bar(wotch);
	wotch->redraw();
};

void show_dfu_message(wotch_struct *wotch)
{
	wotch->display_mode = DFU_MODE;
	lcd_write_menu_line(wotch,&DFU_msg,0);
	for (uint8_t k=1;k<7;k++)
		lcd_write_menu_line(wotch,NULL,k);

	lcd_menu_arrow(wotch,0);

};

void show_status(wotch_struct *wotch)
{
	wotch->display_mode = STATUS_MODE;
	lcd_write_menu_line(wotch,&BAT_msg,0);
	for (uint8_t k=1;k<7;k++)
		lcd_write_menu_line(wotch,NULL,k);
	lcd_write_float(wotch, wotch->vbat,12,0);
};

void menu_set_time(wotch_struct * wotch)
{
	lcd_clear(wotch);
	wotch->display_mode = TIME_SET_MODE;
	wotch->read_time(wotch);
	lcd_put_time_str(wotch);

};

void menu_set_date(wotch_struct * wotch)
{
	lcd_clear(wotch);
	wotch->display_mode = DATE_SET_MODE;
	wotch->read_time(wotch);
	lcd_put_date_str(wotch);

};
