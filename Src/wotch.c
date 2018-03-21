/*
 * wotch.c
 *
 *  Created on: Feb 3, 2017
 *      Author: gudozhni
 */

#include "wotch.h"
#include <string.h>
#include "menu.h"
#include "sh1106.h"
#include "m41t62.h"
#include "lcd.h"
#include "kx023.h"


extern ADC_HandleTypeDef hadc1;
extern wotch_struct wotch1;

void wotch_init(wotch_struct *wotch)
{
	wotch->vbat = 0.0;
	wotch->autosleep = 0;
	wotch->use_autosleep = 1;
	wotch->acc_int = 0;
	wotch->acc_event = 0;
	wotch->tap_data = 0;
	wotch->use_acc_ui = 0;
	wotch->measure_bat_voltage = 0;
	wotch->button_pressed = 0;
	wotch->display_mode = MAIN_MODE;
	wotch->current_menu = &mainscreen;
	//wotch->current_menu = &main_menu;
	wotch->menu_level = 0;
	wotch->writing_lcd =0;
	wotch->read_time = &m41t62_read_time;
	wotch->write_time = &m41t62_set_time;
	wotch->redraw = &SH1106_write_frame;
	wotch->active_manager = &menu_manager;
	memset(wotch->lcd_frame_buffer,0x00,1024);
};

uint16_t read_bat_voltage(void){
	uint16_t temp;

	GPIOH->BSRR = GPIO_PIN_0;
	HAL_Delay(20);
	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
		Error_Handler();
	else
		temp = HAL_ADC_GetValue(&hadc1);

	GPIOH->BRR = GPIO_PIN_0;
	return temp;
};

void manage_taps_and_tilts(wotch_struct *wotch)
{

};

void manage_buttons(wotch_struct *wotch){
	uint8_t temp=0;
	uint16_t temp1=0;

	//temp1 = wotch->autosleep;
	//wotch->autosleep = 0;


	if (wotch->button_pressed & UR_BUTTON)
	{
		if (wotch->display_mode == MAIN_MODE){
			//go to MENU_MODE
			wotch->display_mode = MENU_MODE;
			//show main menu
			wotch->current_menu->fptr(wotch);
			SH1106_write_frame();
		}
		else if (wotch->display_mode == MENU_MODE)
		{
			//open chosen sub-menu
			temp = wotch->current_menu->current_choice;
			wotch->current_menu = wotch->current_menu->members[temp];
			wotch->current_menu->fptr(wotch);
			wotch->redraw();

		}
		else if (wotch->display_mode == DFU_MODE)
		{
			NVIC_SystemReset();
		}
		else if (wotch->display_mode == TIME_SET_MODE)
		{
			if (wotch->current_menu->current_choice == 3)
				wotch->current_menu->current_choice =0;
			else
				wotch->current_menu->current_choice +=1;
			lcd_put_time_str(wotch);
			wotch->redraw();
		}
		else if (wotch->display_mode == DATE_SET_MODE)
		{
			if (wotch->current_menu->current_choice == 5)
				wotch->current_menu->current_choice =0;
			else
				wotch->current_menu->current_choice +=1;
			lcd_put_date_str(wotch);
			wotch->redraw();
		};
		wotch->autosleep = 0;
		wotch->button_pressed &= ~ UR_BUTTON;
	};

	if (wotch->button_pressed & GPIO_PIN_7)
	{
		wotch->button_pressed &= ~ GPIO_PIN_7;
	};


	if (wotch->button_pressed & T2_BUTTON)
	{
		wotch->autosleep = 0;
		wotch->button_pressed &= ~T2_BUTTON;
	};
	if (wotch->button_pressed & DL_BUTTON)
	{
		if (wotch->display_mode == MAIN_MODE)
		{

		}
		else if (wotch->display_mode == MENU_MODE)
		{
			menu_next(wotch);
			wotch->redraw();
		}
		else if (wotch->display_mode == TIME_SET_MODE)
		{
			switch (wotch->current_menu->current_choice)
			{
			case 0:
				if (wotch->cur_time.bcd10hours==0)
					wotch->cur_time.bcd10hours=2;
				else
					wotch->cur_time.bcd10hours--;
				break;
			case 1:

				if (wotch->cur_time.bcd1hours==0)
					wotch->cur_time.bcd1hours=9;
				else
					wotch->cur_time.bcd1hours--;
				break;
			case 2:
				if (wotch->cur_time.bcd10minutes==0)
					wotch->cur_time.bcd10minutes=5;
				else
					wotch->cur_time.bcd10minutes--;
				break;
			case 3:
				if (wotch->cur_time.bcd1minutes==0)
					wotch->cur_time.bcd1minutes=9;
				else
					wotch->cur_time.bcd1minutes--;
				break;
			};
			lcd_put_time_str(wotch);
			wotch->redraw();
		}
		else if (wotch->display_mode == DATE_SET_MODE)
		{
			switch (wotch->current_menu->current_choice)
			{
			case 0:
				if (wotch->cur_time.bcd10date==0)
					wotch->cur_time.bcd10date=3;
				else
					wotch->cur_time.bcd10date--;
				break;
			case 1:

				if (wotch->cur_time.bcd1date==0)
					wotch->cur_time.bcd1date=9;
				else
					wotch->cur_time.bcd1date--;
				break;
			case 2:
				if (wotch->cur_time.bcd10months==0)
					wotch->cur_time.bcd10months=1;
				else
					wotch->cur_time.bcd10months--;
				break;
			case 3:
				if (wotch->cur_time.bcd1months==0)
					wotch->cur_time.bcd1months=9;
				else
					wotch->cur_time.bcd1months--;
				break;
			case 4:
				if (wotch->cur_time.bcd10years==0)
					wotch->cur_time.bcd10years=9;
				else
					wotch->cur_time.bcd10years--;
				break;
			case 5:
				if (wotch->cur_time.bcd1years==0)
					wotch->cur_time.bcd1years=9;
				else
					wotch->cur_time.bcd1years--;
				break;
			};
			lcd_put_date_str(wotch);
			wotch->redraw();
		};
		wotch->autosleep = 0;
		wotch->button_pressed &= ~DL_BUTTON;
	};
	if (wotch->button_pressed & UL_BUTTON)
	{
		if (wotch->display_mode == MAIN_MODE)
		{

		}
		else if (wotch->display_mode == MENU_MODE)
		{
			menu_prev(wotch);
			wotch->redraw();
		}
		else if (wotch->display_mode == TIME_SET_MODE)
		{
			switch (wotch->current_menu->current_choice)
			{
			case 0:
				wotch->cur_time.bcd10hours++;
				if (wotch->cur_time.bcd10hours==3)
					wotch->cur_time.bcd10hours=0;
				break;
			case 1:
				wotch->cur_time.bcd1hours++;
				if (wotch->cur_time.bcd1hours==10)
					wotch->cur_time.bcd1hours=0;
				break;
			case 2:
				wotch->cur_time.bcd10minutes++;
				if (wotch->cur_time.bcd10minutes==6)
					wotch->cur_time.bcd10minutes=0;
				break;
			case 3:
				wotch->cur_time.bcd1minutes++;
				if (wotch->cur_time.bcd1minutes==10)
					wotch->cur_time.bcd1minutes=0;
				break;
			};
			lcd_put_time_str(wotch);
			wotch->redraw();
		}
		else if (wotch->display_mode == DATE_SET_MODE)
		{
			switch (wotch->current_menu->current_choice)
			{
			case 0:
				wotch->cur_time.bcd10date++;
				if (wotch->cur_time.bcd10date==4)
					wotch->cur_time.bcd10date=0;
				break;
			case 1:
				wotch->cur_time.bcd1date++;
				if (wotch->cur_time.bcd1date==10)
					wotch->cur_time.bcd1date=0;
				break;
			case 2:
				wotch->cur_time.bcd10months++;
				if (wotch->cur_time.bcd10months==2)
					wotch->cur_time.bcd10months=0;
				break;
			case 3:
				wotch->cur_time.bcd1months++;
				if (wotch->cur_time.bcd1months==10)
					wotch->cur_time.bcd1months=0;
				break;
			case 4:
				wotch->cur_time.bcd10years++;
				if (wotch->cur_time.bcd10years==10)
					wotch->cur_time.bcd10years=0;
				break;
			case 5:
				wotch->cur_time.bcd1years++;
				if (wotch->cur_time.bcd1years==10)
					wotch->cur_time.bcd1years=0;
				break;
			};
			lcd_put_date_str(wotch);
			wotch->redraw();
		};
		wotch->autosleep = 0;
		wotch->button_pressed &= ~UL_BUTTON;
	};
	if (wotch->button_pressed & T1_BUTTON)
	{
		wotch->autosleep = 0;
		wotch->button_pressed &= ~T1_BUTTON;
	};

	if (wotch->button_pressed & ACC_INT1_PIN)
	{
		wotch->acc_event = kx023_read_reg(INS2);
		wotch->tap_data = kx023_read_reg(INS1);
		kx023_read_reg(INT_REL);
		if (wotch->display_mode == TAP_TEST_MODE)
		{
			lcd_clear(wotch);
			if ((wotch->acc_event)&TAP_EVENT_SINGLE_TAP)
				lcd_write_char(wotch,0,0,'S');
			if ((wotch->acc_event)&TAP_EVENT_DOUBLE_TAP)
				lcd_write_char(wotch,0,0,'D');
			if ((wotch->tap_data)&0x20)
			{
				lcd_write_char(wotch,0,1,'X');
				lcd_write_char(wotch,0,2,'-');
			};
			if ((wotch->tap_data)&0x10)
			{
				lcd_write_char(wotch,0,1,'X');
				lcd_write_char(wotch,0,2,'+');
			};
			if ((wotch->tap_data)&0x08)
			{
				lcd_write_char(wotch,0,1,'Y');
				lcd_write_char(wotch,0,2,'-');
			};
			if ((wotch->tap_data)&0x04)
			{
				lcd_write_char(wotch,0,1,'Y');
				lcd_write_char(wotch,0,2,'+');
			};
			if ((wotch->tap_data)&0x02)
			{
				lcd_write_char(wotch,0,1,'Z');
				lcd_write_char(wotch,0,2,'-');
			};
			if ((wotch->tap_data)&0x01)
			{
				lcd_write_char(wotch,0,1,'Z');
				lcd_write_char(wotch,0,2,'+');
			};
			wotch->redraw();
		};
		wotch->button_pressed &= ~ACC_INT1_PIN;
	};
};

void show_dfu_message(wotch_struct *wotch)
{
	wotch->active_manager = &dfu_manager;
	lcd_write_menu_line(wotch,&DFU_msg,0);
	for (uint8_t k=1;k<7;k++)
		lcd_write_menu_line(wotch,NULL,k);

	lcd_menu_arrow(wotch,0);

};
void dfu_manager(wotch_struct *wotch){

	if (wotch->button_pressed & UR_BUTTON)
	{
		// here is a response to Up-Right button (ENTER) press
		NVIC_SystemReset();											// do system reset followed by bootloader mode
	};
	if (wotch->button_pressed & DR_BUTTON)
	{
		wotch->autosleep = 0;
		// here is a response to Down-Right button (BACK) press
		wotch->current_menu = wotch->current_menu->parent;			// set parent menu as current menu
		wotch->current_menu->fptr(wotch);							// run associated function (show_menu() or show_main_screen())
		wotch->redraw();											// refresh lcd
		wotch->active_manager = &menu_manager;						// go back to menu manager
		wotch->button_pressed &= ~ DR_BUTTON;						// reset flag for this button
	};
};

void run_tap_app(wotch_struct *wotch){

	lcd_clear(wotch);
	wotch->redraw();
	kx023_TAP_setup();
	wotch->active_manager = acc_tap_app;
	wotch->use_autosleep = 0;
};

void acc_tap_app(wotch_struct *wotch){
// This is a function that implements an accelerometer tap/double_tap feature test. It intercepts the accelerometer interrupt,
// reads the source, and displays the detected event - (S-single,D-double)(x,y,z-axis)(+/- direction)
	if (wotch->button_pressed & ACC_INT1_PIN)
	{
		wotch->acc_event = kx023_read_reg(INS2);
		wotch->tap_data = kx023_read_reg(INS1);
		kx023_read_reg(INT_REL);
		if (wotch->display_mode == TAP_TEST_MODE)
		{
			lcd_clear(wotch);
			if ((wotch->acc_event)&TAP_EVENT_SINGLE_TAP)
				lcd_write_char(wotch,0,0,'S');
			if ((wotch->acc_event)&TAP_EVENT_DOUBLE_TAP)
				lcd_write_char(wotch,0,0,'D');
			if ((wotch->tap_data)&0x20)
			{
				lcd_write_char(wotch,0,1,'X');
				lcd_write_char(wotch,0,2,'-');
			};
			if ((wotch->tap_data)&0x10)
			{
				lcd_write_char(wotch,0,1,'X');
				lcd_write_char(wotch,0,2,'+');
			};
			if ((wotch->tap_data)&0x08)
			{
				lcd_write_char(wotch,0,1,'Y');
				lcd_write_char(wotch,0,2,'-');
			};
			if ((wotch->tap_data)&0x04)
			{
				lcd_write_char(wotch,0,1,'Y');
				lcd_write_char(wotch,0,2,'+');
			};
			if ((wotch->tap_data)&0x02)
			{
				lcd_write_char(wotch,0,1,'Z');
				lcd_write_char(wotch,0,2,'-');
			};
			if ((wotch->tap_data)&0x01)
			{
				lcd_write_char(wotch,0,1,'Z');
				lcd_write_char(wotch,0,2,'+');
			};
			wotch->redraw();
		};
		wotch->button_pressed &= ~ACC_INT1_PIN;
	};

	if (wotch->button_pressed & DR_BUTTON)
	{		// here is a response to Down-Right button (BACK) press. This is the only button being checked in this app to be able to go back to menu
		wotch->autosleep = 0;
		wotch->current_menu = wotch->current_menu->parent;			// set parent menu as current menu
		wotch->current_menu->fptr(wotch);							// run associated function (show_menu() or show_main_screen())
		wotch->redraw();											// refresh lcd
		wotch->active_manager = &menu_manager;						// go back to menu manager
		wotch->use_autosleep = 1;									// allow auto-sleep
		wotch->button_pressed &= ~ DR_BUTTON;						// reset flag for this button
	};
};

void run_tilt_app(wotch_struct *wotch){

	lcd_clear(wotch);
	wotch->redraw();
	kx023_TILT_setup();
	wotch->active_manager = acc_tilt_app;
	wotch->use_autosleep = 0;
};

void acc_tilt_app(wotch_struct *wotch){



	if (wotch->button_pressed & DR_BUTTON)
	{		// here is a response to Down-Right button (BACK) press. This is the only button being checked in this app to be able to go back to menu
		wotch->autosleep = 0;
		wotch->current_menu = wotch->current_menu->parent;			// set parent menu as current menu
		wotch->current_menu->fptr(wotch);							// run associated function (show_menu() or show_main_screen())
		wotch->redraw();											// refresh lcd
		wotch->active_manager = &menu_manager;						// go back to menu manager
		wotch->use_autosleep = 1;									// allow auto-sleep
		wotch->button_pressed &= ~ DR_BUTTON;						// reset flag for this button
	};
};

void menu_manager(wotch_struct *wotch){
	// This function implements navigation through menus. It is called from the main loop if chosen as an active manager
	uint8_t temp=0;

#ifdef USE_ACC_UI
// manage interrupts from accelerometer
	if (wotch->use_acc_ui)
	{


	}
#endif

	if (wotch->button_pressed & UR_BUTTON)
	{
		wotch->autosleep = 0;										// reset autosleep counter
		// here is a response to Up-Right button (ENTER) press
		//if (wotch->menu_level == 0)
		temp = wotch->current_menu->current_choice;
		wotch->current_menu = wotch->current_menu->members[temp];	// set selected menu as current menu
		wotch->current_menu->fptr(wotch);							// run associated function
		wotch->redraw();											// refresh lcd
		wotch->button_pressed &= ~ UR_BUTTON;						// reset flag for this button
	};
	if (wotch->button_pressed & DR_BUTTON)
	{
		wotch->autosleep = 0;
		// here is a response to Down-Right button (BACK) press
		wotch->current_menu = wotch->current_menu->parent;			// set parent menu as current menu
		wotch->current_menu->fptr(wotch);							// run associated function
		wotch->redraw();											// refresh lcd
		wotch->button_pressed &= ~ DR_BUTTON;						// reset flag for this button
	};
	if (wotch->button_pressed & UL_BUTTON)
	{
		wotch->autosleep = 0;
		// here is a response to Up-Left button (UP) press
		menu_prev(wotch);											// select previous menu item
		wotch->redraw();											// refresh lcd
		wotch->button_pressed &= ~ UL_BUTTON;						// reset flag for this button
	};
	if (wotch->button_pressed & DL_BUTTON)
	{
		wotch->autosleep = 0;
		// here is a response to Down-Left button (DOWN) press
		menu_next(wotch);											// select next menu item
		wotch->redraw();											// refresh lcd
		wotch->button_pressed &= ~ DL_BUTTON;						// reset flag for this button
	};
};

void second_int_manager(wotch_struct *wotch){

	if (wotch->button_pressed & GPIO_PIN_8)
	{
		//square wave input 1s period
		wotch->measure_bat_voltage = 1;			// set flag to measure battery voltage every second when active
		wotch->cur_time.seconds++;
		if (wotch->cur_time.seconds > 59)
		{
			wotch->cur_time.seconds = 0;
			wotch->cur_time.minutes++;
			if (wotch->cur_time.minutes > 59)
			{
				wotch->cur_time.minutes = 0;
				wotch->cur_time.hours++;
				if (wotch->cur_time.hours > 23)
				{
					wotch->cur_time.hours = 0;
					wotch->read_time();
				};
			};

		};

		if (wotch->display_mode == MAIN_MODE){
			wotch->read_time();
			lcd_clear(wotch);
			lcd_put_date(wotch);
			lcd_update_time(wotch);
			lcd_draw_sec_bar(wotch);
			wotch->redraw();
		};
		//wotch->autosleep += temp1;
		wotch->button_pressed &= ~GPIO_PIN_8;
	};
}

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

void menu_set_tilt(wotch_struct * wotch)
{

};

void menu_set_tap(wotch_struct * wotch)
{

};
void menu_set_wum(wotch_struct * wotch)
{

};
void menu_set_buf(wotch_struct * wotch)
{

};



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	wotch1.button_pressed |= (uint16_t)GPIO_Pin;
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(UL_LED_PORT,UL_LED);
};

void HAL_SYSTICK_Callback(void)
{
	wotch1.autosleep++;
}

