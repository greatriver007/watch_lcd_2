/*
 * wotch.h
 *
 *  Created on: Feb 2, 2017
 *      Author: OG
 */

#ifndef WOTCH_H_
#define WOTCH_H_

#include "stm32l4xx_hal.h"
//#include "menu.h"

#define VERSION 1

#define USE_STOP_MODE 0

//#define USE_ACC_UI

#define USE_ACCEL_TAP	1
#define USE_ACCEL_TILT	1
#define USE_ACCEL_WOM	1
//Display modes
#define MAIN_MODE 1
#define MENU_MODE 2
#define DFU_MODE 3
#define TIME_SET_MODE 4
#define DATE_SET_MODE 5
#define STATUS_MODE	6
#define TAP_TEST_MODE 7
#define OTHER_MODE 0

//LEDs
#define UL_LED		GPIO_PIN_14 //GPIOB
#define UL_LED_PORT		GPIOB
#define UR_LED		GPIO_PIN_14 //GPIOC
#define UR_LED_PORT		GPIOC
#define DR_LED		GPIO_PIN_4 //GPIOA
#define DR_LED_PORT		GPIOA

//buttons
#define UL_BUTTON	GPIO_PIN_13 //GPIOB
#define UR_BUTTON	GPIO_PIN_3	//GPIOH
#define DL_BUTTON	GPIO_PIN_12 //GPIOB
#define DR_BUTTON	GPIO_PIN_0 //GPIOB
#define T1_BUTTON	GPIO_PIN_15 //GPIOB
#define T2_BUTTON	GPIO_PIN_9 //GPIOB
#define T3_BUTTON	GPIO_PIN_8 //GPIOB, no EXTI, taken by GPIOA8 - RTC interrupt line
#define T4_BUTTON	GPIO_PIN_13 //GPIOC, no EXTI, taken by GPIOB13 - UL_BUTTON

//outputs
#define PIEZO_OUT	GPIO_PIN_1 //GPIOA
#define OP_AMP_EN	GPIO_PIN_0 //GPIOH

//inputs

// #define

typedef struct time_struct{

	uint8_t bcd10years;
	uint8_t bcd1years;
	uint8_t bcd10months;
	uint8_t bcd1months;
	uint8_t bcd10date;
	uint8_t bcd1date;
	uint8_t bcdWday;
	uint8_t bcd10hours;
	uint8_t bcd1hours;

	uint8_t bcd10minutes;
	uint8_t bcd1minutes;

	uint8_t bcd10seconds;
	uint8_t bcd1seconds;

	uint16_t year;
	uint8_t month;
	uint8_t date;
	uint8_t day;
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;



}time_struct;

typedef struct wotch_struct{
	uint8_t display_mode;
	uint8_t writing_lcd;
	uint16_t button_pressed;
	time_struct prev_time;
	time_struct cur_time;
	uint16_t autosleep;
	uint8_t use_autosleep;
	uint8_t acc_int;
	uint8_t acc_event;
	uint8_t tap_data;
	uint8_t use_acc_ui;
	uint8_t measure_bat_voltage;
	float vbat;
	uint8_t lcd_frame_buffer[1024];
	struct MENU * current_menu;
	uint8_t menu_level;
	void (*read_time)();
	void (*write_time)();
	void (*active_manager)();
	int (*redraw)();


} wotch_struct;

void wotch_init(wotch_struct *wotch);

uint16_t read_bat_voltage(void);

void menu_manager(wotch_struct *wotch);

void show_dfu_message(wotch_struct *wotch);
void dfu_manager(wotch_struct *wotch);
void manage_buttons(wotch_struct *wotch);
void manage_taps_and_tilts(wotch_struct *wotch);
void run_tap_app(wotch_struct *wotch);
void acc_tap_app(wotch_struct *wotch);
void run_tilt_app(wotch_struct *wotch);
void acc_tilt_app(wotch_struct *wotch);
void show_status(wotch_struct *wotch);
void menu_set_time(wotch_struct * wotch);
void menu_set_date(wotch_struct * wotch);
void menu_set_tilt(wotch_struct * wotch);
void menu_set_tap(wotch_struct * wotch);
void menu_set_wum(wotch_struct * wotch);
void menu_set_buf(wotch_struct * wotch);

void second_int_manager(wotch_struct *wotch);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_SYSTICK_Callback(void);

#endif /* WOTCH_H_ */
