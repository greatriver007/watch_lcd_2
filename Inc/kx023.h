/*
 * kx023.h
 *
 *  Created on: Jan 28, 2017
 *      Author: gudozhni
 */

#ifndef KX023_H_
#define KX023_H_

#include "stm32l4xx_hal.h"



#define ACC_CS_PIN 0x8000  //GPIOA
#define ACC_INT1_PIN GPIO_PIN_6  //GPIOB
#define ACC_INT2_PIN GPIO_PIN_7	 //GPIOB

// KX023 registers
#define		XHPL			0x00
#define		XHPH        	0x01
#define		YHPL        	0x02
#define		YHPH        	0x03
#define		ZHPL        	0x04
#define		ZHPH        	0x05
#define		XOUTL       	0x06
#define		XOUTH       	0x07
#define		YOUTL       	0x08
#define		YOUTH       	0x09
#define		ZOUTL       	0x0A
#define		ZOUTH       	0x0B
#define		COTR        	0x0C
#define		WHOAMI			0x0F
#define		TSCP        	0x10
#define		TSPP        	0x11
#define		INS1        	0x12
#define		INS2        	0x13
#define		INS3        	0x14
#define		STAT        	0x15
#define		INT_REL       	0x17
#define		CNTL1       	0x18
#define		CNTL2       	0x19
#define		CNTL3       	0x1A
#define		ODCNTL       	0x1B
#define		INC1        	0x1C
#define		INC2        	0x1D
#define		INC3        	0x1E
#define		INC4        	0x1F
#define		INC5        	0x20
#define		INC6			0x21
#define		TILT_TIMER      0x22
#define		WUFC            0x23
#define		TDTRC           0x24
#define		TDTC            0x25
#define		TTH             0x26
#define		TTL             0x27
#define		FTD             0x28
#define		STD             0x29
#define		TLT             0x2A
#define		TWS             0x2B
#define		ATH             0x30
#define		TILT_ANGLE_LL   0x32
#define		TILT_ANGLE_HL   0x33
#define		HYST_SET        0x34
#define		LP_CNTL         0x35
#define		BUF_CNTL1       0x3A
#define		BUF_CNTL2       0x3B
#define		BUF_STATUS_1    0x3C
#define		BUF_STATUS_2    0x3D
#define		BUF_CLEAR       0x3E
#define		BUF_READ        0x3F
#define		SELF_TEST       0x60

//bits CNTL1
#define		RUN			0x80
#define		HRES_EN			0x40
#define		DRDY_EN			0x20
#define		MAX_2G			0x00
#define		MAX_4G			0x08
#define		MAX_8G			0x10
#define		TAP_EN			0x04
#define		WUM_EN			0x02
#define		TILT_EN			0x01
// bits INC1
#define		INT1_EN			0x20
#define		HIGH_POL		0x10
#define		INT_LATCH		0x00
#define		INT_PULSE		0x08
// bits INS2
#define		TAP_EVENT_NO_TAP 0x00
#define		TAP_EVENT_SINGLE_TAP 0x04
#define		TAP_EVENT_DOUBLE_TAP 0x08

typedef struct acc_struct{
	uint16_t filt_xdata;
	uint16_t filt_ydata;
	uint16_t filt_zdata;
	uint16_t xdata;
	uint16_t ydata;
	uint16_t zdata;
	uint8_t curr_tilt;	//TSCP
	uint8_t prev_tilt;	//TSPP
	uint8_t tap_dir;	//INS1
	uint8_t int_source;	//INS2
	uint8_t wum_dir;	//INS3
	uint8_t status; //STATUS
	uint8_t ctl1;   //CNTL1
	uint8_t ctl2;   //CNTL2
	uint8_t ctl3;   //CNTL3
	uint8_t out_ctl; //ODCNTL
	uint8_t tilt_timer; //TILT_TIMER
	uint8_t WUM_mask; //WUFC
	uint8_t s_tap_en; //TDTRC
	uint8_t d_tap_en; //TDTRC
	uint8_t delta_tap; //TDTC (0x78)
	uint8_t tap_level_high_thr; //TTH (0xCB)
	uint8_t tap_level_low_thr; //TTL (0x1A)
	uint8_t tap_dur_low_thr; //FTD (0xA2)
	uint8_t tap_dur_high_thr; //FTD
	uint8_t both_taps_length; //STD (0x24)
	uint8_t tap_scan_length; //TLT (0x28)
	uint8_t max_tap_length; //TWS (0xA0)
	uint8_t WUM_thr; //ATH (0x08)
	uint8_t tilt_low_thr; //TILT_ANGLE_LL
	uint8_t tilt_high_thr; //TILT_ANGLE_HL
	uint8_t nsamp_avg; //LP_CNTL (0x4B)
	uint8_t buf_thr; //BUF_CNTL1
	uint8_t buf_level; //BUF_STATUS_1

}acc_struct;

int kx023_init(void);
int kx023_command(uint8_t addr, uint8_t val);
uint8_t kx023_read_reg(uint8_t addr);
void kx023_int1_config(uint8_t int_en, uint8_t int_pol, uint8_t int_mask);
void kx023_int2_config(uint8_t int_en, uint8_t int_pol, uint8_t int_mask);
void kx023_buf_config(uint8_t buf_en, uint8_t watermark_level, uint8_t resolution, uint8_t buf_mode);
void kx023_standby(void);
void kx023_wake(void);

void kx023_WUM_setup(void);
void kx023_TAP_setup(void);
void kx023_TILT_setup(void);

#endif /* KX023_H_ */
