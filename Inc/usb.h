/*
 * usb.h
 *
 *  Created on: Jan 9, 2017
 *      Author: gudozhni
 */

#ifndef USB_H_
#define USB_H_

#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define RxBufferSize 128
extern volatile uint8_t usb_command_received;

uint8_t USB_Receive_FS(uint8_t* Buf, uint32_t *Len);
uint8_t USB_decode_command(void);
#endif /* USB_H_ */
