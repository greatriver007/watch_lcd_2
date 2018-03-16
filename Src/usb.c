/*
 * usb.c
 *
 *  Created on: Jan 9, 2017
 *      Author: gudozhni
 */

#include "usb.h"

char RxBuffer[RxBufferSize];
volatile uint16_t readIndex = RxBufferSize;
volatile uint16_t RxFifoIndexStart = 0;
volatile uint16_t RxFifoIndexEnd = 0;
volatile uint8_t usb_command_received = 0;

uint8_t USB_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
     volatile uint32_t counter = 0;

     while(counter < *Len){
          RxBuffer[RxFifoIndexEnd ] = Buf[counter];
          if (RxBuffer[RxFifoIndexEnd] == 13)
        	 usb_command_received = 1;
          counter++, RxFifoIndexEnd++;
          if(RxFifoIndexEnd  == RxBufferSize){
        	   //RxFifoIndexStart = 0;
               RxFifoIndexEnd  = 0;
          	  }
          }

  return (USBD_OK);
}

uint8_t USB_decode_command(void)
{
	uint8_t addr;
	uint8_t temp = 0;
	uint8_t err = 0;
	uint8_t v_packet[2];
	uint16_t pulse = 0;

	addr = RxBuffer[RxFifoIndexStart+1];
	if (RxBuffer[RxFifoIndexStart+7]!=13)
	{
		err |= 2;
		RxFifoIndexStart = 0;
		RxFifoIndexEnd  = 0;
		usb_command_received = 0;
		return err;
	}
	else
	{	// look-up for command code
		switch (RxBuffer[RxFifoIndexStart]){

		case 'P': //PWM value

			break;
		case 'B': // configure bimorphs PWM value
			pulse = ((uint16_t)RxBuffer[RxFifoIndexStart+2] << 8) + (uint16_t)RxBuffer[RxFifoIndexStart+3];
			if (addr == 1)
				TIM2->CCR1 = pulse;
			else if (addr == 2)
				TIM2->CCR2 = pulse;
			else if (addr == 3)
				TIM2->CCR3 = pulse;
			else if (addr == 4)
				TIM2->CCR4 = pulse;
			break;
		case 'M': // configure mirror PWM value
			pulse = ((uint16_t)RxBuffer[RxFifoIndexStart+2] << 8) + (uint16_t)RxBuffer[RxFifoIndexStart+3];
			TIM1->CCR1 = pulse;
			break;
		case 'L': // configure laser PWM value
			pulse = ((uint16_t)RxBuffer[RxFifoIndexStart+2] << 8) + (uint16_t)RxBuffer[RxFifoIndexStart+3];
			TIM1->CCR2 = pulse;
			break;

		case 'U': // all taxels up

			break;
		case 'D': // all taxels down

			break;
		case 'R': // send current matrix (24 bytes)
			//CDC_Transmit_FS(bp1.matrix_old, 24);
			break;
		case 'V':  // DAC offset calibration

			break;
		case 'G': //DAC gain calibration, not used

			break;
		case 'H': //input offsets calibration, make sure NO connectors are plugged in

			break;

		case 'K': //input gain calibration using precise 100Vrms or 5Arms AC sources

			break;
		case 'E': //update matrix

			break;
		case 'F': //refresh matrix

			break;
		case 'O': //off
			//HAL_GPIO_WritePin(POWER_LOCK_PORT, POWER_LOCK_PIN,0);
			break;

		default:
			err |= 1;
			//return err;
		};
	};
	RxFifoIndexStart +=8;
	if (RxFifoIndexStart>RxBufferSize-1)
		{
			RxFifoIndexStart -= RxBufferSize;
		};
	if (RxFifoIndexStart == RxFifoIndexEnd)
		{
			usb_command_received = 0;
			RxFifoIndexStart = 0;
			RxFifoIndexEnd = 0;
		};
	return err;
}
