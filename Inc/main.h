/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
//PA0 - ADC1_IN5, battery voltage measurement
//PA1 - TIM2_CH2, piezo element drive
//PA2 -
//PA3 -
//PA4 - GPIO_Out, DR_LED, lower right LED
//PA5 - SPI1_SCK, lcd SPI clock
//PA6 -
//PA7 - SPI1_MOSI, lcd SPI output
//PA8 - GPIO_In, interrupt line from RTC (alarm?), pullup?
//PA9 - I2C1_SCL
//PA10- I2C1_SDA
//PA11 - USB_DM
//PA12 - USB_DP
//PA13 - SWDIO
//PA14 - SWCLK
//PA15 - GPIO_Out, SPI3_CS, accel chip select

//PB0 - EXTI0, DR_BUTTON, lower right button, "Back" button
//PB1 -
//PB2 - GPIO_Out, LCD_CD_PIN, lcd command/data pin
//PB3 - SPI3_SCK, accel SPI clock
//PB4 - SPI3_MISO, accel SPI input
//PB5 - SPI3_MOSI, accel SPI output
//PB6 - EXTI6, accel INT1
//PB7 - EXTI7, accel INT2
//PB8 - EXTI8, square wave line from RTC (1s intervals)
//PB9 - EXTI9
//PB10- GPIO_Out, LCD_RST_PIN, lcd reset line
//PB11 - GPIO_Out, LCD_CS_PIN, lcd chip select
//PB12 - EXTI12, DL_BUTTON, lower left button, "Down" button
//PB13 - EXTI13, UL_BUTTON, upper left button, "Up" button
//PB14 - GPIO_Out, UL_LED, upper left LED
//PB15 - EXTI15, T1_BUTTON, first top button (not used, not mounted)

//PC13 - GPIO_In, T4_BUTTON
//PC14 - GPIO_Out, UR_LED, upper right LED
//PC15 -

//PH0 - GPIO_Out, enable line for OpAmp for voltage measurement
//PH1 -


/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
