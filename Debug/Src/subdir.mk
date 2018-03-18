################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SH1106.c \
../Src/kx023.c \
../Src/lcd.c \
../Src/m41t62.c \
../Src/main.c \
../Src/menu.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c \
../Src/usb.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c \
../Src/wotch.c 

OBJS += \
./Src/SH1106.o \
./Src/kx023.o \
./Src/lcd.o \
./Src/m41t62.o \
./Src/main.o \
./Src/menu.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o \
./Src/usb.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o \
./Src/wotch.o 

C_DEPS += \
./Src/SH1106.d \
./Src/kx023.d \
./Src/lcd.d \
./Src/m41t62.d \
./Src/main.d \
./Src/menu.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d \
./Src/usb.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d \
./Src/wotch.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L433xx -I"C:/watch/watch_lcd_2/Inc" -I"C:/st/STM32Cube_FW_L4_V1.11.0/Middlewares/ST/STemWin/inc" -I"C:/watch/watch_lcd_2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/watch/watch_lcd_2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/watch/watch_lcd_2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/watch/watch_lcd_2/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/watch/watch_lcd_2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/watch/watch_lcd_2/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


