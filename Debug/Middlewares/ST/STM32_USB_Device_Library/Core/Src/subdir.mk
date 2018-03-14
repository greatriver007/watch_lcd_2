################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.o: ../Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L433xx -I"C:/Users/gudozhni/Dropbox/Work/watch/watch_lcd_2/Code/watch_short/Inc" -I"C:/st/STM32Cube_FW_L4_V1.11.0/Middlewares/ST/STemWin/inc" -I"C:/Users/gudozhni/Dropbox/Work/watch/watch_lcd_2/Code/watch_short/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/gudozhni/Dropbox/Work/watch/watch_lcd_2/Code/watch_short/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/gudozhni/Dropbox/Work/watch/watch_lcd_2/Code/watch_short/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/gudozhni/Dropbox/Work/watch/watch_lcd_2/Code/watch_short/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/gudozhni/Dropbox/Work/watch/watch_lcd_2/Code/watch_short/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/gudozhni/Dropbox/Work/watch/watch_lcd_2/Code/watch_short/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


