################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/pdm_can.c \
../Core/Src/pdm_config.c \
../Core/Src/pdm_driver_control.c \
../Core/Src/pdm_driver_pwm.c \
../Core/Src/pdm_interrupts.c \
../Core/Src/pdm_readings.c \
../Core/Src/pdm_threads.c \
../Core/Src/pdm_variables.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/pdm_can.o \
./Core/Src/pdm_config.o \
./Core/Src/pdm_driver_control.o \
./Core/Src/pdm_driver_pwm.o \
./Core/Src/pdm_interrupts.o \
./Core/Src/pdm_readings.o \
./Core/Src/pdm_threads.o \
./Core/Src/pdm_variables.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/pdm_can.d \
./Core/Src/pdm_config.d \
./Core/Src/pdm_driver_control.d \
./Core/Src/pdm_driver_pwm.d \
./Core/Src/pdm_interrupts.d \
./Core/Src/pdm_readings.d \
./Core/Src/pdm_threads.d \
./Core/Src/pdm_variables.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pdm_can.d ./Core/Src/pdm_can.o ./Core/Src/pdm_can.su ./Core/Src/pdm_config.d ./Core/Src/pdm_config.o ./Core/Src/pdm_config.su ./Core/Src/pdm_driver_control.d ./Core/Src/pdm_driver_control.o ./Core/Src/pdm_driver_control.su ./Core/Src/pdm_driver_pwm.d ./Core/Src/pdm_driver_pwm.o ./Core/Src/pdm_driver_pwm.su ./Core/Src/pdm_interrupts.d ./Core/Src/pdm_interrupts.o ./Core/Src/pdm_interrupts.su ./Core/Src/pdm_readings.d ./Core/Src/pdm_readings.o ./Core/Src/pdm_readings.su ./Core/Src/pdm_threads.d ./Core/Src/pdm_threads.o ./Core/Src/pdm_threads.su ./Core/Src/pdm_variables.d ./Core/Src/pdm_variables.o ./Core/Src/pdm_variables.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

