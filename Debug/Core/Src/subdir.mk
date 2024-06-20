################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/pdm_can.c \
../Core/Src/pdm_config.c \
../Core/Src/pdm_driver_control.c \
../Core/Src/pdm_driver_pwm.c \
../Core/Src/pdm_interrupts.c \
../Core/Src/pdm_readings.c \
../Core/Src/pdm_variables.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/pdm_can.o \
./Core/Src/pdm_config.o \
./Core/Src/pdm_driver_control.o \
./Core/Src/pdm_driver_pwm.o \
./Core/Src/pdm_interrupts.o \
./Core/Src/pdm_readings.o \
./Core/Src/pdm_variables.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/pdm_can.d \
./Core/Src/pdm_config.d \
./Core/Src/pdm_driver_control.d \
./Core/Src/pdm_driver_pwm.d \
./Core/Src/pdm_interrupts.d \
./Core/Src/pdm_readings.d \
./Core/Src/pdm_variables.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pdm_can.cyclo ./Core/Src/pdm_can.d ./Core/Src/pdm_can.o ./Core/Src/pdm_can.su ./Core/Src/pdm_config.cyclo ./Core/Src/pdm_config.d ./Core/Src/pdm_config.o ./Core/Src/pdm_config.su ./Core/Src/pdm_driver_control.cyclo ./Core/Src/pdm_driver_control.d ./Core/Src/pdm_driver_control.o ./Core/Src/pdm_driver_control.su ./Core/Src/pdm_driver_pwm.cyclo ./Core/Src/pdm_driver_pwm.d ./Core/Src/pdm_driver_pwm.o ./Core/Src/pdm_driver_pwm.su ./Core/Src/pdm_interrupts.cyclo ./Core/Src/pdm_interrupts.d ./Core/Src/pdm_interrupts.o ./Core/Src/pdm_interrupts.su ./Core/Src/pdm_readings.cyclo ./Core/Src/pdm_readings.d ./Core/Src/pdm_readings.o ./Core/Src/pdm_readings.su ./Core/Src/pdm_variables.cyclo ./Core/Src/pdm_variables.d ./Core/Src/pdm_variables.o ./Core/Src/pdm_variables.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

