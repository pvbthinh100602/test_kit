################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/button.c \
../Core/Src/dht11.c \
../Core/Src/ds3231.c \
../Core/Src/global.c \
../Core/Src/i2c.c \
../Core/Src/lcd.c \
../Core/Src/led7.c \
../Core/Src/main.c \
../Core/Src/pwm.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/timer.c \
../Core/Src/uart.c \
../Core/Src/utils.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/button.o \
./Core/Src/dht11.o \
./Core/Src/ds3231.o \
./Core/Src/global.o \
./Core/Src/i2c.o \
./Core/Src/lcd.o \
./Core/Src/led7.o \
./Core/Src/main.o \
./Core/Src/pwm.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/timer.o \
./Core/Src/uart.o \
./Core/Src/utils.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/button.d \
./Core/Src/dht11.d \
./Core/Src/ds3231.d \
./Core/Src/global.d \
./Core/Src/i2c.d \
./Core/Src/lcd.d \
./Core/Src/led7.d \
./Core/Src/main.d \
./Core/Src/pwm.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/timer.d \
./Core/Src/uart.d \
./Core/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

