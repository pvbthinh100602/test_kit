################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/ili9341/ili9341.c \
../Drivers/BSP/Components/ili9341/ili9341_reg.c 

OBJS += \
./Drivers/BSP/Components/ili9341/ili9341.o \
./Drivers/BSP/Components/ili9341/ili9341_reg.o 

C_DEPS += \
./Drivers/BSP/Components/ili9341/ili9341.d \
./Drivers/BSP/Components/ili9341/ili9341_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/ili9341/%.o: ../Drivers/BSP/Components/ili9341/%.c Drivers/BSP/Components/ili9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

