################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/SSD1963_STM32_Driver/SSD1963.c \
../Core/SSD1963_STM32_Driver/SSD1963_api.c 

OBJS += \
./Core/SSD1963_STM32_Driver/SSD1963.o \
./Core/SSD1963_STM32_Driver/SSD1963_api.o 

C_DEPS += \
./Core/SSD1963_STM32_Driver/SSD1963.d \
./Core/SSD1963_STM32_Driver/SSD1963_api.d 


# Each subdirectory must supply rules for building sources it contributes
Core/SSD1963_STM32_Driver/%.o Core/SSD1963_STM32_Driver/%.su Core/SSD1963_STM32_Driver/%.cyclo: ../Core/SSD1963_STM32_Driver/%.c Core/SSD1963_STM32_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-SSD1963_STM32_Driver

clean-Core-2f-SSD1963_STM32_Driver:
	-$(RM) ./Core/SSD1963_STM32_Driver/SSD1963.cyclo ./Core/SSD1963_STM32_Driver/SSD1963.d ./Core/SSD1963_STM32_Driver/SSD1963.o ./Core/SSD1963_STM32_Driver/SSD1963.su ./Core/SSD1963_STM32_Driver/SSD1963_api.cyclo ./Core/SSD1963_STM32_Driver/SSD1963_api.d ./Core/SSD1963_STM32_Driver/SSD1963_api.o ./Core/SSD1963_STM32_Driver/SSD1963_api.su

.PHONY: clean-Core-2f-SSD1963_STM32_Driver

