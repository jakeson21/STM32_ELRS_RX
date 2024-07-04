################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../io/serial.c 

OBJS += \
./io/serial.o 

C_DEPS += \
./io/serial.d 


# Each subdirectory must supply rules for building sources it contributes
io/%.o io/%.su: ../io/%.c io/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -DUSE_SERIALRX_CRSF -DSTM32F7 -DUSE_SERIALRX -c -I../Core/Inc -I../ProjDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../ProjDrivers/CMSIS/Device/ST/STM32F7xx/Include -I../ProjDrivers/STM32F7xx_HAL_Driver/Inc -I../ProjDrivers/CMSIS/Include -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/rx" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/common" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/build" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/pg" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/fc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/drivers" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/io" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/config" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-io

clean-io:
	-$(RM) ./io/serial.d ./io/serial.o ./io/serial.su

.PHONY: clean-io

