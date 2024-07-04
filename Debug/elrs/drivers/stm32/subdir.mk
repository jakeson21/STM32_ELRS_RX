################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../elrs/drivers/stm32/io_stm32.c \
../elrs/drivers/stm32/rcc_stm32.c \
../elrs/drivers/stm32/serial_uart_hal.c \
../elrs/drivers/stm32/serial_uart_stm32f7xx.c \
../elrs/drivers/stm32/system_stm32f7xx.c 

OBJS += \
./elrs/drivers/stm32/io_stm32.o \
./elrs/drivers/stm32/rcc_stm32.o \
./elrs/drivers/stm32/serial_uart_hal.o \
./elrs/drivers/stm32/serial_uart_stm32f7xx.o \
./elrs/drivers/stm32/system_stm32f7xx.o 

C_DEPS += \
./elrs/drivers/stm32/io_stm32.d \
./elrs/drivers/stm32/rcc_stm32.d \
./elrs/drivers/stm32/serial_uart_hal.d \
./elrs/drivers/stm32/serial_uart_stm32f7xx.d \
./elrs/drivers/stm32/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
elrs/drivers/stm32/%.o elrs/drivers/stm32/%.su: ../elrs/drivers/stm32/%.c elrs/drivers/stm32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -DUSE_SERIALRX_CRSF -DSTM32F7 -DUSE_SERIALRX -DUSE_FULL_LL_DRIVER=1 -DSTM32 -c -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Core/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/build" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/common" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/pg" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/fc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/drivers" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/io" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/config" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/rx" -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-elrs-2f-drivers-2f-stm32

clean-elrs-2f-drivers-2f-stm32:
	-$(RM) ./elrs/drivers/stm32/io_stm32.d ./elrs/drivers/stm32/io_stm32.o ./elrs/drivers/stm32/io_stm32.su ./elrs/drivers/stm32/rcc_stm32.d ./elrs/drivers/stm32/rcc_stm32.o ./elrs/drivers/stm32/rcc_stm32.su ./elrs/drivers/stm32/serial_uart_hal.d ./elrs/drivers/stm32/serial_uart_hal.o ./elrs/drivers/stm32/serial_uart_hal.su ./elrs/drivers/stm32/serial_uart_stm32f7xx.d ./elrs/drivers/stm32/serial_uart_stm32f7xx.o ./elrs/drivers/stm32/serial_uart_stm32f7xx.su ./elrs/drivers/stm32/system_stm32f7xx.d ./elrs/drivers/stm32/system_stm32f7xx.o ./elrs/drivers/stm32/system_stm32f7xx.su

.PHONY: clean-elrs-2f-drivers-2f-stm32

