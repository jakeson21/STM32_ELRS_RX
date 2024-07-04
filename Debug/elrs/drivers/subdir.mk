################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../elrs/drivers/dma.c \
../elrs/drivers/io.c \
../elrs/drivers/resource.c \
../elrs/drivers/serial.c \
../elrs/drivers/serial_uart.c \
../elrs/drivers/serial_uart_pinconfig.c \
../elrs/drivers/system.c 

OBJS += \
./elrs/drivers/dma.o \
./elrs/drivers/io.o \
./elrs/drivers/resource.o \
./elrs/drivers/serial.o \
./elrs/drivers/serial_uart.o \
./elrs/drivers/serial_uart_pinconfig.o \
./elrs/drivers/system.o 

C_DEPS += \
./elrs/drivers/dma.d \
./elrs/drivers/io.d \
./elrs/drivers/resource.d \
./elrs/drivers/serial.d \
./elrs/drivers/serial_uart.d \
./elrs/drivers/serial_uart_pinconfig.d \
./elrs/drivers/system.d 


# Each subdirectory must supply rules for building sources it contributes
elrs/drivers/%.o elrs/drivers/%.su: ../elrs/drivers/%.c elrs/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -DUSE_SERIALRX_CRSF -DSTM32F7 -DUSE_SERIALRX -DUSE_FULL_LL_DRIVER=1 -DSTM32 -c -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Core/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/build" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/common" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/pg" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/fc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/drivers" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/io" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/config" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/rx" -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-elrs-2f-drivers

clean-elrs-2f-drivers:
	-$(RM) ./elrs/drivers/dma.d ./elrs/drivers/dma.o ./elrs/drivers/dma.su ./elrs/drivers/io.d ./elrs/drivers/io.o ./elrs/drivers/io.su ./elrs/drivers/resource.d ./elrs/drivers/resource.o ./elrs/drivers/resource.su ./elrs/drivers/serial.d ./elrs/drivers/serial.o ./elrs/drivers/serial.su ./elrs/drivers/serial_uart.d ./elrs/drivers/serial_uart.o ./elrs/drivers/serial_uart.su ./elrs/drivers/serial_uart_pinconfig.d ./elrs/drivers/serial_uart_pinconfig.o ./elrs/drivers/serial_uart_pinconfig.su ./elrs/drivers/system.d ./elrs/drivers/system.o ./elrs/drivers/system.su

.PHONY: clean-elrs-2f-drivers

