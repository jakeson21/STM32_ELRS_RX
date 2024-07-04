################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../common/crc.c \
../common/filter.c \
../common/maths.c \
../common/streambuf.c \
../common/time.c 

OBJS += \
./common/crc.o \
./common/filter.o \
./common/maths.o \
./common/streambuf.o \
./common/time.o 

C_DEPS += \
./common/crc.d \
./common/filter.d \
./common/maths.d \
./common/streambuf.d \
./common/time.d 


# Each subdirectory must supply rules for building sources it contributes
common/%.o common/%.su: ../common/%.c common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -DUSE_SERIALRX_CRSF -DSTM32F7 -DUSE_SERIALRX -c -I../Core/Inc -I../ProjDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../ProjDrivers/CMSIS/Device/ST/STM32F7xx/Include -I../ProjDrivers/STM32F7xx_HAL_Driver/Inc -I../ProjDrivers/CMSIS/Include -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/rx" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/common" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/build" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/pg" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/fc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/drivers" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/io" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/config" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-common

clean-common:
	-$(RM) ./common/crc.d ./common/crc.o ./common/crc.su ./common/filter.d ./common/filter.o ./common/filter.su ./common/maths.d ./common/maths.o ./common/maths.su ./common/streambuf.d ./common/streambuf.o ./common/streambuf.su ./common/time.d ./common/time.o ./common/time.su

.PHONY: clean-common

