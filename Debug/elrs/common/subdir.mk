################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../elrs/common/crc.c \
../elrs/common/filter.c \
../elrs/common/maths.c \
../elrs/common/streambuf.c \
../elrs/common/time.c 

OBJS += \
./elrs/common/crc.o \
./elrs/common/filter.o \
./elrs/common/maths.o \
./elrs/common/streambuf.o \
./elrs/common/time.o 

C_DEPS += \
./elrs/common/crc.d \
./elrs/common/filter.d \
./elrs/common/maths.d \
./elrs/common/streambuf.d \
./elrs/common/time.d 


# Each subdirectory must supply rules for building sources it contributes
elrs/common/%.o elrs/common/%.su elrs/common/%.cyclo: ../elrs/common/%.c elrs/common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -DUSE_SERIALRX_CRSF -DSTM32F7 -DUSE_SERIALRX -DUSE_FULL_LL_DRIVER=1 -DSTM32 -c -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Core/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/build" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/common" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/rx" -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-elrs-2f-common

clean-elrs-2f-common:
	-$(RM) ./elrs/common/crc.cyclo ./elrs/common/crc.d ./elrs/common/crc.o ./elrs/common/crc.su ./elrs/common/filter.cyclo ./elrs/common/filter.d ./elrs/common/filter.o ./elrs/common/filter.su ./elrs/common/maths.cyclo ./elrs/common/maths.d ./elrs/common/maths.o ./elrs/common/maths.su ./elrs/common/streambuf.cyclo ./elrs/common/streambuf.d ./elrs/common/streambuf.o ./elrs/common/streambuf.su ./elrs/common/time.cyclo ./elrs/common/time.d ./elrs/common/time.o ./elrs/common/time.su

.PHONY: clean-elrs-2f-common

