################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/platform.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/platform.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/platform.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -DUSE_SERIALRX_CRSF -DSTM32F7 -DUSE_SERIALRX -DUSE_FULL_LL_DRIVER=1 -DSTM32 -c -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Core/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/build" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/common" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/rx" -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/platform.cyclo ./Core/Src/platform.d ./Core/Src/platform.o ./Core/Src/platform.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src

