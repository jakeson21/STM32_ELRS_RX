################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../elrs/pg/rx.c 

OBJS += \
./elrs/pg/rx.o 

C_DEPS += \
./elrs/pg/rx.d 


# Each subdirectory must supply rules for building sources it contributes
elrs/pg/%.o elrs/pg/%.su elrs/pg/%.cyclo: ../elrs/pg/%.c elrs/pg/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -DUSE_SERIALRX_CRSF -DSTM32F7 -DUSE_SERIALRX -DUSE_FULL_LL_DRIVER=1 -DSTM32 -c -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Core/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/build" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/common" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/pg" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/fc" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/drivers" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/io" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/config" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs" -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/rx" -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Fuguru/STM32CubeIDE/workspace_1.10.1/RC_Test/elrs/msp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-elrs-2f-pg

clean-elrs-2f-pg:
	-$(RM) ./elrs/pg/rx.cyclo ./elrs/pg/rx.d ./elrs/pg/rx.o ./elrs/pg/rx.su

.PHONY: clean-elrs-2f-pg

