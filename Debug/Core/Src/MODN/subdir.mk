################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MODN/MODN.c 

OBJS += \
./Core/Src/MODN/MODN.o 

C_DEPS += \
./Core/Src/MODN/MODN.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MODN/%.o: ../Core/Src/MODN/%.c Core/Src/MODN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/home/peisen/STM32CubeIDE/workspace_1.8.0/brushless_navi/Core/Src/VESC_CAN" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MODN

clean-Core-2f-Src-2f-MODN:
	-$(RM) ./Core/Src/MODN/MODN.d ./Core/Src/MODN/MODN.o

.PHONY: clean-Core-2f-Src-2f-MODN

