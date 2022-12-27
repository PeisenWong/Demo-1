################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/RGB/rgb.c 

OBJS += \
./Core/Src/RGB/rgb.o 

C_DEPS += \
./Core/Src/RGB/rgb.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/RGB/%.o: ../Core/Src/RGB/%.c Core/Src/RGB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/home/peisen/STM32CubeIDE/workspace_1.8.0/brushless_navi/Core/Src/VESC_CAN" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-RGB

clean-Core-2f-Src-2f-RGB:
	-$(RM) ./Core/Src/RGB/rgb.d ./Core/Src/RGB/rgb.o

.PHONY: clean-Core-2f-Src-2f-RGB

