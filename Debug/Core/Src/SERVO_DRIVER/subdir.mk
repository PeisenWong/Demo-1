################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SERVO_DRIVER/servo_driver.c 

OBJS += \
./Core/Src/SERVO_DRIVER/servo_driver.o 

C_DEPS += \
./Core/Src/SERVO_DRIVER/servo_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/SERVO_DRIVER/%.o: ../Core/Src/SERVO_DRIVER/%.c Core/Src/SERVO_DRIVER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"/home/peisen/STM32CubeIDE/workspace_1.8.0/brushless_navi/Core/Src/VESC_CAN" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-SERVO_DRIVER

clean-Core-2f-Src-2f-SERVO_DRIVER:
	-$(RM) ./Core/Src/SERVO_DRIVER/servo_driver.d ./Core/Src/SERVO_DRIVER/servo_driver.o

.PHONY: clean-Core-2f-Src-2f-SERVO_DRIVER

