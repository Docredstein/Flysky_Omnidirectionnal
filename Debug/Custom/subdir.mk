################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Custom/IBus.c \
../Custom/Omnidirection.c \
../Custom/Sabertooth.c 

OBJS += \
./Custom/IBus.o \
./Custom/Omnidirection.o \
./Custom/Sabertooth.o 

C_DEPS += \
./Custom/IBus.d \
./Custom/Omnidirection.d \
./Custom/Sabertooth.d 


# Each subdirectory must supply rules for building sources it contributes
Custom/%.o Custom/%.su Custom/%.cyclo: ../Custom/%.c Custom/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/cubeIDE/Omnidirectionnnel/Custom" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Custom

clean-Custom:
	-$(RM) ./Custom/IBus.cyclo ./Custom/IBus.d ./Custom/IBus.o ./Custom/IBus.su ./Custom/Omnidirection.cyclo ./Custom/Omnidirection.d ./Custom/Omnidirection.o ./Custom/Omnidirection.su ./Custom/Sabertooth.cyclo ./Custom/Sabertooth.d ./Custom/Sabertooth.o ./Custom/Sabertooth.su

.PHONY: clean-Custom

