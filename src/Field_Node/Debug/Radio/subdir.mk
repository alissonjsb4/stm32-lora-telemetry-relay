################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Radio/radio_board_if.c \
../Radio/radio_driver.c 

OBJS += \
./Radio/radio_board_if.o \
./Radio/radio_driver.o 

C_DEPS += \
./Radio/radio_board_if.d \
./Radio/radio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Radio/%.o Radio/%.su Radio/%.cyclo: ../Radio/%.c Radio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/Field_Node/Radio" -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/Field_Node/Utils/misc" -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/Field_Node/Utils/conf" -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/Field_Node/Drivers/BSP/STM32WLxx_Nucleo" -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/Field_Node/Utils" -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/Field_Node/Drivers/CMSIS" -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/Field_Node/Drivers/STM32WLxx_HAL_Driver" -I"/home/master/Desktop/stm32-lora-telemetry-relay/src/common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Radio

clean-Radio:
	-$(RM) ./Radio/radio_board_if.cyclo ./Radio/radio_board_if.d ./Radio/radio_board_if.o ./Radio/radio_board_if.su ./Radio/radio_driver.cyclo ./Radio/radio_driver.d ./Radio/radio_driver.o ./Radio/radio_driver.su

.PHONY: clean-Radio

