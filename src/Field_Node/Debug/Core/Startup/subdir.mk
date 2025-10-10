################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32wl55jcix.s 

OBJS += \
./Core/Startup/startup_stm32wl55jcix.o 

S_DEPS += \
./Core/Startup/startup_stm32wl55jcix.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Radio" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Core/Inc" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Utils" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Utils/misc" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Utils/conf" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Drivers/BSP/STM32WLxx_Nucleo" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Drivers/CMSIS" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Drivers/STM32WLxx_HAL_Driver" -I"C:/GitHub/stm32-lora-telemetry-relay/src/Field_Node/Core/Inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32wl55jcix.d ./Core/Startup/startup_stm32wl55jcix.o

.PHONY: clean-Core-2f-Startup

