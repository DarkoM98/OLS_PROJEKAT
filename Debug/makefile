################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/Windows/subdir.mk
-include Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Drivers/Drajveri_Ekran_Tac/Sors/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
OlsProjectCO23Click.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
OlsProjectCO23Click.list \

OBJCOPY_HEX += \
OlsProjectCO23Click.hex \


# All Target
all: OlsProjectCO23Click.elf secondary-outputs

# Tool invocations
OlsProjectCO23Click.elf: $(OBJS) $(USER_OBJS) E:\E1\MAS\OLS\Projekat\OLSPOJECT\OlsProjectCO23Click\STM32F746ZGTX_FLASH.ld
	arm-none-eabi-gcc -o "OlsProjectCO23Click.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"E:\E1\MAS\OLS\Projekat\OLSPOJECT\OlsProjectCO23Click\STM32F746ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="OlsProjectCO23Click.map" -Wl,--gc-sections -static -L../STemWinLib/Lib --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

OlsProjectCO23Click.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "OlsProjectCO23Click.list"
	@echo 'Finished building: $@'
	@echo ' '

OlsProjectCO23Click.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "OlsProjectCO23Click.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
