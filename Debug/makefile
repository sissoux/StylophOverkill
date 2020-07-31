################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
Stylophone_proto.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Stylophone_proto.list \

OBJCOPY_BIN += \
Stylophone_proto.bin \


# All Target
all: Stylophone_proto.elf secondary-outputs

# Tool invocations
Stylophone_proto.elf: $(OBJS) $(USER_OBJS) C:\Users\adamiens\STM32CubeIDE\workspace_1.3.0\Stylophone_proto\STM32G431KBTX_FLASH.ld
	arm-none-eabi-gcc -o "Stylophone_proto.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\adamiens\STM32CubeIDE\workspace_1.3.0\Stylophone_proto\STM32G431KBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Stylophone_proto.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Stylophone_proto.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Stylophone_proto.list"
	@echo 'Finished building: $@'
	@echo ' '

Stylophone_proto.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Stylophone_proto.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
