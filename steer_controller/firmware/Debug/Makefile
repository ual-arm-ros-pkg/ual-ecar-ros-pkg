################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../common


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../common/adc_internal.cpp \
../common/gpio.cpp \
../common/leds.cpp \
../common/millis_timer.cpp \
../common/pwm.cpp \
../common/spi.cpp \
../common/uart.cpp \
../main.cpp \
../mod_dac_max5500.cpp \
../process_internal_adc.cpp \
../process_command.cpp \
../mod_ems22a.cpp \
../process_encoder.cpp \
../process_steer_controller.cpp \
../serial.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
common/adc_internal.o \
common/gpio.o \
common/leds.o \
common/millis_timer.o \
common/pwm.o \
common/spi.o \
common/uart.o \
main.o \
mod_dac_max5500.o \
process_internal_adc.o \
process_command.o \
mod_ems22a.o \
process_encoder.o \
process_steer_controller.o \
serial.o

OBJS_AS_ARGS +=  \
common/adc_internal.o \
common/gpio.o \
common/leds.o \
common/millis_timer.o \
common/pwm.o \
common/spi.o \
common/uart.o \
main.o \
mod_dac_max5500.o \
process_internal_adc.o \
process_command.o \
mod_ems22a.o \
process_encoder.o \
process_steer_controller.o \
serial.o

C_DEPS +=  \
common/adc_internal.d \
common/gpio.d \
common/leds.d \
common/millis_timer.d \
common/pwm.d \
common/spi.d \
common/uart.d \
main.d \
mod_dac_max5500.d \
process_internal_adc.d \
process_command.d \
mod_ems22a.d \
process_encoder.d \
process_steer_controller.d \
serial.d

C_DEPS_AS_ARGS +=  \
common/adc_internal.d \
common/gpio.d \
common/leds.d \
common/millis_timer.d \
common/pwm.d \
common/spi.d \
common/uart.d \
main.d \
mod_dac_max5500.d \
process_internal_adc.d \
process_command.d \
mod_ems22a.d \
process_encoder.d \
process_steer_controller.d \
serial.d

OUTPUT_FILE_PATH +=steer_controller_firmware.elf

OUTPUT_FILE_PATH_AS_ARGS +=steer_controller_firmware.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler































common/%.o: ../common/%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.106\include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega164pa -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.106\gcc\dev\atmega164pa" -c -std=c++11 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.106\include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega164pa -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.106\gcc\dev\atmega164pa" -c -std=c++11 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="steer_controller_firmware.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega164pa -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.106\gcc\dev\atmega164pa"  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "steer_controller_firmware.elf" "steer_controller_firmware.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "steer_controller_firmware.elf" "steer_controller_firmware.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "steer_controller_firmware.elf" > "steer_controller_firmware.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "steer_controller_firmware.elf" "steer_controller_firmware.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "steer_controller_firmware.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "steer_controller_firmware.elf" "steer_controller_firmware.a" "steer_controller_firmware.hex" "steer_controller_firmware.lss" "steer_controller_firmware.eep" "steer_controller_firmware.map" "steer_controller_firmware.srec" "steer_controller_firmware.usersignatures"
	