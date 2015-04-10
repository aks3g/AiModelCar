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
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../adc_driver.cpp \
../core_driver.cpp \
../dac_driver.cpp \
../gpio_driver.cpp \
../i2c_driver.cpp \
../pwm_driver.cpp \
../spi_driver.cpp \
../timer_driver.cpp \
../uart_driver.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
adc_driver.o \
core_driver.o \
dac_driver.o \
gpio_driver.o \
i2c_driver.o \
pwm_driver.o \
spi_driver.o \
timer_driver.o \
uart_driver.o

OBJS_AS_ARGS +=  \
adc_driver.o \
core_driver.o \
dac_driver.o \
gpio_driver.o \
i2c_driver.o \
pwm_driver.o \
spi_driver.o \
timer_driver.o \
uart_driver.o

C_DEPS +=  \
adc_driver.d \
core_driver.d \
dac_driver.d \
gpio_driver.d \
i2c_driver.d \
pwm_driver.d \
spi_driver.d \
timer_driver.d \
uart_driver.d

C_DEPS_AS_ARGS +=  \
adc_driver.d \
core_driver.d \
dac_driver.d \
gpio_driver.d \
i2c_driver.d \
pwm_driver.d \
spi_driver.d \
timer_driver.d \
uart_driver.d

OUTPUT_FILE_PATH +=libxmegaE5.a

OUTPUT_FILE_PATH_AS_ARGS +=libxmegaE5.a

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler



















./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DNDEBUG  -I"../../include/xmegaE5" -I"../../include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -mmcu=atxmega32e5 -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
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


$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Archiver : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-ar.exe$(QUOTE) -r  -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) 
	@echo Finished building target: $@
	
	




# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "libxmegaE5.elf" "libxmegaE5.a" "libxmegaE5.hex" "libxmegaE5.lss" "libxmegaE5.eep" "libxmegaE5.map" "libxmegaE5.srec" "libxmegaE5.usersignatures"
	