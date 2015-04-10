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
../system \
../userland


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../system/AiMini4wdBase.cpp \
../system/ff.cpp \
../system/ioconfig.cpp \
../system/LSM330DLC.cpp \
../system/sdmm.cpp \
../userland/userland.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
system/AiMini4wdBase.o \
system/ff.o \
system/ioconfig.o \
system/LSM330DLC.o \
system/sdmm.o \
userland/userland.o

OBJS_AS_ARGS +=  \
system/AiMini4wdBase.o \
system/ff.o \
system/ioconfig.o \
system/LSM330DLC.o \
system/sdmm.o \
userland/userland.o

C_DEPS +=  \
system/AiMini4wdBase.d \
system/ff.d \
system/ioconfig.d \
system/LSM330DLC.d \
system/sdmm.d \
userland/userland.d

C_DEPS_AS_ARGS +=  \
system/AiMini4wdBase.d \
system/ff.d \
system/ioconfig.d \
system/LSM330DLC.d \
system/sdmm.d \
userland/userland.d

OUTPUT_FILE_PATH +=AiModelCar.elf

OUTPUT_FILE_PATH_AS_ARGS +=AiModelCar.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler













system/%.o: ../system/%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DNDEBUG  -I"../../../AtmelLibrarys/include/xmegaE5" -I"../userland" -I"../system" -I"../../../AtmelLibrarys/include"  -Os -ffunction-sections -fdata-sections -fpack-struct -Wall -mmcu=atxmega32e5 -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

userland/%.o: ../userland/%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DNDEBUG  -I"../../../AtmelLibrarys/include/xmegaE5" -I"../userland" -I"../system" -I"../../../AtmelLibrarys/include"  -Os -ffunction-sections -fdata-sections -fpack-struct -Wall -mmcu=atxmega32e5 -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
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
	@echo Invoking: AVR8/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="AiModelCar.map" -Wl,--start-group -Wl,-lm -Wl,-lxmegaE5 -Wl,-lxmegaE5  -Wl,--end-group -Wl,-L"../../lib"  -Wl,--gc-sections -mmcu=atxmega32e5  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "AiModelCar.elf" "AiModelCar.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "AiModelCar.elf" "AiModelCar.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "AiModelCar.elf" > "AiModelCar.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "AiModelCar.elf" "AiModelCar.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-size.exe" "AiModelCar.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "AiModelCar.elf" "AiModelCar.a" "AiModelCar.hex" "AiModelCar.lss" "AiModelCar.eep" "AiModelCar.map" "AiModelCar.srec" "AiModelCar.usersignatures"
	