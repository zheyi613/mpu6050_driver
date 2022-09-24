# brief:  build STM32F767ZI
# author: zheyi613

# build target
TARGET = main
# debug build?
DEBUG = 1
# build path
BUILD_DIR = build

# C sources
C_SOURCES =  $(wildcard ./Src/*.c)
# ASM sources
ASM_SOURCES = startup_stm32f767xx.s

# Arm GNU prefix
PREFIX = arm-none-eabi-
# gcc for c
CC = $(PREFIX)gcc
# gcc for assembly
AS = $(PREFIX)gcc -x assembler-with-cpp
# list the section sizes (displayed closer System V conventions)
SZ = $(PREFIX)size -A
# not used
# CP = $(PREFIX)objcopy
# HEX = $(CP) -O ihex
# BIN = $(CP) -O binary -S

# ARM processor
CPU = -mcpu=cortex-m7
# fpu (floating-point hardware)
FPU = -mfpu=fpv5-d16
# use hardware floating-point instructions and hareware floating-point linkage
FLOAT-ABI = -mfloat-abi=hard
# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# AS defines
AS_DEFS = 
# C defines macro name
C_DEFS = \
#-DSTM32F767xx 
#-DUSE_HAL_DRIVER

# AS includes
AS_INCLUDES = 
# C includes
C_INCLUDES =  -I./Inc

# minimum optimization (default) 
OPT = -O0

# gcc flags
# -fdata-sections is used, the variables are put into different sections.
# -ffunction-sections reduces the potential for sharing addresses, data, and string literals between functions.
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
# Adds debug tables for source-level debugging.
CFLAGS += -gdwarf-2
# Generate dependency information
# -MMD create a makefile dependency file (source.d) for each source file and allow for compliation or assembly to continue.
# -MP emits dummy dependency rules.
# -MF specifies a filename for the makefile dependency rules.
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# LDFLAGS
# link script
LDSCRIPT = STM32F767ZITx_FLASH.ld

# libraries
# use C standard, math, nosys (pass compiler without syscalls) librarys
# use float with printf and scanf from newlib-nano
LIBS = -lc -lm -lnosys -u _printf_float -u _scanf_float
LIBDIR = 
# -specs-nano.specs replace -lc (newlib) with newlib-nano
# --cref tells the linker to add cross-reference information to the map file.
# -Wl,--gc-sections perform a garbage collection of code and data never referenced. (only link useful function)
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

all: $(BUILD_DIR)/$(TARGET).elf

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
	
$(BUILD_DIR):
	mkdir $@		

.PHONY: clean
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***