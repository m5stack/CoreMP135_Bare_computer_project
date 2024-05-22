SDK_PATH	:= ../../
CURRENT_DIR := $(shell basename $(CURDIR))
CROSS 		:= $(SDK_PATH)toolchain/bin/arm-none-eabi-
CC   		:=  $(CROSS)gcc
CXX   		:=  $(CROSS)g++
DEBUG   	:=  
# SOURCES   	+=  $(wildcard ./*.c)
CSRCPATH	:= Src $(SDK_PATH)Drivers/BSP/Components/mcp23x17 
CXXSRCPATH	?= 
CSOURCES    += $(wildcard $(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Src/stm32mp13xx_hal*.c)
CSOURCES   	+= $(foreach dir, $(CSRCPATH), $(wildcard $(dir)/*.c))
CXXSOURCES	+= $(foreach dir, $(CXXSRCPATH), $(wildcard $(dir)/*.cpp))
CSOURCES   	+= $(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/gcc/startup_stm32mp135c_ca7.c						\
			$(SDK_PATH)Drivers/CMSIS/Core_A/Source/irq_ctrl_gic.c																\
			$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/mmu_stm32mp13xx.c									\
			$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/system_stm32mp13xx_A7.c								\
			$(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Src/stm32mp13xx_ll_sdmmc.c												
			   
CSOURCES_REMOVE += $(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Src/stm32mp13xx_hal_msp_template.c \
				   $(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Src/stm32mp13xx_hal_timebase_tim_template.c

CSOURCES := $(filter-out $(CSOURCES_REMOVE), $(CSOURCES))

INCLUDES   	+=   -ISrc -IInc  -I$(SDK_PATH)Drivers/CMSIS/Core_A/Include -I$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Include 
INCLUDES   	+=   -I$(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Inc -I$(SDK_PATH)Drivers/BSP/STM32MP13xx_DISCO -I$(SDK_PATH)Drivers/BSP/Components/Common



LIB_NAMES  	+=
LIB_PATH  	+=  -L./lib
OBJ   		+=  $(patsubst %.c, %.cc.o, $(CSOURCES))
OBJ   		+=  $(patsubst %.cpp, %.cxx.o, $(CXXSOURCES))
CFLAGS  	+=  -mcpu=cortex-a7 -std=gnu11 -g3 
# CFLAGS  	+=  -DGPT_TABLE_PRESENT
CFLAGS  	+=  -DSTM32MP135Fxx -DUSE_STM32MP13XX_DK -DCORE_CA7 -DNO_CACHE_USE -DNO_MMU_USE -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT -DDDR_TYPE_DDR3_4Gb -DUSE_FULL_ASSERT
CFLAGS  	+=  -Og -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -fcyclomatic-complexity -MMD -MP
# CFLAGS  	+=  -MF"/home/nihao/work/github/STM32CubeMP13/Projects/STM32MP135C-DK/Examples/UART/UART_Receive_Transmit_Console/STM32CubeIDE/Debug/Drivers/STM32MP13xx_HAL_Driver/stm32mp13xx_hal.d" -MT"Drivers/STM32MP13xx_HAL_Driver/stm32mp13xx_hal.o" 
CFLAGS  	+=  --specs=nano.specs -mfpu=vfpv4-d16 -mfloat-abi=hard -mthumb


CXXFLAGS  	+=  -Wall
LDFLAGS	    +=  -mcpu=cortex-a7 
LDFLAGS	    +=  -T"$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/gcc/linker/stm32mp13xx_a7_sysram.ld" 
LDFLAGS	    +=  --specs=nosys.specs
LDFLAGS	    +=  -Wl,-Map="$(CURRENT_DIR).map" 
LDFLAGS	    +=  -Wl,--gc-sections -static --specs=nano.specs -mfpu=vfpv4-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group


# $(CURRENT_DIR)_A7_Signed.bin: $(CURRENT_DIR).elf
# 	$(CROSS)size  $(CURRENT_DIR).elf 
# 	$(CROSS)objcopy -O binary $(CURRENT_DIR).elf $(CURRENT_DIR)_A7.bin
# 	$(SDK_PATH)Utilities/ImageHeader/postbuild.sh $(shell pwd) $(CURRENT_DIR)_A7.bin $(CURRENT_DIR)_A7_Signed.bin "-bt 10" "-ep 2FFE0000"


# mini_sdraw.bin: $(CURRENT_DIR).stm32
# 	dd if=/dev/zero of=mini_sdraw.bin bs=1M count=1
# 	dd if=$(CURRENT_DIR).stm32 of=mini_sdraw.bin bs=1K seek=64


$(CURRENT_DIR).stm32:$(CURRENT_DIR).elf
	$(CROSS)size  $(CURRENT_DIR).elf 
	$(SDK_PATH)Utilities/ImageHeader/postbuild_STM32MP13.sh $(SDK_PATH)toolchain/bin $(CURRENT_DIR)


#links
$(CURRENT_DIR).elf:$(OBJ)
	@mkdir -p output
	$(CC) $(OBJ) $(LIB_PATH) $(LIB_NAMES) -o $(CURRENT_DIR).elf $(LDFLAGS)
 
#compile
%.cc.o: %.c
	$(CC) $(INCLUDES) $(DEBUG) -c $(CFLAGS) $< -o $@
%.cxx.o: %.cpp
	$(CXX) $(INCLUDES) $(DEBUG) -c $(CXXFLAGS) $< -o $@

run:
	@ echo run output/$(TARGET)$(VERSION)
	@ ./output/$(TARGET)$(VERSION)


CLEANOBJ   		+=  $(patsubst %.c, %.cc.cyclo, $(CSOURCES))
CLEANOBJ   		+=  $(patsubst %.c, %.cc.d, $(CSOURCES))
CLEANOBJ   		+=  $(patsubst %.c, %.cc.su, $(CSOURCES))
.PHONY:clean
clean:
	@echo "Remove linked and compiled files......"
	rm -rf $(OBJ) $(TARGET) output 
	rm -rf $(CLEANOBJ)
	rm -rf $(CURRENT_DIR)*
	rm -rf mini_sdraw.bin

