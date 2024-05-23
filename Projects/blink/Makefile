BUILDTYPE ?= sysram
SDK_PATH	:= ../../
CURRENT_DIR := $(shell basename $(CURDIR))
CROSS 		:= $(SDK_PATH)toolchain/bin/arm-none-eabi-
CC   		:=  $(CROSS)gcc
CXX   		:=  $(CROSS)g++
DEBUG   	:=  

# SOURCES   	+=  $(wildcard ./*.c)
CSRCPATH	:= Src $(SDK_PATH)Drivers/BSP/Components/mcp23x17 $(SDK_PATH)Drivers/BSP/STM32MP13xx_DISCO
CXXSRCPATH	?= 
CSOURCES    += $(wildcard $(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Src/stm32mp13xx_hal*.c)
CSOURCES   	+= $(foreach dir, $(CSRCPATH), $(wildcard $(dir)/*.c))
CXXSOURCES	+= $(foreach dir, $(CXXSRCPATH), $(wildcard $(dir)/*.cpp))
CSOURCES   	+= $(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/gcc/startup_stm32mp135c_ca7.c						\
			$(SDK_PATH)Drivers/CMSIS/Core_A/Source/irq_ctrl_gic.c																\
			$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/mmu_stm32mp13xx.c									\
			$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/system_stm32mp13xx_A7.c
			   
CSOURCES_REMOVE += $(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Src/stm32mp13xx_hal_msp_template.c \
				   $(SDK_PATH)Drivers/BSP/STM32MP13xx_DISCO/stm32mp13xx_disco_sd.c   \
				   $(SDK_PATH)Drivers/BSP/STM32MP13xx_DISCO/stm32mp13xx_disco_lcd.c   \
				   $(SDK_PATH)Drivers/BSP/STM32MP13xx_DISCO/stm32mp13xx_disco_camera.c \
				   $(SDK_PATH)Drivers/BSP/STM32MP13xx_DISCO/stm32mp13xx_disco_ts.c

CSOURCES := $(filter-out $(CSOURCES_REMOVE), $(CSOURCES))

OBJ   		+=  $(patsubst %.c, %.cc.o, $(CSOURCES))
OBJ   		+=  $(patsubst %.cpp, %.cxx.o, $(CXXSOURCES))

INCLUDES   	+=   -ISrc -IInc  -I$(SDK_PATH)Drivers/CMSIS/Core_A/Include -I$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Include 
INCLUDES   	+=   -I$(SDK_PATH)Drivers/STM32MP13xx_HAL_Driver/Inc -I$(SDK_PATH)Drivers/BSP/STM32MP13xx_DISCO -I$(SDK_PATH)Drivers/BSP/Components/Common

LIB_NAMES  	+=
LIB_PATH  	+=  

CFLAGS  	+=  -mcpu=cortex-a7 -std=gnu11 -g3 
CFLAGS  	+=  -DSTM32MP135Fxx -DMCP_IOEXPANDER -DUSE_STM32MP13XX_DK -DCORE_CA7 -DCACHE_USE -DMMU_USE -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT
CFLAGS  	+=  -O0 -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -fcyclomatic-complexity -MMD -MP
CFLAGS  	+=  --specs=nano.specs -mfpu=vfpv4-d16 -mfloat-abi=hard -mthumb

CXXFLAGS  	+=  -Wall

LDFLAGS	    +=  -mcpu=cortex-a7
LDFLAGS	    +=  --specs=nosys.specs -Wl,-Map="$(CURRENT_DIR).map" 
LDFLAGS	    +=  -Wl,--gc-sections -static --specs=nano.specs -mfpu=vfpv4-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group


ifeq ($(BUILDTYPE), sysram)
LDFLAGS	    +=  -T"$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/gcc/linker/stm32mp13xx_a7_sysram.ld" 
all:sysram_bin
	@echo "BUILDTYPE is sysram"
else
LDFLAGS	    +=  -T"$(SDK_PATH)Drivers/CMSIS/Device/ST/STM32MP13xx/Source/Templates/gcc/linker/stm32mp13xx_a7_ddr.ld" 
CFLAGS  	+=  -DUSE_DDR
all:ddr_bin
	@echo "BUILDTYPE is ddr"
endif



sysram_bin: $(CURRENT_DIR).stm32
	dd if=/dev/zero of=mini_sdraw.bin bs=1M count=1
	dd if=$(CURRENT_DIR).stm32 of=mini_sdraw.bin bs=1K seek=64


$(CURRENT_DIR).stm32:$(CURRENT_DIR).elf
	$(CROSS)size  $(CURRENT_DIR).elf 
	$(SDK_PATH)Utilities/ImageHeader/postbuild_STM32MP13.sh $(SDK_PATH)toolchain/bin $(CURRENT_DIR)


ddr_bin:$(CURRENT_DIR).elf
	mv $(CURRENT_DIR).elf tmp_$(CURRENT_DIR).elf
	make clean
	cd ../FSBLA_Sdmmc ;make -j ;cp mini_sdraw.bin ../blink ;make clean
	$(CROSS)size  tmp_$(CURRENT_DIR).elf
	$(CROSS)objcopy -O binary tmp_$(CURRENT_DIR).elf $(CURRENT_DIR).bin
	dd if=$(CURRENT_DIR).bin of=mini_sdraw.bin bs=512 seek=368 conv=notrunc

#links
$(CURRENT_DIR).elf:$(OBJ)
	$(CC) $(OBJ) $(LIB_PATH) -o $(CURRENT_DIR).elf $(LDFLAGS)
 
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
	@rm -rf $(OBJ) $(TARGET) output 
	@rm -rf $(CLEANOBJ)
	@rm -rf $(CURRENT_DIR)*
	@rm -rf mini_sdraw.bin

