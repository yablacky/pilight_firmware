TARGET=pilight_firmware
VERSION=v4

ifeq ($(type),t85)
    MCU=attiny85
    AVRDUDEMCU=t85
else ifeq ($(type),t25)
    MCU=attiny25
    AVRDUDEMCU=t25
else ifeq ($(subst t45,,$(type)),)
    MCU=attiny45
    AVRDUDEMCU=t45
else
    $(error Unknown type "$(type)". Please 'make help' for more information.)
endif

TIMESTAMP:=$(shell date +"%Y-%m-%dT%H-%M-%S")

CC=/usr/bin/avr-gcc
CFLAGS= -mmcu=$(MCU) -DF_CPU=16000000UL -DTARGET_MCU_$(MCU) -Wall -Os -fno-inline-small-functions -mrelax $(EXTRA_CFLAGS)
OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=/usr/bin/avrdude
PILIGHT_FLASH=pilight-flash 

.PHONY: help compile asm program flash clean erase all compile-all

.DELETE_ON_ERROR: ;

help :
	@echo "Generate pilight firmware files and optionally flash them immediately.\n"
	@echo "Usage: make [type=<txx>] <target>\n\nThe type option selects the target device type and txx is one of"
	@echo "\tt25, t45, or t85 for ATTiny 25, 45 or 85. t45 is default."
	@echo "Consider to use shell command 'export type=txx' to set a different default."
	@echo "\nMake targets are:"
	@echo "\tall    : Compile Erase and Flash $(MCU) using $(AVRDUDE)"
	@echo "\tprogram: Compile and Flash $(MCU) using $(AVRDUDE)"
	@echo "\terase  : Set $(MCU)(??) fuses using $(AVRDUDE)"
	@echo "\tflash  : Compile and Flash $(MCU) using $(PILIGHT_FLASH)"
	@echo "\tcompile: Compile and generate $(MCU) .hex file that can be flashed."
	@echo "\tcompile-all: Compile and generate .hex file for all ATTiny 25,45 and 85."
	@echo "\tasm    : Compile and generate .s file to inspect generated code."
	@echo "\tpulsi  : Create the stand-alone pulse generator pulsi."
	@echo "\tclean  : Remove files that can be regenerated."

program : $(TARGET)_$(AVRDUDEMCU).hex
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P gpio -c gpio -b 10000 -U flash:w:$(TARGET)_$(AVRDUDEMCU).hex

compile : $(TARGET)_$(AVRDUDEMCU).hex

asm : $(TARGET).s

clean :
	rm -f *.hex *.obj *.o *.elf *.s

flash : $(TARGET)_$(AVRDUDEMCU).hex
	sudo $(PILIGHT_FLASH) --file=$(TARGET)_$(AVRDUDEMCU).hex

$(TARGET).s: $(TARGET).c
	$(CC) $(CFLAGS) -S $(TARGET).c

$(TARGET)_$(AVRDUDEMCU).hex: $(TARGET)_$(AVRDUDEMCU).elf
	$(OBJ2HEX) -j .text -j .data -O ihex $(TARGET)_$(AVRDUDEMCU).elf $(TARGET)_$(AVRDUDEMCU).hex

$(TARGET)_$(AVRDUDEMCU).elf: $(TARGET)_$(AVRDUDEMCU).o
	$(CC) $(CFLAGS) -o $(TARGET)_$(AVRDUDEMCU).elf $(TARGET)_$(AVRDUDEMCU).o

$(TARGET)_$(AVRDUDEMCU).o : $(TARGET).c
	$(CC) $(CFLAGS) -c $(TARGET).c -o $(TARGET)_$(AVRDUDEMCU).o

all: compile erase program
erase:
	$(AVRDUDE) -p $(AVRDUDEMCU) -P gpio -c gpio -e -U lfuse:w:0xe1:m -U hfuse:w:0xdf:m

compile-all: clean
	type=t25 make compile
	type=t45 make compile
	type=t85 make compile

pulsi: pulsi.c
	gcc pulsi.c -o pulsi

deploy: compile-all pulsi
	mkdir -p deployed/$(TIMESTAMP)
	cp $(TARGET)*.hex $(TARGET).c firmware-control pulsi pulsi.c [Mm]akefile deployed/$(TIMESTAMP)
	cd deployed/$(TIMESTAMP) && zip firmware-$(shell date +"%Y%m%d%H").zip $(TARGET)*.hex firmware-control pulsi

# ATtiny45 (internal 1 MHz clock) 	0x62 	0xdf 
# ATtiny45 (internal 8 MHz clock) 	0xe2	0xdf 
# ATtiny45 (external 20 MHz clock) 	0xfe	0xdf 
# ATtiny85 (internal 1 MHz clock) 	0x62 	0xdf 
# ATtiny85 (internal 8 MHz clock) 	0xe2 	0xdf 
# ATtiny85 (external 20 MHz clock) 	0xfe 	0xdf
