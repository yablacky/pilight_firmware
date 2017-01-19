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
    MCU=unknown-type-$(type)
endif

CC=/usr/bin/avr-gcc
CFLAGS= -mmcu=$(MCU) -DF_CPU=16000000UL -DTARGET_MCU_$(MCU) -Wall -Os -fno-inline-small-functions -mrelax
OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=/usr/bin/avrdude
PILIGHT_FLASH=pilight-flash 

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
	@echo "\tasm    : Compile and generate .s file to inspect generated code."
	@echo "\tclean  : Remove files that can be regenerated."

attiny85:
attiny45:
attiny25:

program : $(MCU) $(TARGET)_$(AVRDUDEMCU).hex
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P gpio -c gpio -b 10000 -U flash:w:$(TARGET)_$(AVRDUDEMCU).hex

compile : $(MCU)
	$(CC) $(CFLAGS) -c $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET)_$(AVRDUDEMCU).elf $(TARGET).o
	$(OBJ2HEX) -j .text -j .data -O ihex $(TARGET)_$(AVRDUDEMCU).elf $(TARGET)_$(AVRDUDEMCU).hex

asm : $(MCU)
	$(CC) $(CFLAGS) -S $(TARGET).c

clean :
	rm -f *.hex *.obj *.o *.elf

flash : $(MCU) $(TARGET)_$(AVRDUDEMCU).hex
	sudo $(PILIGHT_FLASH) --file=$(TARGET)_$(AVRDUDEMCU).hex

$(TARGET)_$(AVRDUDEMCU).hex : $(MCU) $(TARGET).c
	$(CC) $(CFLAGS) -c $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET)_$(AVRDUDEMCU).elf $(TARGET).o
	$(OBJ2HEX) -j .text -j .data -O ihex $(TARGET)_$(AVRDUDEMCU).elf $(TARGET)_$(AVRDUDEMCU).hex

all: compile erase program
erase:
	$(AVRDUDE) -p $(AVRDUDEMCU) -P gpio -c gpio -e -U lfuse:w:0xe1:m -U hfuse:w:0xdf:m

pulsi: pulsi.c
	gcc pulsi.c -o pulsi
	
# ATtiny45 (internal 1 MHz clock) 	0x62 	0xdf 
# ATtiny45 (internal 8 MHz clock) 	0xe2	0xdf 
# ATtiny45 (external 20 MHz clock) 	0xfe	0xdf 
# ATtiny85 (internal 1 MHz clock) 	0x62 	0xdf 
# ATtiny85 (internal 8 MHz clock) 	0xe2 	0xdf 
# ATtiny85 (external 20 MHz clock) 	0xfe 	0xdf
