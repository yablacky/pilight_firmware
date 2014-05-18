TARGET=pilight_firmware
VERSION=v3

MCU=attiny45
AVRDUDEMCU=t45
CC=/usr/bin/avr-gcc
CFLAGS= -mmcu=$(MCU) -DF_CPU=16000000UL -Wall -Os -fno-inline-small-functions -mrelax
OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=/usr/bin/avrdude

program : $(TARGET)_$(AVRDUDEMCU)_$(VERSION).hex
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P gpio -c gpio -b 10000 -U flash:w:$(TARGET)_$(AVRDUDEMCU)_$(VERSION).hex

compile :
	$(CC) $(CFLAGS) -c $(TARGET)_$(AVRDUDEMCU)_$(VERSION).c
	$(CC) $(CFLAGS) -o $(TARGET)_$(AVRDUDEMCU)_$(VERSION).elf $(TARGET)_$(AVRDUDEMCU)_$(VERSION).o
	$(OBJ2HEX) -j .text -j .data -O ihex $(TARGET)_$(AVRDUDEMCU)_$(VERSION).elf $(TARGET)_$(AVRDUDEMCU)_$(VERSION).hex

clean :
	rm -f *.hex *.obj *.o *.elf

all: compile erase program
#clean
erase:
	$(AVRDUDE) -p $(AVRDUDEMCU) -P gpio -c gpio -e -U lfuse:w:0xe1:m -U hfuse:w:0xdf:m
	
# ATtiny45 (internal 1 MHz clock) 	0x62 	0xdf 
# ATtiny45 (internal 8 MHz clock) 	0xe2	0xdf 
# ATtiny45 (external 20 MHz clock) 	0xfe	0xdf 
# ATtiny85 (internal 1 MHz clock) 	0x62 	0xdf 
# ATtiny85 (internal 8 MHz clock) 	0xe2 	0xdf 
# ATtiny85 (external 20 MHz clock) 	0xfe 	0xdf
