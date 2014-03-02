TARGET=pilight_firmware_v2

MCU=attiny45
AVRDUDEMCU=t45
CC=/usr/bin/avr-gcc
CFLAGS= -Wall -mmcu=$(MCU) -DF_CPU=16000000UL
OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=/usr/bin/avrdude

program : $(TARGET).hex
	sudo $(AVRDUDE) -p $(AVRDUDEMCU) -P gpio -c gpio -b 10000 -U flash:w:$(TARGET).hex

compile :
	$(CC) $(CFLAGS) -c $(TARGET).c
	$(CC) $(CFLAGS) -o $(TARGET).elf $(TARGET).o
	$(OBJ2HEX) -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex

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