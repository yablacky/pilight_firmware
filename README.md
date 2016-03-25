pilight firmware
=============

this is an rf receiver prefilter for pilight build with an ATTiny45 / 85. it prefilters the received signal to reduce the load for pilight (http://www.pilight.org).
you be warned: this could potentionally damage your raspberry pi and also other hardware. i take no responsability for any damages! use at your own risk!

1. Install avrgcc:
------------------
	sudo apt-get install gcc-avr avr-libc

2. Install modified avrdude:
-------------------
	wget http://project-downloads.drogon.net/files/avrdude_5.10-4_armhf.deb
	wget http://project-downloads.drogon.net/files/avrdude-doc_5.10-4_all.deb
	sudo dpkg -i avrdude_5.10-4_armhf.deb
	sudo dpkg -i avrdude-doc_5.10-4_all.deb

3. Calculate the MIN_PULSELENGTH and MAX_PULSELENGTH values
-------------------
	MIN_PULSELENGTH =  ( shortest expected pulse - 10% ) / 10
	MAX_PULSELENGTH =  ( longest expected pulse + 10% ) / 10
	
	the shortest expected pulse is the minimal base pulse-width of all your protocols
	the longest expected pulse is the maximal base pulse-width of all your protocols multiplied by 34
	
4. Compile and program ATTiny:
------------------------------
	make all
	
	make asm		# generate pilight_firmware.s file (good to review compiler generated assembler code)
	make flash		# compile and flash the ATTiny using pilight-flash on your system.

	type=t85 make ....	# generate firmware for ATTiny 85 (ATTiny25 no longer supported since VERSION 4).


Additional info:
----------------
###Calculate fuses:
	http://www.engbedded.com/fusecalc/

###Pinout:
	
I use this circuit without resistors. if you want you can even power the ATTiny with 3.3V, it works for 4 out 5 ATTiny's, but i think the timing is less accurate.
	![schematic](circuit.png "schematic")

|  Name  | Raspberry Pi V2 | ATTiny45 | 433 Receiver| Notes
|--------|-----------------|----------|-------------|--------------------
|  MOSI  |       19        |    5     |      -      | A HI+LO signal on this ATTiny pin selects a different filter method (since version 4).
|  MISO  |       21        |    6     |      -      |
|  SCK   |       23        |    7     |      -      |
| RESET  |       24        |    1     |      -      |
| PI_IN  | see pilight cfg |    2     |      -      |
|REC_OUT |       -         |    3     |   DATA_OUT  |


to change the pins create an .avrduderc file in your home directory containing (the numbering is wiringpi numbering!):


	programmer
		id    = "gpio";
		desc  = "Use sysfs interface to bitbang GPIO lines";
		type  = gpio;
		reset = 8;
		sck   = 11;
		mosi  = 10;
		miso  = 9;
	;

###FilterMethods

Since version 4, the firmware implements four different filter methods that can be selected
in normal operation and without flashing by signalling a HI+LO on ATTiny pin 5. After going
LO the ATTiny sends a signature in about 500 microseconds.

The version 4 signature contains the selected filter method in the version
number like this: if
	version_in_signature % 100 == 4 then
	version_in_signature / 100 == Index of selected filter method, where:

	0:	Builtin default filter method; not from EEPROM; this is version 4 filter.
	1:	A new filter that is silent and only generates firmware signatures.
	2:	A new filter that generates test signal pulses (and signature).
	3:	The previous version 3 filter (check and dismiss 7 pulses before passing).
	4:	A new version 4 filter (searches longer pulse trains w/o loosing pulses). Default now.

###ATTiny85

If you want to program an ATTiny85 add a parameter when calling make:

```
Make all type=t85
```
