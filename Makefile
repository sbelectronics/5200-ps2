# Makefile for programming the ATtiny85
# modified the one generated by CrossPack

DEVICE      = attiny861
CLOCK      = 8000000
PROGRAMMER = -c usbtiny 
# for ATTiny861
# see http://www.engbedded.com/fusecalc/
FUSES       = -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 

# Tune the lines below only if you know what you are doing:
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	5200ps2.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:5200ps2.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID 5200ps2.hex

clean:
	rm -f 5200ps2.hex 5200ps2.elf

# file targets:
5200ps2.elf: 5200ps2.c
	$(COMPILE) -o 5200ps2.elf 5200ps2.c

5200ps2.hex: 5200ps2.elf
	rm -f 5200ps2.hex
	avr-objcopy -j .text -j .data -O ihex 5200ps2.elf 5200ps2.hex
#	avr-size --format=avr --mcu=$(DEVICE) 5200ps2.elf
	avr-size 5200ps2.elf

# Targets for code debugging and analysis:
disasm:	5200ps2.elf
	avr-objdump -d 5200ps2.elf

cpp:
	$(COMPILE) -E 5200ps2.c
