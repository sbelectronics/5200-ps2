# Makefile for programming the ATtiny85
# modified the one generated by CrossPack

DEVICE      = atmega328
CLOCK      = 8000000
PROGRAMMER = -c usbtiny 
# for atmega328
# see http://www.engbedded.com/fusecalc/
FUSES       = -U lfuse:w:0xE2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m 

# Tune the lines below only if you know what you are doing:
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	5200ps2-deluxe.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -B 3 -c usbasp -U flash:w:5200ps2-deluxe.hex:i

fuse:
	$(AVRDUDE) -B 3 -c usbasp $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID 5200ps2-deluxe.hex

clean:
	rm -f 5200ps2-deluxe.hex 5200ps2-deluxe.elf

# file targets:
5200ps2-deluxe.elf: 5200ps2-deluxe.c
	$(COMPILE) -o 5200ps2-deluxe.elf 5200ps2-deluxe.c ../lib/ps2.c ../lib/digipot.c

5200ps2-deluxe.hex: 5200ps2-deluxe.elf
	rm -f 5200ps2-deluxe.hex
	avr-objcopy -j .text -j .data -O ihex 5200ps2-deluxe.elf 5200ps2-deluxe.hex
	avr-size 5200ps2-deluxe.elf

# Targets for code debugging and analysis:
disasm:	5200ps2-deluxe.elf
	avr-objdump -d 5200ps2-deluxe.elf

cpp:
	$(COMPILE) -E 5200ps2-deluxe.c ../lib/ps2.c
