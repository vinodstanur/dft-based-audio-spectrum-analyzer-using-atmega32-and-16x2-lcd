# AVR-GCC Makefile
PROJECT=project
SOURCES += main.c
#SOURCES=lcd.c
#SOURCES=tv.c
CC=avr-gcc
OBJCOPY=avr-objcopy
MMCU=atmega32

CFLAGS=-mmcu=$(MMCU)

$(PROJECT).hex: $(PROJECT).out
	$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES)
	$(CC) $(CFLAGS) -O3 -I./ -o $(PROJECT).out $(SOURCES)

p: $(PROJECT).hex
	sudo avrdude -p m32 -c usbasp -U flash:w:$(PROJECT).hex:i

burn_fuse: $(PROJECT).hex
	avrdude -p m32 -c usbasp -U lfuse:w:0xef:m
	
clean:
	rm -f $(PROJECT).out
	rm -f $(PROJECT).hex
	rm -f $(PROJECT).s
