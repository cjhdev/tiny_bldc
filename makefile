CC = avr-gcc
AS = avr-gcc
MCU = attiny861

OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SREC = srec_cat
CHECKSUM = ./tools/checksum

# build code should be defined externally i.e. make {rule} BUILD={32bit buildcode}
ifndef BUILD
BUILD = 0x00000000
endif

# extract constants from the source
FIRM_VERSION = $(shell grep -Po '(?<=VERSION ")(.+)(?=")' bldc.c)
BOOT_VERSION = $(shell grep -Po '(?<=VERSION ")(.+)(?=")' boot.c)

CFLAGS = -O2 -Wall -mmcu=$(MCU) -funsigned-char -funsigned-bitfields  \
	-fpack-struct -fshort-enums -std=gnu99 -g

LDFLAGS = -Wl,-Map=$@.map,--cref

# build bootloader
boot: LDFLAGS += -nostartfiles -nodefaultlibs -Wl,-Tboot.lnk
boot: CFLAGS += -DBUILD=$(BUILD)
boot: OUTFILE = $@_$(BUILD)_v$(BOOT_VERSION)
boot: boot.o twi.o util.o db.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@.elf
	$(OBJDUMP) -h -S $< > $@.lss
	$(OBJCOPY) -O binary -j .text  $@.elf $(OUTFILE).bin
	$(CHECKSUM) $(OUTFILE).bin
	$(SREC) $(OUTFILE).bin -binary -output $(OUTFILE).hex -intel --address_length=2 --line_length=44

# build firmware
firm: LDFLAGS += -Wl,-Tfirm.lnk
firm: CFLAGS += -DBUILD=$(BUILD)
firm: OUTFILE = $@_$(BUILD)_v$(FIRM_VERSION)
firm: bldc.o twi.o util.o db.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@.elf
	$(OBJDUMP) -h -S $< > $@.lss
	$(OBJCOPY) -O binary -j .text  $@.elf $(OUTFILE).bin
	$(CHECKSUM) $(OUTFILE).bin
	$(SREC) $(OUTFILE).bin -binary -output $(OUTFILE).hex -intel --address_length=2 --line_length=44

clean:
	$(RM) -rf *.o *.elf *.map *.lss
