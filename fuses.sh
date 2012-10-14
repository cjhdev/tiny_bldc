#!/bin/bash
# set fuses for the ATTINY861
#
# 0 means programmed, 1 means unprogrammed
#
# Regular Configuration:
#
#EXTEND=0xfe
EXTEND=0x00 #I read somewhere that AVR dude reads unused fuses as 0 is this true?
# 0x80: 1 (null)
# 0x40: 1 (null)
# 0x20: 1 (null)
# 0x10: 1 (null)
# 0x08: 1 (null)
# 0x04: 1 (null)
# 0x02: 1 (null)
# 0x01: 0 (self programming enable)
#
HIGH=0xd5
# 0x80: 1 (external reset disable)
# 0x40: 1 (debug wire enabled)
# 0x20: 0 (enable serial programming_
# 0x10: 1 (WDT always on)
# 0x08: 0 (eeprom is saved through chip erase)
# 0x04: 1 (BOD2) (101 = 2.7V typical brownout, 4.5 might be a bit close)
# 0x02: 0 (BOD1)
# 0x01: 1 (BOD0)
#
LOW=0xc1
# 0x80: 1 (system clock 1/8 prescaler)
# 0x40: 1 (clock out enable)
# 0x20: 0 (SUT1) (00 if BOD enabled)
# 0x10: 0 (SUT0)
# 0x08: 0 (CKSEL3) (0001 = PLL/4 == ~16MHz)
# 0x04: 0 (CKSEL2)
# 0x02: 0 (CKSEL1)
# 0x01: 1 (CKSEL0)
#
# Debug Wire Configuration: (Only if debugger is required)
#
#HIGH=0x85
# 0x80: 1 (external reset disable)
# 0x40: 0 (debug wire enabled)
# 0x20: 0 (enable serial programming_
# 0x10: 0 (WDT always on)
# 0x08: 0 (eeprom is saved through chip erase)
# 0x04: 1 (BOD2) (101 = 2.7V typical brownout)
# 0x02: 0 (BOD1)
# 0x01: 1 (BOD0)
#

#PROG=dragon_dw
PROG=dragon_isp

PART=ATTINY861

avrdude -c $PROG -P usb -p $PART -U lfuse:w:$LOW:m -U hfuse:w:$HIGH:m -U efuse:w:$EXTEND:m
