#!/bin/bash
# install the bootloader
#
# Have you set the fuses yet? see fuses.sh
#

#PROG=dragon_dw
PROG=dragon_isp

PART=ATTINY861

avrdude -c $PROG -P usb -p $PART -U flash:w:$1
