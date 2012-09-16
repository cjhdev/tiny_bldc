# dev0002: Low Cost BLDC ESC for Radio Controlled Multi-Rotor Applications

Firmware and bootloader for a low-cost BLDC ESC designed for radio controlled multi-rotor applications.

## Features Included
* 12bit PWM at 8, 16 and 32KHz (lower or higher if application suits, though there are practical limits and tradeoffs)
* syncronous rectification; dead time blanking without extra componenets
* fast hardware based current limiting
* temperature measurement
* voltage measurement
* configurable control parameters
* BEMF position detection
* 100Kbit two-wire bus for control, configuration and data collection
* Bootloader

## Features Excluded
* PPM; no spare IO in design and two-wire is more elegant.
* UART; target does not have a UART.
* Braking
* High speed commutation
* PID speed control

## License
Simplified BSD.

## Associated Documentation
[PR01 Two-Wire Bridging Protocol](http://cjh.id.au/doc/doc0002-0.01.pdf)
[PR02 Two-Wire Transfer Protocol](http://cjh.id.au/doc/doc0004-0.01.pdf)
More to come.

## Associated Tools
[Two-wire Bridge](http://github/cjhdev/dev0001)
[ESC Tools](http://github/cjhdev/dev0003)(Included as a sub repository in /tools)

## Status of Project
Bootloader bootloading on prototype hardware.
Application under development.

## Target MCU
Firmware is written for the ATTINY861. In a pinch you might be able to fit the application onto an ATTINY461.
This is the MCU of choice for the following reasons:
* Common; supported by avr-gcc.
* Low cost, even in low quantity.
* Internal PLL oscillator for high frequency, high resolution PWM; this can also be used to clock the CPU so no crystal is required.
* 12bit PWM generator with deadtime control.
* Hardware PWM cutoff for fast overcurrent limiting; comparator mode input with analogue hysterisis and digital glitch filtering.
* EEPROM and self programmable flash

## Control Interface
The ESC is controlled, configured and updated through a 100Kbit two-wire serial interface.

This interface is ideal for the following reasons:
* Multidrop
* Built-in addressing
* Bus is clocked by primary station
    * No baud rate negotiation
    * No crystal required on the secondary station
* No spare IO in this design for a pulse input;
* Cheap

## Communication Protocol
* PR03; Firmware upload, configuration, general.
* Flight protocol; broadcast and unicast modes for delivering throttle settings and retrieving status information.

Flight protocol is somewhat flexible through configuration and/or special build codes.
Multiple protocols exist at the same time by allocating certain addresses to certain protocols.

## Bootloader
The bootloader compiles from this repository. This allows code and structures to be easily reused between bootloader and application.

## Compiling and Loading
You will need the following:
* gcc and avr-gcc; compiling firmware, bootloader and associated tools.
* avrdude; loading bootloader onto fresh chips.
* srec_cat; converting between binary formats.
* bash

For the time being you need to be running linux. Just put ubuntu in a virtual machine and you are pretty much done. The only platform specific code is the serial driver which can be ported to windows at some stage.

### Build Tools
    cd tools
    make clean
    make all
    make clean

These are PC tools that allow you to:
* checksum; appends CRC to firmware and bootloader images.
* pack; packs release files from firmware images.
* setup; for loading firmware and configuration to the ESC.
More tools to come.

### Build the Bootloader
    make clean
    make boot
If the code compiles, the makefile will produce a number of files. The one you want to load (or send to others) is boot_{build}_{version}.hex.

### Load the Bootloader
The bootloader needs to be loaded onto the the MCU using a programmer; I use the AVR Dragon.
    sh install_bootloader.sh

### Build Firmware
    sh build.sh
This will run the build system and produce a packed release file containing all firmware builds.
The output "firm_pack_{version}.pack" contains the images, this can be input to the bootloader tool and/or distributed to others.
Alternatively you can build individual images:
    make clean
    make firm BUILD={your buildcode}
Much like bootloader you will be able to locate *.bin and *.hex files.

### Setting Fuses
If you are starting with a fresh chip simply run:
    sh fuses.sh


Cameron Harper 2012
(cam@cjh.id.au) 
