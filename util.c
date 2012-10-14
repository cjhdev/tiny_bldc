/*

Copyright (c) 2012, Cameron Harper
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <stdint.h>
#include <avr/pgmspace.h>

#include "bldc.h"
#include "util.h"

/* Generate CRC8 for a character; ccitt-crc8 (0x07) */
uint8_t crc8_char(uint8_t crc, uint8_t c)
{
    uint8_t rem = (c ^ crc);
    uint8_t i;
    for(i=0; i < 8; i++)
        rem = (rem & 0x80)?((rem << 1) ^ 0x07):rem << 1;

    return (crc ^ rem);
}

/* Generate CRC8 for a block of characters */
uint8_t crc8_block(uint8_t crc, uint8_t *b, size_t len)
{
    while(len--)
        crc = crc8_char(crc, *b++);

    return crc;
}

/* Generate CRC16 for a character; ccitt-crc16 (0x1021) */
uint16_t crc16_char(uint16_t crc, uint8_t c)
{
    uint16_t rem = (((uint16_t)c << 8) ^ crc) & 0xff00;
    uint8_t i;
    for(i=0; i < 8; i++)
        rem = (rem & 0x8000)?((rem << 1) ^ 0x1021):rem << 1;

    return ((crc << 8) ^ rem);
}

#if 0
/* Generate CRC16 for a block of characters */
uint16_t crc16_block(uint16_t crc, uint8_t *b, size_t len)
{
    while(len--)
        crc = crc16_char(crc, *b++);

    return crc;
}
#endif

static uint8_t get_byte(enum memtype type, uint16_t addr)
{
    switch(type){
    case mem_ram:
        return *((uint8_t *)(addr));
        break;
    case mem_flash:
        return pgm_read_byte_near(addr);
    case mem_eeprom:
        EEAR = addr;
        EECR |= (1<<EERE);
        return EEDR;        
    default:
        return 0;
    }
}

uint16_t crc16_block2(uint16_t crc, enum memtype type, uint16_t addr, size_t size)
{
    size_t i;
    for(i=0; i < size; i++)
        crc = crc16_char(crc, get_byte(type, addr+i));

    return crc;
}

void copy_to_ram(enum memtype type, uint8_t *ram, uint16_t addr, size_t size)
{
    size_t i;
    for(i=0; i < size; i++)
        *ram++ = get_byte(type, addr+i);
}

int compare_to_ram(enum memtype type, uint8_t *ram, uint16_t addr, size_t size)
{
    size_t i;
    for(i=0; i < size; i++)
        if(*ram++ != get_byte(type, addr+i))
            return -1;

    return 0;
}
