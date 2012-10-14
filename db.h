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
#ifndef DB_H
#define DB_H

#include <stdint.h>

#include "bldc.h"

#define DB_HOSTADDR_START   0   /*offset in eeprom of struct db_addr*/

/* eeprom assignment
 * 0..1: address structure
 *
 *
 * */
struct db_addr {
    uint8_t address[2];
};

void db_init(void);

/* get two-wire address from eeprom;
 * returns default if parity is bad or value is outside of acceptable host
 * range. */
uint8_t db_get_addr(void);

#if !BOOTLOADER
/* write a two-wire address to eeprom;
 * returns failure if value is outside of acceptable host range */
int db_set_address(uint8_t address);
#endif


/* update the shared user field;
 * bits are first cleared, then set */
void db_update_usr(uint8_t clr, uint8_t set);

#endif
