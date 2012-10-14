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
#include <avr/eeprom.h>
#include <stddef.h>

#include "bldc.h"
#include "db.h"
#include "util.h"

#if !BOOTLOADER
void db_init(void)
{
}
#endif

uint8_t db_get_addr(void)
{
    struct db_addr addr;

    copy_to_ram(mem_eeprom, (unsigned char *)&addr, DB_HOSTADDR_START,
        sizeof(addr));

    if((addr.address[0] == (addr.address[1] ^ 0xff))
        && (addr.address[0] < (TWIADDR_FLIGHT_START>>1)))
        return (addr.address[0] << 1);
    else
        return TWIADDR_DEFAULT_HOST;
}

#if !BOOTLOADER

int db_set_address(uint8_t address)
{
    struct db_addr addr;

    if(address >= (TWIADDR_FLIGHT_START>>1))
        return -1;

    addr.address[0] = address;
    addr.address[1] = address ^ 0xff;

    eeprom_write_block(&addr, DB_HOSTADDR_START, sizeof(addr));

    return 0;
}
#endif

void db_update_usr(uint8_t clr, uint8_t set)
{
    usr[0] &= ~(clr);
    usr[0] |= set;
    usr[1] = usr[0] ^ 0xff;
}
