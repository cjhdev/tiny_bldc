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
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#include <avr/common.h>
#include <string.h>

#include "bldc.h"
#include "twi.h"
#include "db.h"
#include "util.h"

/*### Bootloader Version #####################################################*/

#define VERSION "0.01"

/*############################################################################*/

#if (BOOTLOADER == 0)
#error "BUILD code not appropriate for bootloader"
#endif

volatile uint8_t usr[2] __attribute__((section(".USERVAR")));
volatile uint8_t usr_reserved[6] __attribute__((section(".USERVAR")));

volatile const struct b_id1 boot_id __attribute__((section(".INFO"))) =
{
    BUILD,
    VERSION,
    MAGIC
};

volatile const uint16_t boot_crc __attribute__((section(".CRC"))) = 0x0000;

static int usr_function(uint8_t cmd, uint16_t id, uint8_t *in, uint8_t *out,
    uint8_t len)
{
    int ret = 0;
    uint16_t pos = 0;

    uint32_t page;
    uint8_t i;
    uint16_t w;

    /* get block address from id */
    if((id < (OBJ_FIRM+(SIZEOF_FIRM / SPM_PAGESIZE))) && (id >= OBJ_FIRM)){
        pos = (id - OBJ_FIRM);
        id = OBJ_FIRM;
    }

    /* write */
    if(cmd == PR03_WRITE_REQ){

        switch(id){
        case OBJ_MODE:

            if((len != 2) || (in[0] != PR03_DT_ENUM)){
                out[ret++] = PR03_RESP_TEMPFAIL;
                break;
            }

            switch(in[1]){
            case STATE_BOOTLOADER:
                out[ret++] = PR03_RESP_SUCCESS;
                break;
            case STATE_IDLE:
            case STATE_FLIGHT:
            case STATE_SETUP:
                db_update_usr(STATE_MASK, in[1]);
                wdt_enable(WDTO_15MS);  /* setup a wdt reset */
                while(1);
            default:
                out[ret++] = PR03_RESP_TEMPFAIL;
            }
            break;

        case OBJ_STATE:
        case OBJ_BININFO:
        case OBJ_ADDRESS:
            out[ret++] = PR03_RESP_ACCESSDENIED;
            break;

        /* program firmware; blocksize == page size */
        case OBJ_FIRM:

            if((len != (SPM_PAGESIZE+2))
                ||(in[0] != PR03_DT_OCTET)
                ||(in[1] != SPM_PAGESIZE)){
                out[ret++] = PR03_RESP_TEMPFAIL;
                break;
            }

            in += 2;

            page = ADDR_FIRM_START + (pos * SPM_PAGESIZE);

            boot_page_erase(page);
            boot_spm_busy_wait();

            for(i=0; i < SPM_PAGESIZE; i+=2){

                w = in[i+1];
                w <<= 8;
                w |= in[i];

                boot_page_fill(page+i, w);
            }

            boot_page_write(page);
            boot_spm_busy_wait();

            out[ret++] = PR03_RESP_SUCCESS;
            break;

        default:
            out[ret++] = PR03_RESP_OBJECTNOTFOUND;
        }
    }
    /* read */
    else{

        switch(id){
        case OBJ_MODE:
            out[ret++] = 0;
            out[ret++] = PR03_DT_ENUM;
            out[ret++] = *usr & 0x7;
            break;

        case OBJ_STATE:
            out[ret++] = 0;
            out[ret++] = PR03_DT_UINT8;
            out[ret++] = *usr;
            break;

        case OBJ_BININFO:
            out[ret++] = 0;
            out[ret++] = PR03_DT_OCTET;
            out[ret++] = 2 + (sizeof(struct b_id1)*2);

            /* block size (little endian) */
            out[ret++] = SPM_PAGESIZE;
            out[ret++] = 0x0;

            /*bootloader info*/
            copy_to_ram(mem_flash, out+ret,
                ADDR_BOOT_START+SIZEOF_BOOT-sizeof(struct b_id1)-2,
                sizeof(struct b_id1));
            ret += sizeof(struct b_id1);

            /* firmware info */
            copy_to_ram(mem_flash, out+ret,
                ADDR_FIRM_START+SIZEOF_FIRM-sizeof(struct b_id1)-2,
                sizeof(struct b_id1));
            ret += sizeof(struct b_id1);

            break;

        /*no need to be able to read these objects*/
        case OBJ_ADDRESS:
        case OBJ_FIRM:
            out[ret++] = 1;
            out[ret++] = PR03_RESP_ACCESSDENIED;
            break;

        default:
            out[ret++] = 1;
            out[ret++] = PR03_RESP_OBJECTNOTFOUND;
        }
    }

    return ret;
}

/* user interface */
void usr_process(void)
{
    uint8_t *data = (uint8_t *)twidriver.data;
    uint8_t cmd;
    uint16_t id;
    int ret;

    twidriver.size = 0;

    /* PR02 wrapper - check length and crc */
    if((twidriver.inlen < 2)
        ||((twidriver.inlen-2) != data[0])
        ||(crc8_block(crc8_char(0xff, twidriver.cmd), data, twidriver.inlen-1)
            != data[twidriver.inlen-1])
        )
        return;

    /* PR03_apdu */
    cmd = data[1];
    id = ((uint16_t)data[3]) << 8;
    id |= data[4];

    switch(cmd){
    case PR03_WRITE_REQ:

        if(!(ret = usr_function(cmd, id, data+5, data+3, data[0]-4)))
            return;

        data[0] = ret+2;    /* (service prim + invoke + payload) */
        data[1]++;
        break;

    case PR03_READ_REQ:

        if(!(ret = usr_function(cmd, id, NULL, data+3, TWI_BUFSIZE-4)))
            return;

        data[0] = ret+2;    /* (service prim + invoke + payload) */
        data[1]++;
        break;

    /*discard unkown commands*/
    default:
        return;
    }

    data[3+ret] = crc8_block(crc8_char(0xff, twidriver.cmd|0x1), data, 3+ret);
    twidriver.size = 3+ret+1;
}

void main(void)__attribute__((noreturn));

void main(void)
{
    uint8_t magic[] = MAGIC;

    wdt_enable(WDTO_1S);
    
    /*power stage pins*/
    PWM_PORT |=  (1<<PWMA_HI) | (1<<PWMA_LO) | (1<<PWMB_HI) | (1<<PWMB_LO) |
        (1<<PWMC_HI) | (1<<PWMC_LO);
    PWM_DDR |=  (1<<PWMA_HI) | (1<<PWMA_LO) | (1<<PWMB_HI) | (1<<PWMB_LO) |
        (1<<PWMC_HI) | (1<<PWMC_LO);

    /* disable digital inputs to the analogue pins
     * ADC1, ADC3, AREF, ADC5, ADC6, ADC9 */
    DIDR0 = 0xda; DIDR1 = 0x40;

    /* validate shared field */
    if(usr[0] != (usr[1] ^ 0xff))
        db_update_usr(0xff, 0x0);

    /* self check */
    if(crc16_block2(0xffff, mem_flash, ADDR_BOOT_START, SIZEOF_BOOT))
        db_update_usr(0x0, FLAG_BOOTCRC);
    else
        db_update_usr(FLAG_BOOTCRC, 0x0);

    /* check firmware */
    if(crc16_block2(0xffff, mem_flash, ADDR_FIRM_START, SIZEOF_FIRM)
        ||compare_to_ram(mem_flash, magic,
            ADDR_FIRM_START+SIZEOF_FIRM-sizeof(struct b_id1)-2+
                offsetof(struct b_id1, magic), sizeof(magic))
                )
        db_update_usr(0x0, FLAG_FIRMCRC);
    else
        db_update_usr(FLAG_FIRMCRC, 0x0);

    /* boot application? */
    if(!(*usr & (FLAG_FIRMCRC|FLAG_GOTOBOOT))){
        asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START));        
    }

    db_update_usr(FLAG_GOTOBOOT|STATE_MASK, STATE_BOOTLOADER);

    /* comms */
    twi_init(db_get_addr());

    while(1){

        wdt_reset();

        /* start condition */
        if(USISR & (1<<USISIF))
            twi_start_irq();

        else if(twidriver.state){
            /* overflow condition */
            if(USISR & (1<<USIOIF))
                twi_overflow_irq();
        }
    }
}

/* just a link into the .initN chain */
void __init0(void) __attribute__((naked)) __attribute__ ((section (".init0")));
void __init0(void){
    /* clear the zero register */
    asm volatile("clr r1\n");

    /* SREG already cleared on ATTINYx61 */

    /* SP already at RAMEND on ATTINYx61 */    
}

/* we turn off the default start file because it puts stuff in the vector
 * table */
void __init9(void) __attribute__((naked)) __attribute__ ((section (".init9")));
void __init9(void)
{
    asm volatile ("rjmp main\n");
}

/* jump to the application vector table */
void vector_table(void) __attribute__((naked))
    __attribute__ ((section (".vectors")));
void vector_table(void)
{
    asm volatile ("rjmp __init0\n");
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+2));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+4));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+6));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+8));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+10));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+12));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+14));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+16));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+18));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+20));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+22));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+24));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+26));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+28));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+30));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+32));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+34));
    asm volatile ("rjmp %0\n" :: "p" (ADDR_FIRM_START+36));
}
