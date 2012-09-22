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
#include <avr/wdt.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "bldc.h"
#include "twi.h"
#include "db.h"
#include "util.h"

/*### Firmware Version #######################################################*/

#define VERSION "0.01"

/*############################################################################*/

#if (BOOTLOADER == 1)
#error "BUILD code not appropriate for firmware"
#endif

#ifndef NO_BOOTLOADER
#define NO_BOOTLOADER 0
#endif

volatile uint8_t usr[2] __attribute__((section(".USERVAR")));
volatile uint8_t usr_reserved[6] __attribute__((section(".USERVAR")));

volatile const struct b_id1 firmwareid __attribute__((section(".INFO"))) = {
    BUILD,
    VERSION,
    MAGIC
};

volatile const uint16_t firm_crc __attribute__((section(".CRC"))) = 0x0000;
const uint8_t BLOCKSIZE = SIZEOF_BLOCK;

#if 0
/* work in progress, mostly notes */

typedef struct {

    /* parameters */
    uint8_t blanking;
    uint8_t min_v;

    #define DIR_CW()

    /* IDLE
     * START    I haven't worked out the start system yet...
     * RUN
     * BLANK    while blanking we sample vin and temperature
     *
     *  */
    #define IDLE        0
    #define START       1
    #define RUN         2
    #define BLANK       3
    #define UNDERVOLT   4
    #define LIMIT       5
    #define CW          0x80
    uint8_t state;

    #define FLAG_NOLOCK         0x01    /*BEMF not detected*/
    #define FLAG_UNDERVOLT      0x02    /*supply voltage less than minimum*/
    #define FLAG_CURRENT_L1     0x04    /*the last phase to current limit*/
    #define FLAG_CURRENT_L2     0x08
    #define FLAG_CURRENT_L3     0x10
    #define FLAG_OVERTEMP       0x11    /*not supported but reserved all the same*/
    uint8_t flags;

    uint8_t sect_b;     /*blanking counter*/

    uint16_t sect_m;        /*measured time from start of sector*/
    uint16_t sect_size;     /*sector size for next commute*/
    uint16_t sect;          /*time remaining until commute*/

    uint8_t v;              /*voltage / compare point*/
    uint8_t phase;          /*active phase*/


} pwm_driver_t;

pwm_driver_t pwm;

const uint8_t commute_table[2][6] PROGMEM = {
    {(1<<PWMB_LO_PIN), (1<<PWMC_LO_PIN)}},
    {PWMA_MASK, PWMB_MASK},
    {(1<<PWMC_LO_PIN), (1<<PWMA_LO_PIN)},
    {PWMB_MASK, PWMC_MASK},
    {(1<<PWMA_LO_PIN), (1<<PWMC_LO_PIN)},
    {PWMC_MASK, PWMA_MASK},
    {(1<<PWMC_LO_PIN), (1<<PWMB_LO_PIN)}
};


ISR(ADC_vect)
{
    uint8_t ph, on, off;

    pwm.sect_m++;

    switch(ADMUX & 0xf){
    case AMUX_VIN:  /*vin sampled during blanking time*/

        pwm.v = ADCH;   //add filter

        if(pwm.v < pwm.min_v)
            usr.flags |= FLAG_UNDERVOLT;
        else
            usr.flags &= ~FLAG_UNDERVOLT;

        switch(pwm.state){
        case BLANK:

            if(pwm.sect_b)pwm.sect_b--;
            else{
                pwm.state = RUN;
                ADMUX &= 0xf0;
                ADMUX |= commute_table[2][pwm.phase];
            }
            break;

        case UNDERVOLT:

            if(!(pwm.state & FLAG_UNDERVOLT)){

                //restart
            }
            break;

        case LIMIT:

            if(!(pwm.state & FLAG_LIMIT)){

                //restart
            }
            break
        }
        break;

    case AMUX_A:
    case AMUX_B:
    case AMUX_C:

        ph = ADCH;  //add filter

        /* count down until time to commute */
        if(pwm.sect){
            pwm.sect--;
        }
        else{

            pwm.phase++;
            if(pwm.phase == 6)
                pwm.phase = 0;

            if(pwm.phase & 0x1){

                on = commute_table[(DIR_CW())?1:0][pwm.phase];
                off = commute_table[(DIR_CW())?0:1][pwm.phase];
                pwm.phase = commute_table[2][pwm.phase];

                TCCR1E &= ~(off);
                TCCR1E |= on;
            }
            else{

                on = commute_table[(DIR_CW())?1:0][pwm.phase];
                off = commute_table[(DIR_CW())?0:1][pwm.phase];
                pwm.phase = commute_table[2][pwm.phase];

                PWMB_LO_PORT &= ~(off);
                PWMB_LO_PORT |= on;
            }

            /* load the new sector timing */
            pwm.sect = pwm.sect_m;
            /* load the new sector blanking */
            pwm.sect_b = pwm.blank;

            pwm.state = BLANK;
            ADMUX &= 0xf0;
            ADMUX |= AMUX_VIN;
        }

        /* detect zero crossing */
        if((pwm.rising && (ph >= pwm.v))||(!pwm.rising && (ph <= pwm.v))){

            pwm.next_sector = (pwm.ticks / 3);
            pwm.ticks = 0;
        }
    }


    //preload next compare point
}
#endif

static int usr_function(uint8_t cmd, uint16_t id, uint8_t *in, uint8_t *out, uint8_t len)
{
    int ret = 0;

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
                db_update_usr(0xff, FLAG_GOTOBOOT);
                wdt_enable(WDTO_15MS);  /*setup a wdt reset*/
                while(1);

            case STATE_IDLE:
            case STATE_FLIGHT:
            case STATE_SETUP:
                db_update_usr(STATE_MASK, in[1]);
                out[ret++] = PR03_RESP_SUCCESS;
                break;
            default:
                out[ret++] = PR03_RESP_TEMPFAIL;
            }
            break;

        case OBJ_STATE:
        case OBJ_BININFO:
        case OBJ_FIRM:
            out[ret++] = PR03_RESP_ACCESSDENIED;
            break;

        case OBJ_ADDRESS:

            if((len != 2) || (in[0] != PR03_DT_UINT8)){
                out[ret++] = PR03_RESP_TEMPFAIL;
                break;
            }

            if(db_set_address(in[1]))
                out[ret++] = PR03_RESP_TEMPFAIL;
            else
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
            out[ret++] = BLOCKSIZE;
            out[ret++] = 0x0;

            /* boot info */
            copy_to_ram(mem_flash, out+ret, ADDR_BOOT_START+SIZEOF_BOOT-sizeof(struct b_id1)-2, sizeof(struct b_id1));    /*bootloader info*/
            ret += sizeof(struct b_id1);

            /* firmware info */
            copy_to_ram(mem_flash, out+ret, ADDR_FIRM_START+SIZEOF_FIRM-sizeof(struct b_id1)-2, sizeof(struct b_id1));
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
    uint16_t set;
    int ret;

    /* host command */
    if((twidriver.cmd & 0xfe) == twidriver.addr){

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
    /* unicast or broadcast throttle command */
    else{

        /* read */
        if(twidriver.cmd & 0x1){

            /* can't read from the broadcast address */
            if((twidriver.cmd & 0xfe) == TWIADDR_BROADCAST)
                return;

            //fill response data here
        }
        /* write */
        else{

            /* broadcast setting;
             * {addr}[esc1_setting][esc2_setting]...[escN_setting]
             * position in vector depends on the ESC address */
            if((twidriver.cmd & 0xfe) == TWIADDR_BROADCAST){

                if(twidriver.inlen < twidriver.addr)
                    return;

                set = ((uint16_t)data[(twidriver.addr>>1)]) << 8;
                set |= data[(twidriver.addr>>1)+1];
            }
            /* unicast setting;
             * {addr}[esc_setting] */
            else{

                if(twidriver.inlen < 2)
                    return;

                set = ((uint16_t)data[0]) << 8;
                set |= data[1];
            }

            //change the throttle set point
        }
    }
}

void main(void)__attribute__((noreturn));

void main(void)
{
    wdt_enable(WDTO_1S);
    wdt_reset();

#if NO_BOOTLOADER

    /*power stage pins*/
    PWM_PORT |=  (1<<PWMA_HI) | (1<<PWMA_LO) | (1<<PWMB_HI) | (1<<PWMB_LO) |
        (1<<PWMC_HI) | (1<<PWMC_LO);
    PWM_DDR |=  (1<<PWMA_HI) | (1<<PWMA_LO) | (1<<PWMB_HI) | (1<<PWMB_LO) |
        (1<<PWMC_HI) | (1<<PWMC_LO);

    /* disable digital inputs to the analogue pins
     * ADC1, ADC3, AREF, ADC5, ADC6, ADC9 */
    DIDR0 = 0xda; DIDR1 = 0x40;
#endif

#if 1

    twi_init(db_get_addr());
    db_update_usr(STATE_MASK, STATE_IDLE);

#else
    /* work in progress */

    /* adc; enable and clock at 1MHz note. this will produce the max
     * advertised sample rate of 13us per conversion */
    ADCSRA = 0x80 | 0x4;
    /*left adjust; enable 2.54 reference with capacitor*/
    ADMUX = 0xe0; ADCSRB = 0x08;

    /* analog comparator for overcurrent fault
     * enable 1.1V reference for positive */
    ACSRA = 0x40;

    /*settings*/
    db_init();

    /*comms*/
    twi_init(db_get_addr());

    /*pwm*/

    /* 7 global pwm invert
     * 6 not sure
     * 5..4 deadtime prescaler
     * 3..0 clock selector/prescaler */
    TCCR1B = 0x0d;

    /* 7..6 phA mode
     * 5..4 phB mode
     * 3..2 phC mode
     */
    TCCR1C = 0x40 | 0x10 | 0x04;

    /* enable fault interrupt
     * enable fault function
     * enable noise 4 cycle cancel (review this...)
     * trip on rising edge
     * enable comparator to trip fault function
     * enable fast pwm6 mode
     */
    TCCR1D = 0x80 | 0x40 | 0x20 | 0x10 | 0x2 | 0x4;


    /* top */
    OCR1C = 0xff;

    /* 7..4 hi side dead time
     * 3..0 lo side dead time */
    DT1 = 0xff;


    /* self test here */

#endif

#if INTERRUPTS
    sei();
#endif    

    while(1){

        wdt_reset();

#if !INTERRUPTS

        /* start condition */
        if(USISR & (1<<USISIF))
            twi_start_irq();

        else if(twidriver.state){
            /* overflow condition */
            if(USISR & (1<<USIOIF))
                twi_overflow_irq();
#if TWI_STOP
            /* poll for stop condition */
            else
                twi_check_stop();
#endif
        }
#else
#if TWI_STOP
        twi_check_stop();
#endif
#endif
    }
}
