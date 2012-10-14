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

volatile const struct b_id1 firmwareid __attribute__((section(".INFO"))) =
{
    BUILD,
    VERSION,
    MAGIC
};

volatile const uint16_t firm_crc __attribute__((section(".CRC"))) = 0x0000;

#if 0
typedef struct {

    /* parameters */
    uint8_t blanking;   /* blanking cycles between commutes */
    uint8_t min_v;      /* minimum supply voltage alarm */
    uint8_t max_t;      /* maximum temperature alarm */

    
    /* running variables */
    uint8_t ph;         /* phase voltage result */
    uint8_t v;          /*voltage / compare point*/
    
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
    #define RISING      0x40
    #define CW          0x80
    uint8_t state;

    #define FLAG_NOLOCK         0x01    /*BEMF not detected*/
    #define FLAG_UNDERVOLT      0x02    /*supply voltage less than minimum*/
    #define FLAG_CURRENT_L1     0x04    /*the last phase to current limit*/
    #define FLAG_CURRENT_L2     0x08
    #define FLAG_CURRENT_L3     0x10
    #define FLAG_OVERTEMP       0x20    
    uint8_t flags;

    uint8_t tmr_blank;      /* blanking counter */
    uint8_t phase;          /*active phase*/

} pwm_driver_t;

volatile pwm_driver_t pwm;


/* Sample handler
 *
 * The ADC is manually triggered with each successive sample triggering the
 * next. This frees up a timer where it isn't really required and allows the
 * ADC to run as fast as the prescaler (and other higher priority interrupts)
 * will allow it. In practice the sample frequency becomes predictable but
 * will not be as precise as a timer driven ADC.
 * 
 * During the commutation blanking period the interrupt samples Vin and
 * temperature. These results are compared to the programmed limit values and
 * may set warning flags and/or limit the throttle setting depending on device
 * configuration.
 *
 * The rest of the time phase voltage is sampled and compared to Vin to detect
 * a zero crossing.
 *
 * timer0 is used to measure the time between zero crossings.
 *
 */
ISR(ADC_vect)
{
    uint8_t cross = 0;

    switch(ADMUX & 0xf){
    case AMUX_VIN:  
    case AMUX_TEMP:

        if((ADMUX & 0xf) == AMUX_VIN)
            pwm.v = ADCH;   //add filter
        else
            pwm.t = ADCH;   //add filter

        if(pwm.v < pwm.min_v)
            usr.flags |= FLAG_UNDERVOLT;
        else
            usr.flags &= ~FLAG_UNDERVOLT;

        switch(pwm.state){
        case BLANK:

            if(!pwm.sect_b){
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

        pwm.ph = ADCH;  

        /* detect zero crossing...hmmm */
        if(pwm.flags & RISING){
            if(pwm.ph >= pwm.v){
                pwm.flags &= ~(RISING);
                cross = 1;
            }
        }
        else{
            if(pwm.ph <= pwm.v){
                pwm.flags |= RISING;
                cross = 1;
            }
        }

        if(cross){

            /*halt the timer*/
            TCCR0B |= (1 << TSM);

            /* not locked - this is the first crossing */
            if(pwm.flags & FLAG_NOLOCK){
                
            }
            /* if locked, save the measured time*/
            else{
                pwm.sect = (TCNT0 / 3);
            }

            /* has the commute point been serviced yet? (unlikely) */
            if(!(TIFR & (1 << OCF0A))){

                /* reset compare point relative to cleared timer */
                if(OCR0 < TCNT0)
                    OCR0 += 0xffff - TCNT0;
                else
                    OCR0 -= TCNT0;

                /* ensure overflow never happened */
                TIFR |= (1 << OCF0A);
            }
                
            /* clear the counter to begin measuring again */
            TCTN0 = 0;
            
            /* restart timer */
            TCCR0B &= ~(1 << TSM);                
        }        
    }

    /* start next conversion */
    ADCSRA |= (1 << ADSC);
}

/* every commute timeout a signal is turned on and another is turned off
 * depending on the phase */
const uint8_t commute_table[2][6] = {
    {(1<<PWMB_LO_PIN), (1<<PWMC_LO_PIN)}},
    {PWMA_MASK, PWMB_MASK},
    {(1<<PWMC_LO_PIN), (1<<PWMA_LO_PIN)},
    {PWMB_MASK, PWMC_MASK},
    {(1<<PWMA_LO_PIN), (1<<PWMC_LO_PIN)},
    {PWMC_MASK, PWMA_MASK},
    {(1<<PWMC_LO_PIN), (1<<PWMB_LO_PIN)}
};

/* an overflow means that sync was lost, the control algorithm must be restarted */
IST(TIMER0_OVF_vect)
{
    pwm.flags |= FLAG_NOLOCK;
    TIMSK &= ~(TOIE0);
}

/* time to commute the next phase */
ISR(TIMER0_COMPA_vect)
{
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

    /* next commute time */
    OCR0 += pwm.sect;

    /* load the new sector blanking */
    pwm.tmr_blank = pwm.blank;
    pwm.state = BLANK;

    ADMUX &= 0xf0;
    ADMUX |= AMUX_VIN;
}
#endif

static int usr_function(uint8_t cmd, uint16_t id, uint8_t *in, uint8_t *out,
    uint8_t len)
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
            out[ret++] = SPM_PAGESIZE;
            out[ret++] = 0x0;

            /* boot info */
            copy_to_ram(mem_flash, out+ret,
                ADDR_BOOT_START+SIZEOF_BOOT-sizeof(struct b_id1)-2,
                sizeof(struct b_id1));    /*bootloader info*/
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
    uint16_t set;
    int ret;

    twidriver.size = 0;

    /* host command */
    if((twidriver.cmd & 0xfe) == twidriver.addr){

        /* PR02 wrapper - check length and crc */
        if((twidriver.inlen < 2)
            ||((twidriver.inlen-2) != data[0])
            ||(crc8_block(crc8_char(0xff, twidriver.cmd), data,
                twidriver.inlen-1) != data[twidriver.inlen-1])
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

        data[3+ret] = crc8_block(crc8_char(0xff, twidriver.cmd|0x1), data,
            3+ret);
        twidriver.size = 3+ret+1;
    }
    /* unicast or broadcast throttle command */
    else{

        /* read */
        if(twidriver.cmd & 0x1){

            /* can't read from the broadcast address */
            if((twidriver.cmd & 0xfe) == TWIADDR_BROADCAST)
                return;

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

            /* apply the new set point */
            OCR1A = set & 0x3ff;
        }
    }
}

void main(void)__attribute__((noreturn));

void main(void)
{
    //wdt_enable(WDTO_1S);
    
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
    
    db_update_usr(STATE_MASK, STATE_IDLE);

    /* settings */
    db_init();

    /* comms */
    twi_init(db_get_addr());
#if 0
    /* adc;
     * enable
     * clock = 16Mhz / 16 == 13us conversion time */
    ADCSRA = (1<<ADEN)|(1<<ADPS2);

    /* left adjust;
     * enable 2.54 reference with capacitor */
    ADMUX = (1<<REFS0)|(1<<REFS1);
    ADCSRB = (1<<REFS2);
    
    /* ac;
     * enable 1.1V reference for positive */
    ACSRA = (1<<ACBG);

    
    /* PWM module */

    /* PWM1X - Invert all PWM ports
     * DTPS11:DTPS10 - dead time prescaler (x1,x2,x4,x8)
     * CS13:CS12:CS11:CS10 - clock prescale 0 to 16384
     * 
     * 64MHz / 4096 */
    TCCR1B = (1<<PWM1X) | (1<<CS13) | (1<<CS12) | (1<<CS10);

    /* Pin mode
     * 
     * COM1A1S:COM1A0S (0:1) set 0x000, clear at compare match
     * COM1B1S:COM1B0S (0:1) set 0x000, clear at compare match
     * COM1D1:COM1D0 (0:1) set 0x000, clear at compare match
     *
     * Note that after the FP unit triggers, these modes are cleared
     * to disconnect the output
     */
    PWM_ENGAGE();

    /* Fault protection
     *
     * FPEN1 - enable fault protect
     * FPNC1 - 4 cycle noise canceller
     * FPES1 - edge select (1 rising / 0 falling)
     * FPAC1 - source from analog comparator output
     *
     * setup the FP here but do not enable
     * 
     * Wave Generation Mode
     * 
     * WGM11:WGM10 (1:0) PWM6 /single slope
     * WGM11:WGM10 (1:1) PWM6 /dual slope (unused)
     *
     * */
    TCCR1D = |(1<<FPNC1)|(1<<FPES1)|(1<<FPAC1)|(1<<WGM11);

    /* Set top to maximum resolution */
    OCR1C = 0xff;

    /* Dead time
     * 
     * 7..4 high side dead time
     * 3..0 low side dead time
     *
     * 0..16 x DT prescale x timer tick
     *
     * */
    DT1 = 0xff;


    /* motor test
     *
     * 1: Conductivity on all phases
     * For each phase, set hi, measure voltage on remaining 3 phases
     *
     * todo.
     *
     * */


    /* timer0;
     * 16bit mode /64 prescale
     */
    TCCR0B = (1<<TCW0) | (1<<CS01) | (1<<CS00);
#endif

    sei();

    
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
        }
#endif        
    }
}
