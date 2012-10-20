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

    /* blanking cycles between commutes - minimum of 2 */
    uint8_t blanking;   

    /* minimum supply voltage alarm */
    uint8_t min_v_lower;      
    uint8_t min_v_upper;      

    /* temperature alarm and limit setting */
    uint8_t max_t_lower;    
    uint8_t max_t_upper
    uint16_t t_limit;   
    
    /* bit 7: temperature limiting enabled
     * bit 6: voltage limiting enabled
     *
     * */
    uint8_t limit_mode;

    uint16_t min_pwm;   /* minimum allowable running setpoint */
    uint16_t max_pwm;   /* maximum allowable running setpoint */
    uint16_t set_pwm;   /* last set point */

    /* align setting */
    uint16_t t_align;   /* align period (ticks) */
    uint16_t P_align;   /* align set point */
    uint16_t dt_align;  /* align ramp period */
    uint16_t dP_align;  /* align ramp PWM */

    /* start setting (follows align) */
    uint16_t dt_start;  /* start ramp period */
    uint16_t dP_start;  /* start ramp PWM */
    uint16_t T_start;   /* start commute period */
    uint16_t dT_start;  /* start commute ramp */

    /* sync setting (follows start) */
    uint16_t sync_count;  /* number of correct commutations before mode change */
    
    uint8_t adc_ph;         /* phase voltage result */
    uint8_t adc_v;          /* voltage / compare point */
    uint8_t adc_t;          /* temperature */

    

    #define COMMUTE     0x80    /*actively blanking*/
    #define I_LIMIT     0x40    /*current limit*/
    #define T_LIMIT     0x20    /*temperature limit*/
    #define V_LIMIT     0x10    /*voltage limit*/
    #define IDLE        0       
    #define ALIGN       1   /* aligning the rotor */
    #define START       2   /* open loop commutation */
    #define SYNC1       3   /* ready to close loop, waiting for next frame */
    #define SYNC2       4   /* detecting first crossing */
    #define SYNC3       5   /* collecting valid crossings */
    #define RUN         6   /* closed loop */
    uint8_t state;

    /* State Machine:
     *
     * D = duty cycle
     * T = commute frame width
     * 
     * IDLE:
     * - D is 0 or less than the minimum run D
     * - To transition, user must set D equal to or greater
     *   than minimum D
     *
     * ALIGN:
     * - D is set to P_align on the starting phase
     * - Every dt_align ticks, D incremented by dP_align
     * - State advances after t_align ticks
     *
     * START:
     * - Every dt_start ticks, D incremented by dP_start
     * - T set to T_start
     * - T decremented by dT_start every 
     * 
     * SYNC1:
     * - Wait for start of next commute frame
     *
     * SYNC2:
     * - Wait to detect first zero crossing
     * 
     * SYNC3:
     * - Wait to detect 3 successive zero crossings
     *
     * RUN:
     * - Running from zero cross detector
     *
     */



    uint16_t tmr_start; /* period timer */
    uint16_t tmr_ramp;  /* acceleration timer */


    uint8_t dt;
    uint8_t dp;

    #define X_RISING    0x80   /* rising (1); falling (0) */
    #define X_CROSSED   0x40   /* cross detected this frame */
    uint8_t x_state;

    uint8_t tmr_blank;      /* blanking counter */
    uint8_t phase;          /* active phase */

    uint16_t tmr_measure;
    uint16_t tmr_commute;

    uint16_t sector;

} pwm_driver_t;

volatile pwm_driver_t pwm;


/* CW:
 * 
 * 1. A -> C; measure B;
 * 2. B -> C; measure A;
 * 3. B -> A; measure C;
 * 4. C -> A; measure B;
 * 5. C -> B; measure A;
 * 6. A -> B; measure C;
 * 
 * CCW:
 *
 * 1. C -> A; measure B; 
 * 2. C -> B; measure A;
 * 3. A -> B; measure C;
 * 4. A -> C; measure B;
 * 5. B -> C; measure A;
 * 6. B -> A; measure C;
 */
const uint8_t commute_table[][] = {
    {(1<<PWMB_LO_PIN),  (1<<PWMC_LO_PIN), AMUX_B},
    {PWMA_MASK,         PWMB_MASK,        AMUX_A},
    {(1<<PWMC_LO_PIN),  (1<<PWMA_LO_PIN), AMUX_C},
    {PWMB_MASK,         PWMC_MASK,        AMUX_B},
    {(1<<PWMA_LO_PIN),  (1<<PWMC_LO_PIN), AMUX_A},
    {PWMC_MASK,         PWMA_MASK,        AMUX_C}    
};

ISR(ADC_vect)
{
    uint8_t cross = 0;

    switch(ADMUX & 0x1f){
    case AMUX_V:  
    case AMUX_T:
    case AMUX_I:

        switch(ADMUX & 0x1f){
        case AMUX_V:

            pwm.adc_v = ADCH;   

            /* under voltage limiting */
            if(pwm.state & V_LIMIT){

                if(pwm.adc_v > pwm.min_v_upper)
                    pwm.state &= ~(V_LIMIT);
            }
            else if(pwm.adc_v < pwm.min_v_lower){
                    pwm.state |= V_LIMIT;
            }

            ADMUX &= 0xe0;
            ADMUX |= AMUX_T;
            break;

        case AMUX_T:
        
            pwm.adc_t = ADCH;   

            /* over temperature limiting */
            if(pwm.state & I_LIMIT){

                if(pwm.adc_t < pwm.max_t_lower)
                    pwm.state = ~(I_LIMIT);
                
            }
            else if(pwm.adc_t > pwm.max_t_upper){

                pwm.state |= I_LIMIT;

                if((pwm.limit_mode & 0x80) && (pwm.setpoint > pwm.t_limit))
                    pwm.setpoint = pwm.t_limit;                
            }

            ADMUX &= 0xe0;
            ADMUX |= AMUX_V;
            break;
            
        case AMUX_I:
            
            /* can current limit be reset? */
            if(ADCH > ((1.1 * 0xff)/2.5)){
                //yes
            }
            
            ADMUX &= 0xe0;
            ADMUX |= AMUX_V;            
        }

        /* align */
        if((pwm.state & 0x7) == ALIGN){

            /* ramp the align duty */
            if(pwm.tmr_ramp)pwm.tmr_ramp--;
            else{

                pwm.tmr_ramp = pwm.dt_align;                    
                OCR1A += pwm.dP_align;                
            }

            /* wait for next state */
            if(pwm.tmr_start)pwm.tmr_start--;
            else{

                pwm.state++;    /* start */

                /* setup commute timer to interrupt immediately */
                OCR0A = TCNT0L;
                TIMSK |= (1<<OCIE0A);
                TIFR |= (1<<OCF0A);

                pwm.tmr_start = pwm.t_start;
                pwm.tmr_ramp = pwm.dt_start;
            }
        }

        /* start */
        if((pwm.state & 0x7) == START){

            /* ramp the start duty */
            if(pwm.tmr_ramp)pwm.tmr_ramp--;
            else{

                pwm.tmr_ramp = pwm.dt_start;                    
                OCR1A += pwm.dP_start;                
            }

            /* wait for next state */
            if(pwm.tmr_start)pwm.tmr_start--;
            else{

                pwm.state++;    /* sync1 (wait for next frame) */

                /* sample the phase voltage */
                ADMUX &= 0xe0;
                ADMUX |= commute_table[pwm.phase][2];

                /* clear measurement */
                pwm.tmr_measure = 0;
            }
        }

        /* blanking timeout */
        if(((pwm.state & 0x7) >= SYNC) && (pwm.state & COMMUTE)){
        
            if(!pwm.tmr_blank){

                pwm.state &= ~(COMMUTE);
                
                ADMUX &= 0xe0;
                ADMUX |= commute_table[pwm.phase][2];                
            }
            else{
                pwm.tmr_blank--;
            }
        }
        
        break;

    case AMUX_A:
    case AMUX_B:
    case AMUX_C:

        /* discard this result if we are meant to be blanking */
        if(pwm.state & COMMUTE){
            ADMUX &= 0xe0;
            ADMUX |= AMUX_V;
            break;
        }
        
        pwm.adc_ph = ADCH;  

        /* look for the crossing */
        if(!(pwm.x_state & X_CROSSED)){

            if(pwm.x_state & X_RISING){

                if(pwm.ph >= pwm.v)
                    pwm.x_state |= X_CROSSED;
            }
            else{

                if(pwm.ph <= pwm.v)
                    pwm.x_state |= X_CROSSED;
            }

            if(pwm.x_state & X_CROSSED){

                pwm.sect = pwm.tmr_measure;
                pwm.phase = pwm.tmr_phase;
                pwm.tmr_measure = 0;
                
                if((pwm.state & 0xf) < RUN)
                    pwm.state++;
            }
        }
    }    

    if((pwm.state & 0x7) > SYNC1){
        pwm.tmr_measure++;
        
    }
}

/* commute timer */
ISR(TIMER0_COMPA_vect)
{
    /* set next commute frame */
    OCR0A += pwm.frame;

    pwm.phase++;
    if(pwm.phase == 6)
        pwm.phase = 0;

    if(pwm.phase & 0x1){

        on = commute_table[(pwm.state & CW)?1:0][pwm.phase];
        off = commute_table[(pwm.state & CW)?0:1][pwm.phase];
        
        TCCR1E &= ~(off);
        TCCR1E |= on;
    }
    else{

        on = commute_table[(pwm.state & CW)?1:0][pwm.phase];
        off = commute_table[(pwm.state & CW)?0:1][pwm.phase];
        
        PWM_PORT &= ~(off);
        PWM_PORT |= on;
    }

    /* Prepare for commute blanking */
    pwm.tmr_blank = pwm.blanking;
    pwm.state |= COMMUTE;

    pwm.tmr_phase = 0;  /* phase offset */

    pwm.x_state &= ~(X_CROSSED);
    pwm.x_state ^= X_RISING;
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

                set = data[(twidriver.addr>>1)];
                set <<= 8;
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

#if 0            
            pwm.set_pwm = set;

            /* apply the new set point */
            if(!(pwm.state & 0x7)){

                /* do nothing */
                if(set < pwm.pwm_min)
                    return;

                cli();

                /* align mode */
                pwm.mode++;
                OCR1A = pwm.pwm_align;
                pwm.tmr_start = pwm.align_ticks;

                sei();
            }
            else if((pwm.state & 0x7) == RUN){
                OCR1A = set & 0x3ff;
            }
#endif            
        }
    }
}

/* adds in some startup code if you don't have a bootloader */
#ifndef NO_BOOTLOADER
#define NO_BOOTLOADER 0
#endif

void main(void)__attribute__((noreturn));

void main(void)
{
#if NO_BOOTLOADER

    //wdt_enable(WDTO_1S);

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
