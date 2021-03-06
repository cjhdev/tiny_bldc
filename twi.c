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
#include "bldc.h"
#include "twi.h"

#include <stdint.h>
#include <string.h>
#include <avr/io.h>

#if INTERRUPTS
#include <avr/interrupt.h>
#include <util/atomic.h>
#endif

/* release the bus, return to idle */
#define TWI_NACK() do{\
    USICR &= ~((1<<USIWM0)|(1<<USIOIE));\
    twidriver.state = TWI_IDLE;}while(0)

/* produce an /ACK;
 * clear data register
 * set SDA to output
 * set counter to 0xe */
#define TWI_ACK() do{\
    USIDR = 0x00;\
    TWI_DDR |= (1<<SDA_PIN);\
    USISR |= ((1<<USICNT1)|(1<<USICNT2)|(1<<USICNT3));\
    twidriver.state = (twidriver.cmd & 0x1)?TWI_TX_ACK:TWI_RX_ACK;\
    while(TWI_PIN & SDA_PIN);}while(0)

/* setup USI to read an /ACK;
 * set SDA to input
 * set counter to 0xe */
#define TWI_GET_ACK() do{\
    TWI_DDR &= ~(1<<SDA_PIN);\
    USISR |= ((1<<USICNT1)|(1<<USICNT2)|(1<<USICNT3));\
    twidriver.state = TWI_TX_ACK;}while(0)

volatile twi_driver_t twidriver;

/* start condition */
#if INTERRUPTS
ISR(USI_START_vect)
#else
void twi_start_irq(void)
#endif
{
#if INTERRUPTS
    uint8_t sreg;
#endif    

    TWI_DDR &= ~(1<<SDA_PIN);

    /* if previously recieving, go and process the message;
     * Nest interrupts here so the usr_process can take as long as needed. */
    if(twidriver.inlen){
#if INTERRUPTS
        /* turn off both twi interrupts */
        USICR &= ~((1<<USISIE)|(1<<USIOIE));
        sreg = SREG;
        sei();
#endif           
        usr_process();
        twidriver.inlen = 0;
#if INTERRUPTS
        cli();
        SREG = sreg;
        /* reenable start interrupt */
        USICR |= (1<<USISIE);        
#endif        
    }

    /* start condition doesn't mean that SCL is low yet. For
     * this reason we setup the overflow interrupt to catch the falling edge
     * of SCL if it ever happens.
     */

    /* overflow interrupt enable and hold SCL when it happens */
    USICR |= (1<<USIWM0)|(1<<USIOIE);

    /* Is SCL still high? */
    if(TWI_PIN & (1<<SCL_PIN)){

        /* instead of waiting, preload counter to detect falling SCL */
        USISR |= (1<<USICNT0)|(1<<USICNT1)|(1<<USICNT2)|(1<<USICNT3)|
            (1<<USIOIF);

        twidriver.state = TWI_START;
    }

    /* SCL became low since we last checked above. Do everything the overflow
     * handler would do and then advance the state */
    if(!(TWI_PIN & (1<<SCL_PIN))){

        /* clear overflow bit counter */
        USISR &= ~((1<<USICNT0)|(1<<USICNT1)|(1<<USICNT2)|(1<<USICNT3));

        twidriver.state = TWI_ADDR;

        /* make sure the overflow interrupt is cleared */
        USISR |= (1<<USIOIF);
    }
    
    /* clear this flag */
    USISR |= (1<<USISIF)|(1<<USIPF);
}

/* data counter overflow */
#if INTERRUPTS
ISR(USI_OVF_vect)
#else
void twi_overflow_irq(void)
#endif
{
#if INTERRUPTS
    uint8_t sreg;
#endif    
    uint8_t buf = USIDR;

    switch(twidriver.state){
    case TWI_START:

        /* clear overflow bit counter */
        USISR &= ~((1<<USICNT0)|(1<<USICNT1)|(1<<USICNT2)|(1<<USICNT3));

        twidriver.state = TWI_ADDR;
        break;

    case TWI_ADDR:

        /* message for this device... */
        if( ((buf & 0xfe) == twidriver.addr)
#if (BUILD_VERSION > 0)
            ||((buf & 0xfe) == TWIADDR_BROADCAST)
            ||( (twidriver.addr < TWIADDR_FLIGHT_START) &&
                ((buf & 0xfe) == (twidriver.addr+TWIADDR_FLIGHT_START)))
#endif
            ){

            /* if reading, something needs to be ready in the buffer */
            if(buf & 0x1){
#if (BUILD_VERSION > 0)
                /* flight protocol */
                if((buf & 0xfe) != twidriver.addr){
#if INTERRUPTS                    
                    USICR &= ~(1<<USISIE);
                    sreg = SREG;
                    sei();
#endif                    
                    usr_process();
#if INTERRUPTS                    
                    cli();                    
                    SREG = sreg;
                    USICR |= (1<<USISIE);
#endif                    
                }
                else
#endif
                /* host protocol */
                if(!twidriver.size){

                    TWI_NACK();
                    break;
                }
            }

            /* remember cmd byte and zero the length counter */
            twidriver.cmd = buf;
            twidriver.inlen = twidriver.outlen = 0;

            /* do an /ACK; */
            TWI_ACK();
        }
        /* message for some other device */
        else{

            TWI_NACK();
        }
        break;

    /* just read the ACK bit from master... */
    case TWI_TX_ACK:

        /* master did not /ACK */
        if(twidriver.outlen && (buf & 0x1)){

            TWI_NACK();
        }
        /* master /ACKed */
        else{

            TWI_DDR |= (1<<SDA_PIN);

            /* load next byte or handle overflow */
            if(twidriver.outlen < twidriver.size)
                USIDR = twidriver.data[twidriver.outlen++];
            else
                USIDR = 0x55;

            twidriver.state = TWI_TX;
        }
        break;

    /* just sent the ACK bit to master... */
    case TWI_RX_ACK:

        TWI_DDR &= ~(1<<SDA_PIN);
        twidriver.state = TWI_RX;
        break;

    /* master is writing to the buffer */
    case TWI_RX:

        /* buffer if there is space */
        if(twidriver.inlen < TWI_BUFSIZE){
            twidriver.data[twidriver.inlen++] = buf;

            TWI_ACK();
        }
        /* give up if master overruns */
        else{

            TWI_NACK();
        }
        break;

    /* master is reading from the buffer */
    case TWI_TX:

        TWI_GET_ACK();
        break;

    default:
        break;
    }

    USISR |= (1<<USIOIF);   /* clear overflow flag */
}

void twi_init(uint8_t addr)
{
    twidriver.state = TWI_IDLE;
    twidriver.size = 0;

    USIPP |= (1<<USIPOS);   /* enable TWI pins on PA2:PA0 */

    /* 1. interrupt on start condition
     * 2. hold SCL on start condition
     * 3. sample data on rising edge, count both edges */
    USICR = (1<<USISIE)|(1<<USIWM1)|(1<<USICS1);

    TWI_DDR |= (1<<SCL_PIN);
    TWI_PORT |= (1<<SCL_PIN)|(1<<SDA_PIN);

    /* clear all flags */
    USISR |= (1<<USIOIF)|(1<<USISIF)|(1<<USIPF);

    twidriver.state = TWI_IDLE;
    twidriver.addr = addr;
}
