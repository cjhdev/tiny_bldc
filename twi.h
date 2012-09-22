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
#ifndef TWI_H
#define TWI_H

#include <stdint.h>

/* driver states */
typedef enum {
    TWI_IDLE = 0,   /*waiting for a start condition*/
    TWI_START,      /*waiting for SCL to become low in start condition*/
    TWI_ADDR,       /*receiving address/cmd byte*/
    TWI_RX_ACK,     /*just sent /ACK to master*/
    TWI_TX_ACK,     /*just read /ACK from master*/
    TWI_RX,         /*writing buffer*/
    TWI_TX,         /*sending buffer*/
    TWI_BUSY        /*running usr_process from mainloop*/
} twistate_t;

typedef struct {
    uint8_t addr;
    uint8_t cmd;        /* first byte of any message */

    uint8_t inlen;
    uint8_t outlen;

    twistate_t state;
    uint8_t size;

    uint8_t data[TWI_BUFSIZE];

} twi_driver_t;

extern volatile twi_driver_t twidriver;

/* for driving TWI without interrupts */
void twi_start_irq(void);
void twi_overflow_irq(void);

/* called before interrupts enabled */
void twi_init(uint8_t addr);

/* stop conditions must be polled */
void twi_check_stop(void);

/* application message handler */
void usr_process();

#endif
