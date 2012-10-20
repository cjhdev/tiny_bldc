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
#ifndef BLDC_H
#define BLDC_H

/* build code system
 *
 * Lower word is the device id; Upper word identifies the software features
 *
 *
 * Lower Word: (device id)
 * Bit 0..15:
 * 0: B1 ESC
 *
 *
 * Upper Word:
 * Bit 16..23: (software version)
 * 0: Bootloader
 * 1: Firmware
 *
 * bit 24..31: (version features)
 *
 * (0 - Bootloader)
 * bit24: spare
 * bit25: spare
 * bit26: spare
 * bit27: spare
 * bit28: spare
 * bit29: spare
 * bit30: spare
 * bit31: spare
 *
 * (1 - Bootstrap)
 * bit24: spare
 * bit25: spare
 * bit26: spare
 * bit27: spare
 * bit28: spare
 * bit29: spare
 * bit30: spare
 * bit31: spare
 *
 * (2 - Firmware)
 * bit24: spare
 * bit25: spare
 * bit26: spare
 * bit27: spare
 * bit28: spare
 * bit29: spare
 * bit30: spare
 * bit31: spare
 *
 * */
#ifndef BUILD
#error "BUILD must be defined"
#endif

#if (BUILD & 0xffff)
#error "BUILD lower word should be 0x0000 for this ESC"
#endif  

#define BUILD_VERSION ((BUILD & 0xFF0000) >> 16)

/* 0 (bootloader) */
#if (BUILD_VERSION == 0)

#define INTERRUPTS  0

/* 1 (bootstrap) */
#elif (BUILD_VERSION == 1)

#define INTERRUPTS  0

/* 2 (firmware) */
#elif (BUILD_VERSION == 2)

#define INTERRUPTS  1

/* unknown */
#else

#error "unrecognised build version"

#endif

/* program space offsets */
#define SIZEOF_MCU      8192
#define SIZEOF_BOOT     2048
#define SIZEOF_FIRM     (SIZEOF_MCU-SIZEOF_BOOT)
#define SIZEOF_VECTOR   _VECTORS_SIZE

#define ADDR_BOOT_START 0
#define ADDR_FIRM_START SIZEOF_BOOT

/*PR02+PR03*/
#define COMM_OVERHEAD (2 + 4 + 2)

#include <stdint.h>

/*buffer size depends on page size and comm overhead*/
#define TWI_BUFSIZE (SPM_PAGESIZE + COMM_OVERHEAD)

/* shared variable field;
 *
 * bit7: enter bootloader control
 * bit6: firmware crc bad
 * bit5: bootloader crc bad
 * bit4..3: reserved
 * bit2..0: device state
 *  0: bootloader
 *  1: idle (application)
 *  2: flight
 *  3: setup
 *  4..7: reserved
 *
 * Shared between bootloader and application.
 * The compliment of this field is stored in the adjacent byte for validation.
 * */
#define FLAG_GOTOBOOT       0x80
#define FLAG_FIRMCRC        0x40
#define FLAG_BOOTCRC        0x20
#define STATE_MASK          0x07
#define STATE_BOOTLOADER    0
#define STATE_IDLE          1
#define STATE_FLIGHT        2
#define STATE_SETUP         3

extern volatile uint8_t usr[2];

/*two-wire addressing*/
#define TWIADDR_FLIGHT_START    (0x20 << 1)
#define TWIADDR_BROADCAST       (0x40 << 1)
#define TWIADDR_DEFAULT_HOST    (0x7e << 1)

/*pins and ports*/
#include <avr/io.h>

/*PWM*/
#define PWM_PORT    PORTB
#define PWM_DDR     DDRB

#define PWMA_HI     PIN5
#define PWMA_LO     PIN4
#define PWMB_HI     PIN3
#define PWMB_LO     PIN2
#define PWMC_HI     PIN1
#define PWMC_LO     PIN0

#define PWMA_MASK   0x03
#define PWMB_MASK   0x0c
#define PWMC_MASK   0x30

/* After the fault protection unit triggers, the pin mode needs to be
 * reapplied */
#define PWM_ENGAGE() do{TCCR1C = (1<<COM1A0S)|(1<<COM1B0S)|(1<<COM1D0);}while(0)

/* enable/disable the fault protection unit */
#define PWM_FP_ENABLE()     TCCR1D |= (1<<FPEN1);
#define PWM_FP_DISABLE()    TCCR1D &= ~(1<<FPEN1);    

/*two-wire ports*/
#define TWI_PORT    PORTA
#define TWI_DDR     DDRA
#define TWI_PIN     PINA

#define SDA_PIN     PIN0
#define SCL_PIN     PIN2

/* ADC mux channels */
#define AMUX_A
#define AMUX_B
#define AMUX_C
#define AMUX_I
#define AMUX_V
#define AMUX_T

/* start timer driven sampling */
#define ADC_ENGAGE()
#define ADC_DISENGAGE()

/*object IDs*/
#define OBJ_MODE            0xf000
#define OBJ_STATE           0xf001
#define OBJ_BININFO         0xf002
#define OBJ_ADDRESS         0xf003
#define OBJ_FIRM            0x1000

/* magic number to identify firmware from unprogrammed flash */
#define MAGIC {0xab,0xcd}

/* image id */
struct b_id1 {
    uint32_t build;     /* build code */
    uint8_t version[6]; /* "M.mmT" e.g. "1.01A" */
    uint8_t magic[2];   /* always {0xab,0xcd} */
}__attribute__((packed));

/*PR03 data type tags*/
typedef enum {
    /*primitive*/
    PR03_DT_NULL = 0x0,
    PR03_DT_UINT8,
    PR03_DT_INT8,
    PR03_DT_UINT16,
    PR03_DT_INT16,
    PR03_DT_UINT32,
    PR03_DT_INT32,
    PR03_DT_UINT64,
    PR03_DT_INT64,
    PR03_DT_FLOAT,
    PR03_DT_ENUM,
    PR03_DT_BOOL,
    PR03_DT_OCTET,
    PR03_DT_STRING,
    PR03_DT_STRUCT,
    PR03_DT_ARRAY
} pr03_dtype_t;
/*PR03 service response codes*/
typedef enum {
    PR03_RESP_SUCCESS = 0x0,
    PR03_RESP_TEMPFAIL,
    PR03_RESP_OBJECTNOTFOUND,
    PR03_RESP_ACCESSDENIED
} pr03_resp_t;
/*PR03 service tags*/
typedef enum {
    PR03_CONNECT_REQ = 0x0,
    PR03_CONNECT_CNF,
    PR03_RELEASE_REQ,
    PR03_RELEASE_CNF,
    PR03_WRITE_REQ,
    PR03_WRITE_CNF,
    PR03_READ_REQ,
    PR03_READ_CNF,
    PR03_ERROR_CNF
} pr03_primitive_t;
/*PR03 authentication methods*/
typedef enum {
    PR03_AUTH_NONE = 0,
    PR03_AUTH_PLAIN
} pr03_authmech_t;

#endif
