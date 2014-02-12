/*
 * config.h
 *
 * Project: FloodBrain
 * Copyright (c) 2014 Shelby Merrick
 * http://www.forkineye.com
 *
 *  This program is provided free for you to use in any way that you wish,
 *  subject to the laws and regulations where you are using it.  Due diligence
 *  is strongly suggested before using this code.  Please give credit where due.
 *
 *  The Author makes no warranty of any kind, express or implied, with regard
 *  to this program or the documentation contained in this document.  The
 *  Author shall not be liable in any event for incidental or consequential
 *  damages in connection with, or arising out of, the furnishing, performance
 *  or use of these programs.
 *
 */ 

#ifndef CONFIG_H_
#define CONFIG_H_

/* Set our clock define so delay functions are happy */
#define F_CPU   32000000UL

#include "RingBuffer.h"
#include "XSPI.h"
#include "XUSART.h"
#include "XNRF24L01.h"
#include "RENARD.h"

/* XNRF24L01 Config */
#define NRF_CHANNEL 100             /* Default nRF channel */
#define NRF_RATE    XNRF_250KBPS    /* Default nRF data rate */
#define ADDR_P0     0xF0F0F0F0E1LL  /* Default Pipe 0 address */
#define ADDR_P1     0xF0F0F0F0D2LL  /* Default Pipe 1 address */
#define NRF_CBITS   0b00111100      /* Default configuration bits - 2 byte CRC, RX_DR enabled */
/*                    ^^^^^^^^
 *                    ||||||||_____ PRIM_RX - RX/TX control
 *                    |||||||______ PWR_UP - Power control         
 *                    ||||||_______ CRCO - CRC encoding scheme; '0' - 1 byte, '1' - 2 bytes
 *                    |||||________ EN_CRC - Enable CRC
 *                    ||||_________ MASK_MAX_RT - Reflect max retry on IRQ pin - '0' to enable
 *                    |||__________ MASK_TX_DS - Reflect TX data sent on IRQ pin - '0' to enable
 *                    ||___________ MASK_RX_DR - Reflect RX data received on IRQ pin - '0' to enable
 *                    |____________ RESERVED - Only '0' allowed
 */

/* XUSART Config */
#define USART_BAUDRATE  115200      /* Default RS485 baudrate */

/* RingBuffer Config */
#define BUFFER_SIZE     255         /* Ringbuffer size */

/* Pin configuration for FloodBrain
 *    PA0  ........ CH1
 *    PA1  ........ CH2
 *    PA2  ........ CH3
 *    PA3  ........ CH4
 *    PA4  ........ CH5
 *    PA5  ........ CH6
 *    PA6  ........ CH7
 *    PA7  ........ CH8
 *
 *    PC0  ........ STATUS LED
 *    PC1  ........ DATA LED
 *    PC2  ........ NRF IRQ
 *    PC3  ........ NRF CE
 *    PC4  ........ NRF SEL
 *    PC5  ........ NRF SCLK
 *    PC6  ........ MISO/RX
 *    PC7  ........ MOSI/TX
 *
 *    PD0  ........ CH9
 *    PD1  ........ CH10
 *    PD2  ........ CH11
 *    PD3  ........ CH12
 *    PD4  ........ CH13
 *    PD5  ........ CH14
 *    PD6  ........ CH15
 *    PD7  ........ CH16
 *
 *    PR0  ........ CH17
 *    PR1  ........ CH18
 *
 */

// Status LEDs
#define LED_STATUS_ON   PORTC.OUTCLR = PIN0_bm
#define LED_STATUS_OFF  PORTC.OUTSET = PIN0_bm
#define LED_STATUS_TGL  PORTC.OUTTGL = PIN0_bm

#define LED_DATA_ON     PORTC.OUTCLR = PIN1_bm
#define LED_DATA_OFF    PORTC.OUTSET = PIN1_bm
#define LED_DATA_TGL    PORTC.OUTTGL = PIN1_bm

// port definitions
#define NUM_CHANNELS 18

#define PWM_BITOP &= ~

#define CH1_SET (portlevelA &= ~(1 << PIN0_bp))
#define CH2_SET (portlevelA &= ~(1 << PIN1_bp))
#define CH3_SET (portlevelA &= ~(1 << PIN2_bp))
#define CH4_SET (portlevelA &= ~(1 << PIN3_bp))
#define CH5_SET (portlevelA &= ~(1 << PIN4_bp))
#define CH6_SET (portlevelA &= ~(1 << PIN5_bp))
#define CH7_SET (portlevelA &= ~(1 << PIN6_bp))
#define CH8_SET (portlevelA &= ~(1 << PIN7_bp))

#define CH9_SET (portlevelD &= ~(1 << PIN0_bp))
#define CH10_SET (portlevelD &= ~(1 << PIN1_bp))
#define CH11_SET (portlevelD &= ~(1 << PIN2_bp))
#define CH12_SET (portlevelD &= ~(1 << PIN3_bp))
#define CH13_SET (portlevelD &= ~(1 << PIN4_bp))
#define CH14_SET (portlevelD &= ~(1 << PIN5_bp))
#define CH15_SET (portlevelD &= ~(1 << PIN6_bp))
#define CH16_SET (portlevelD &= ~(1 << PIN7_bp))

#define CH17_SET (portlevelR &= ~(1 << PIN0_bp))
#define CH18_SET (portlevelR &= ~(1 << PIN1_bp))

#define PORTA_MASK 0xFF
#define PORTD_MASK 0xFF
#define PORTR_MASK 0x03

#endif /* CONFIG_H_ */