/*
 * main.c
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

#include "config.h"
#include <avr/io.h>
#include <string.h>

/* XSPI configuration structure */
xspi_config_t xspi_config = {
    .spi = &SPIC,
    .port = &PORTC,
    .mosi_pin = 7,
    .miso_pin = 6,
    .sck_pin = 5,
    .ss_pin = 4
};

/* XNRF24L01 configuration structure */
xnrf_config_t xnrf_config = {
    .spi = &SPIC,
    .spi_port = &PORTC,
    .ss_port = &PORTC,
    .ss_pin = 4,
    .ce_port = &PORTC,
    .ce_pin  = 3,
    .addr_width = 5,
    .payload_width = 32,
    .confbits = NRF_CBITS
};

/* XUSART configuration structure */
xusart_config_t xusart_config = {
    .usart = &USARTC0,
    .port = &PORTC,
    .tx_pin = 7
};

uint64_t            addr_p0 = ADDR_P0;      /* Default nRF address for TX and Pipe 0 RX */
uint64_t            addr_p1 = ADDR_P1;      /* Default nRF address for Pipe 1 RX */
volatile uint8_t    rxbuff[32];             /* Packet buffer */
volatile bool       DFLAG = false;          /* Data ready flag */
bool                WIRELESS = false;       /* Flag to tell if we're wireless or not */
uint8_t             compare[NUM_CHANNELS];  /* Channel data for PWM */
volatile uint8_t    compbuff[NUM_CHANNELS]; /* Buffer to feed PWM data */

/* Configure for RS485 / Renard mode */
void setup_rs485() {
    // Configure the USART module
    PORTC.REMAP |= PR_USART0_bm;                                        /* Remap USART to upper half of port */
    xusart_init(&xusart_config);                                        /* Initialize the XUSART driver */
    xusart_set_format(xusart_config.usart, USART_CHSIZE_8BIT_gc,        /* 8N1 on USARTC0 */
            USART_PMODE_DISABLED_gc, false);
    xusart_set_baudrate(xusart_config.usart, USART_BAUDRATE, F_CPU);    /* set baud rate */
    xusart_enable_tx(xusart_config.usart);                              /* Enable module TX */
    xusart_enable_rx(xusart_config.usart);                              /* Enable module RX */
    
    // Single pulse to let us know we're in RS485 mode    
    LED_STATUS_ON;
    _delay_ms(250);
    LED_STATUS_OFF;
}

/* Configure for nRF / RFPixelControl mode */
void setup_nrf() {
    // Configure the nRF radio
    xnrf_init(&xnrf_config, &xspi_config);                  /* Initialize the XNRF driver */
    xnrf_set_channel(&xnrf_config, NRF_CHANNEL);            /* Set our channel */
    xnrf_set_datarate(&xnrf_config, NRF_RATE);              /* Set our data rate */
    xnrf_write_register(&xnrf_config, EN_AA, 0);            /* Disable auto ack's */
    xnrf_write_register(&xnrf_config, SETUP_RETR, 0);       /* Disable auto retries */
    xnrf_write_register(&xnrf_config, EN_RXADDR, 3);        /* Listen on pipes 0 & 1 */
    xnrf_set_tx_address(&xnrf_config, (uint8_t*)&addr_p0);  /* Set TX address */
    xnrf_set_rx0_address(&xnrf_config, (uint8_t*)&addr_p0); /* Set Pipe 0 address */
    xnrf_set_rx1_address(&xnrf_config, (uint8_t*)&addr_p1); /* Set Pipe 1 address */

    // Setup nRF for listening
    xnrf_config_rx(&xnrf_config);   /* Configure nRF for RX mode */
    xnrf_powerup(&xnrf_config);     /* Power-up the nRF */
    _delay_ms(5);                   /* Let the radio stabilize - Section 6.1.7 - Tpd2stdby */

    // Setup pin change interrupt handling for the nRF on PC2
    PORTC_PIN2CTRL = PORT_ISC_FALLING_gc;   /* Setup PC2 to sense falling edge */
    PORTC.INTMASK = PIN2_bm;                /* Enable pin change interrupt for PC2 */
    PORTC.INTCTRL = PORT_INTLVL_LO_gc;      /* Set Port C for low level interrupts */

    // Double pulse to let us know we're in nRF mode
    LED_STATUS_ON;
    _delay_ms(250);
    LED_STATUS_OFF;
    _delay_ms(250);
    LED_STATUS_ON;
    _delay_ms(250);
    LED_STATUS_OFF;
    
    xnrf_enable(&xnrf_config);      /* start listening on nRF */
}

/* Initialize the board */
void init() {
    // Configure clock to 32MHz
    OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;    /* Enable the internal 32MHz & 32KHz oscillators */
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));         /* Wait for 32Khz oscillator to stabilize */
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));         /* Wait for 32MHz oscillator to stabilize */
    DFLLRC32M.CTRL = DFLL_ENABLE_bm ;               /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
    CCP = CCP_IOREG_gc;                             /* Disable register security for clock update */
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;                /* Switch to 32MHz clock */
    OSC.CTRL &= ~OSC_RC2MEN_bm;                     /* Disable 2Mhz oscillator */

     // Setup Status LEDs
    PORTC.DIRSET = PIN0_bm | PIN1_bm;
    LED_STATUS_OFF;
    LED_DATA_OFF;

    // Setup PWM outputs and initialize buffers
    PORTA.DIR = 0xFF;   /* Channels 1-8 */
    PORTD.DIR = 0xFF;   /* Channels 9-16 */
    PORTR.DIR = 0x03;   /* Channels 17 & 18 */
    for (uint8_t i=0; i<NUM_CHANNELS; i++) {
        compare[i] = 0;
        compbuff[i] = 0;
    }
    
    // Check if IRQ line is high.  If so, we have a nRF module inserted.
    if (PORTC.IN & PIN2_bm) {
        setup_nrf();
        WIRELESS = true;
    } else {
        setup_rs485();
    }    
        
 	// Setup timer for PWM
    TCC4.CTRLB |= TC45_WGMODE_NORMAL_gc;    /* Configure TCC4 for normal waveform generation */
    TCC4.PER = 256;                         /* Period value for TCC4 */
    TCC4.INTCTRLA = TC45_OVFINTLVL_MED_gc;  /* Setup TCC4 Overflow Interrupt - Medium level */
        
    // Enable interrupts and start PWM timer
    PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;    /* Enable low level interrupts */
    sei();                                              /* Enable global interrupt flag */
    TCC4.CTRLA = TC45_CLKSEL_DIV2_gc;                   /* Start the PWM timer */
}

/* Software PWM routine from AVR136 */
ISR(TCC4_OVF_vect) {
     static uint8_t portlevelA=PORTA_MASK, portlevelD=PORTD_MASK, portlevelR=PORTR_MASK;
     static uint8_t softcount=0xFF;
     
     // update outputs
     PORTA.OUT = portlevelA;
     PORTD.OUT = portlevelD;
     PORTR.OUT = portlevelR;
     // unrolled for speed -- process for NUM_CHANNELS
     if (++softcount == 0) {
         compare[0] = compbuff[0];
         compare[1] = compbuff[1];
         compare[2] = compbuff[2];
         compare[3] = compbuff[3];
         compare[4] = compbuff[4];
         compare[5] = compbuff[5];
         compare[6] = compbuff[6];
         compare[7] = compbuff[7];
         compare[8] = compbuff[8];
         compare[9] = compbuff[9];
         compare[10] = compbuff[10];
         compare[11] = compbuff[11];
         compare[12] = compbuff[12];
         compare[13] = compbuff[13];
         compare[14] = compbuff[14];
         compare[15] = compbuff[15];
         compare[16] = compbuff[16];
         compare[17] = compbuff[17];

         // set all pins high
         portlevelA |= PORTA_MASK;
         portlevelD |= PORTD_MASK;
         portlevelR |= PORTR_MASK;
     }
     
     if (compare[0] == softcount) CH1_SET;
     if (compare[1] == softcount) CH2_SET;
     if (compare[2] == softcount) CH3_SET;
     if (compare[3] == softcount) CH4_SET;
     if (compare[4] == softcount) CH5_SET;
     if (compare[5] == softcount) CH6_SET;
     if (compare[6] == softcount) CH7_SET;
     if (compare[7] == softcount) CH8_SET;
     if (compare[8] == softcount) CH9_SET;
     if (compare[9] == softcount) CH10_SET;
     if (compare[10] == softcount) CH11_SET;
     if (compare[11] == softcount) CH12_SET;
     if (compare[12] == softcount) CH13_SET;
     if (compare[13] == softcount) CH14_SET;
     if (compare[14] == softcount) CH15_SET;
     if (compare[15] == softcount) CH16_SET;
     if (compare[16] == softcount) CH17_SET;
     if (compare[17] == softcount) CH18_SET;
     
     TCC4.INTFLAGS |= TC4_OVFIF_bm;   
}

/* Interrupt handler for nRF hardware interrupt on PC2 */
ISR(PORTC_INT_vect) {
    xnrf_read_payload(&xnrf_config, rxbuff, xnrf_config.payload_width);     /* Retrieve the payload */
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));            /* Reset the RX_DR status */
    PORTC.INTFLAGS |= PIN2_bm;                                              /* Clear interrupt flag for PC3 */
    DFLAG = true;                                                           /* Set out data ready flag */
}

/* Process a Renard packet */
void procRenard() {
    uint8_t data;
    uint8_t channel = 0;
    
    /* Process for Renard data */
    while (channel < NUM_CHANNELS) {
        data = xusart_getchar(&USARTC0);
        switch (data) {
            case RENARD_PAD:    /* Ignore PAD bytes */
                break;
            case RENARD_ESCAPE: /* Process escaped bytes */
                data = xusart_getchar(&USARTC0);
                switch (data) {
                    case RENARD_ESC_7D:
                        compbuff[channel] = 0x7D;
                        break;
                    case RENARD_ESC_7E:
                        compbuff[channel] = 0x7E;
                        break;
                    case RENARD_ESC_7F:
                        compbuff[channel] = 0x7F;
                        break;
                }
                channel++;
                break;
            default:            /* Hopefully it's just good ol' channel data! */
                compbuff[channel] = data;
                channel++;
        }
    }
}

/* Loop for parsing Renard data */
void renard_loop() {
    uint8_t data;
    
    while(1) {
        /* Poll and retransmit while waiting for the sync byte */
        while ((data = xusart_getchar(&USARTC0)) != RENARD_SYNC)
            xusart_putchar(&USARTC0, data);
        
        LED_DATA_ON;
        
        /* Retransmit the sync byte */
        xusart_putchar(&USARTC0, RENARD_SYNC);
		
        /* Evaluate command byte */
        data = xusart_getchar(&USARTC0);
        if (data == RENARD_ADDR) {			/* Process a Renard packet for this device */
            procRenard();
        } else if (data > RENARD_ADDR) {	/* It's for a downstream device */
            data--;                         /* Decrement the address */
            xusart_putchar(&USARTC0, data); /* And forward it */
        } else {							/* Unsupported */
            xusart_putchar(&USARTC0, data); /* So Forward it */
        }
        
        LED_DATA_OFF;
    }        
}

/* Loop for parsing RFPixelControl data */
void rfp_loop() {
    while(1) {
        while(!DFLAG);  /* Spin our wheels until we have a packet */
        DFLAG = false;  /* and reset out flag */

        LED_DATA_ON;
        
        //TODO: Just copying first 18 channels from packet for now.  Will set for config soon. */
        if(rxbuff[30] == 0)   /* Offset for packet, this is the one we want for now */
            for(uint8_t i=0; i < NUM_CHANNELS; i++)
                compbuff[i] = rxbuff[i];

        LED_DATA_OFF;
    }
}

/* Main Loop */
int main(void) {
    /* Let the power supply stabilize */
    _delay_ms(100);
    
    /* Initialize everything */
    init();
    
    /* Go! */
    if(WIRELESS)
        rfp_loop();
    else
        renard_loop();
}