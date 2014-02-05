/*
 * nRFbridge.c
 *
 * Project: nRFbridge
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
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "XUSART/XUSART.h"
#include "XNRF24L01/XNRF24L01.h"

#define USART_TXMODE PORTD.OUTSET = PIN1_bm
#define USART_RXMODE PORTD.OUTCLR = PIN1_bm

/* NRF24L01 configuration structure */
xnrf_config_t xnrf_config = {
    .spi = &SPIC,
    .spi_port = &PORTC,
    .ss_port = &PORTC,
    .ss_pin = 4,
    .ce_port = &PORTC,
    .ce_pin  = 2,
    .addr_width = 5,
    .payload_width = 32,
    .confbits = 0b00111100    // RX interrupt enabled
    //.confbits = 0b00001100  // All interrupts enabled
    //.confbits = ((1 << EN_CRC) | (1 << CRCO)) //TODO: move confbits elsewhere for runtime config changes
};

uint64_t addr_p0 = ADDR_P0;         /* default nRF address for TX and Pipe 0 RX */
uint64_t addr_p1 = ADDR_P1;         /* default nRF address for Pipe 1 RX */    
RingBuff_t  rxbuff;                 /* Ring buffer to hold our data */
volatile bool DFLAG = false;        /* nRF Data ready flag */
volatile uint8_t usart_counter = 0; /* counter to keep track of buffer updates in USART RX ISR */

/* Initialize the board */
//TODO: Add DMA support
void init() {
    // Configure clock to 32MHz
    OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;    /* Enable the internal 32MHz & 32KHz oscillators */
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));         /* Wait for 32Khz oscillator to stabilize */
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));         /* Wait for 32MHz oscillator to stabilize */
    DFLLRC32M.CTRL = DFLL_ENABLE_bm ;               /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
    CCP = CCP_IOREG_gc;                             /* Disable register security for clock update */
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;                /* Switch to 32MHz clock */
    OSC.CTRL &= ~OSC_RC2MEN_bm;                     /* Disable 2Mhz oscillator */

    // Initialize ring buffer
    RingBuffer_InitBuffer(&rxbuff);
        
    // Configure IO
    PORTA.DIRSET = PIN0_bm;             /* Status LED */
    PORTD.DIRSET = PIN1_bm | PIN3_bm;   /* USART Clock TX line (3) and direction control (1) */
    
    // Configure the nRF
    xnrf_init(&xnrf_config);                                /* initialize the XNRF driver */
    xnrf_set_channel(&xnrf_config, NRF_CHANNEL);            /* set our channel */
    xnrf_set_datarate(&xnrf_config, NRF_RATE);              /* set our data rate */
    xnrf_write_register(&xnrf_config, EN_AA, 0);            /* disable auto ack's */
    xnrf_write_register(&xnrf_config, EN_RXADDR, 3);        /* listen on pipes 0 & 1 */
    xnrf_set_tx_address(&xnrf_config, (uint8_t*)&addr_p0);  /* set TX address */
    xnrf_set_rx0_address(&xnrf_config, (uint8_t*)&addr_p0); /* set Pipe 0 address */
    xnrf_set_rx1_address(&xnrf_config, (uint8_t*)&addr_p1); /* set Pipe 1 address */

    // Clear nRF status and FIFOs in case we're coming out of a soft reset
    xnrf_write_register(&xnrf_config, NRF_STATUS, ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT)));
    xnrf_flush_rx(&xnrf_config);
    xnrf_flush_tx(&xnrf_config);

    // Configure the USART module    
    xusart_set_format(&USARTD0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);  /* 8N1 on USARTD0 */
    xusart_set_baudrate(&USARTD0, USART_BAUDRATE, F_CPU);                               /* set baud rate */
    xusart_enable_tx(&USARTD0);                                                         /* Enable module TX */
    xusart_enable_rx(&USARTD0);                                                         /* Enable module RX */
    
    // Setup pin change interrupt handling for the nRF on PC3
    PORTC_PIN3CTRL = PORT_ISC_FALLING_gc;   /* Setup PC3 to sense falling edge */
    PORTC.INTMASK = PIN3_bm;                /* Enable pin change interrupt for PC3 */
    PORTC.INTCTRL = PORT_INTLVL_LO_gc;      /* Set Port C for low level interrupts */

    // Setup USART RX interrupt handling    
    USARTD0.CTRLA = ((USARTD0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc);
    
    // Initialize listening on both RS485 line and nRF.  First in determines bridge direction
    xnrf_powerup_rx(&xnrf_config);  /* Power-up nRF in RX mode */
    _delay_ms(5);                   /* Let the radio stabilize */
    
    // Enable interrupts and start listening
    PMIC.CTRL |= PMIC_LOLVLEN_bm;   /* Enable low interrupts */
    sei();                          /* Enable global interrupt flag */
    USART_RXMODE;                   /* USART Listen mode */
    xnrf_enable(&xnrf_config);      /* start listening on nRF */
}

/* Interrupt handler for nRF hardware interrupt on PC3 */
ISR(PORTC_INT_vect) {
    xnrf_read_payload(&xnrf_config, &rxbuff, xnrf_config.payload_width);    /* retrieve the payload */
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));            /* reset the RX_DR status */
    
    // Keep coming back until FIFO is empty.
    if((xnrf_read_register(&xnrf_config, FIFO_STATUS)) & (1 << RX_EMPTY))
        PORTC.INTFLAGS = PIN3_bm;   /* Clear interrupt flag for PC3 */
    DFLAG = true;                   /* set state flag */
}

/* Interrupt handler for USART RX on Port D - Every 1100 clock cycles @ 115,200? Polling instead? */
ISR(USARTD0_RXC_vect) {
    RingBuffer_Insert(&rxbuff, xusart_getchar(&USARTD0));   /* get the byte */
    usart_counter++;
}

/* loop for one way nrf->rs485 bridge */
void nrf_to_rs485_loop() {
    USART_TXMODE;       /* enable USART TX */
    while(1) {
        while(!DFLAG);  /* spin our wheels until we have data */
        xusart_send_buffer(&USARTD0, &rxbuff);  /* spit out the buffer */
        //xusart_putchar(&USARTD0, 0x0D);         /* CR */
        //xusart_putchar(&USARTD0, 0x0A);         /* LF */
        PORTA.OUTTGL = PIN0_bm;                 /* Toggle status LED */
        DFLAG = false;                          /* clear our data ready flag */
    }    
}

/* loop for one way rs485->nrf bridge */
void rs485_to_nrf_loop() {
    USART_RXMODE;                   /* Enable USART RX */
    xnrf_powerup_tx(&xnrf_config);  /* Power up transmitter */
    _delay_ms(5);                   /* Give the radio time to stabilize */
    //xnrf_enable(&xnrf_config);      /* Enable and stay in Standby-II mode, auto-sending as data hits the TX FIFO */
        
    while(1) {
        //TODO: This needs a timeout check or some way to handle packets < 32 bytes.  Timer facilty that resets from RX ISR?
        //TODO:  Actually, this doesn't work at all right now. Big design flaw.. really didn't think this one through :)
        while(usart_counter < 32);                      /* Spin our wheels until we have a full packet to forward */
        xnrf_write_payload(&xnrf_config, &rxbuff, 32);  /* Send the packet off */
        xnrf_enable(&xnrf_config);                      /* Pulse the nRF to start TX */
        _delay_us(150);                                 /* -for 130us per datahseet */
        xnrf_disable(&xnrf_config);                     /* End pulse */
        usart_counter = 0;                              /* Reset our counter */
        PORTA.OUTTGL = PIN0_bm;                         /* Toggle status LED */
    }
}

/* loop for bidirectional bridge */
void bidrectional_loop() {
    while(1) {

    }
}

int main(void) {
    init();
    
    //TODO: Add logic to determine bridge direction.  For now, we're going one direction: nRF->RS485.
    nrf_to_rs485_loop();
    //rs485_to_nrf_loop();
    //bidirectional_loop();
    
}