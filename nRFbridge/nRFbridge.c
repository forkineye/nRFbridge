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
#include "renard.h"

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
    _delay_us(150);                 /* Let the radio stabilize - 130us per datasheet */
    
    // Enable interrupts and start listening
    PMIC.CTRL |= PMIC_LOLVLEN_bm;   /* Enable low interrupts */
    sei();                          /* Enable global interrupt flag */
    USART_RXMODE;                   /* USART Listen mode */
    xnrf_enable(&xnrf_config);      /* start listening on nRF */
}

/* Interrupt handler for nRF hardware interrupt on PC3 */
ISR(PORTC_INT_vect) {
    xnrf_read_payload_buffer(&xnrf_config, &rxbuff,                 /* retrieve the payload */
            xnrf_get_rx_width(&xnrf_config));
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));    /* reset the RX_DR status */
    
    // Keep coming back until FIFO is empty.
    if((xnrf_read_register(&xnrf_config, FIFO_STATUS)) & (1 << RX_EMPTY))
        PORTC.INTFLAGS = PIN3_bm;   /* Clear interrupt flag for PC3 */
    DFLAG = true;            
 }

/* Interrupt handler for USART RX on Port D - Every 1100 clock cycles @ 115,200? Polling instead? */
ISR(USARTD0_RXC_vect) {
    RingBuffer_Insert(&rxbuff, xusart_getchar(&USARTD0));   /* get the byte */
    usart_counter++;
}

/* loop for one way nrf->rs485 bridge */
void nrf_to_rs485_loop() {
    USART_TXMODE;                                               /* Enable USART TX */
    xnrf_write_register(&xnrf_config, FEATURE, (1 << EN_DPL));  /* Enable dynamic payloads */
    xnrf_write_register(&xnrf_config, DYNPD, (1 << DPL_P0));    /* on Pipes 0 only for now */
    
    while(1) {
        while(!DFLAG);                          /* Spin our wheels until we have data */
        DFLAG = false;                          /* Clear our data ready flag */
        xusart_send_buffer(&USARTD0, &rxbuff);  /* Spit out the buffer */
        PORTA.OUTTGL = PIN0_bm;                 /* Toggle status LED */
    }    
}

/* loop for one way rs485->nrf bridge */
void rs485_to_nrf_loop() {
    USART_RXMODE;                                               /* Enable USART RX */
    xnrf_write_register(&xnrf_config, FEATURE, (1 << EN_DPL));  /* Enable dynamic payloads */
    xnrf_write_register(&xnrf_config, DYNPD, (1 << DPL_P0));    /* on Pipe 0 -- required to TX */

    xnrf_powerup_tx(&xnrf_config);  /* Power up transmitter */
    _delay_us(150);                 /* Give the radio time to stabilize - 130us per datasheet */
    //xnrf_enable(&xnrf_config);      /* Enable and stay in Standby-II mode, auto-sending as data hits the TX FIFO */
    xnrf_flush_tx(&xnrf_config);
        
    while(1) {
        //TODO: This needs a timeout check or some way to handle packets < 32 bytes.  Timer facilty that resets from RX ISR?
        //^---- calculate timeout based on bitrate of a continuous stream of data + some padding.  if we timeout, terminate
        //      the packet and fire off the payload.  Use variable length payloads or add a header to the data?
        //TODO:  Actually, this doesn't work at all right now. Big design flaw.. really didn't think this one through :)
        while(usart_counter < 32);                              /* Spin our wheels until we have a full packet to forward */
        usart_counter = 0;                                      /* Reset our counter */
        xnrf_write_payload_buffer(&xnrf_config, &rxbuff, 32);   /* Send the packet off */
        xnrf_enable(&xnrf_config);                              /* Pulse the nRF to start TX */
        _delay_us(15);                                          /* -for 10us per datahseet */
        xnrf_disable(&xnrf_config);                             /* End pulse */
        PORTA.OUTTGL = PIN0_bm;                                 /* Toggle status LED */
    }
}

/* loop for bidirectional bridge - not binary safe due to flow control. implement XMODEM or something? */
//TODO: Second thoughts on this and not even started.  How will we handle flow control? XON/OFF?  Should note for terminal / config usage only.
void bidrectional_loop() {
    while(1) {

    }
}

void test_tx_loop() {
    uint8_t	alphatest[32] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50,
                             0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                             0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x5F};
    
    uint8_t numtest[12] = {0x30, 0x31, 0x32, 0x33, 0x34, 
                           0x35, 0x36, 0x37, 0x38, 0x39,
                           0x0A, 0x0D};

    USART_RXMODE;                                               /* Enable USART RX */
    xnrf_write_register(&xnrf_config, FEATURE, (1 << EN_DPL));  /* Enable dynamic payloads */
    xnrf_write_register(&xnrf_config, DYNPD, (1 << DPL_P0));    /* on Pipe 0 -- required to TX */

    xnrf_powerup_tx(&xnrf_config);  /* Power up transmitter */
    _delay_us(150);                 /* Give the radio time to stabilize - 130us per datasheet */
    //xnrf_enable(&xnrf_config);      /* Enable and stay in Standby-II mode, auto-sending as data hits the TX FIFO */
    
    while(1) {
        xnrf_write_payload(&xnrf_config, alphatest, 30);        /* Send the packet off */
        xnrf_enable(&xnrf_config);                              /* Pulse the nRF to start TX */
        _delay_us(15);                                          /* -for 10us per datasheet */
        xnrf_disable(&xnrf_config);                             /* End pulse */

        _delay_ms(100);
        xnrf_flush_tx(&xnrf_config);
        
        xnrf_write_payload(&xnrf_config, numtest, 12);          /* Send the packet off */
        xnrf_enable(&xnrf_config);                              /* Pulse the nRF to start TX */
        _delay_us(15);                                          /* -for 10us per datasheet */
        xnrf_disable(&xnrf_config);                             /* End pulse */

        PORTA.OUTTGL = PIN0_bm;                                 /* Toggle status LED */
        _delay_ms(100);
    }
}

int main(void) {
    init();
    
    //TODO: Add logic to determine bridge direction.  For now, we're going one direction: nRF->RS485.
    //nrf_to_rs485_loop();
    //rs485_to_nrf_loop();
    //bidirectional_loop();
     test_tx_loop();    
}