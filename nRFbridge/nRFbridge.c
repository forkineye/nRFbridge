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
#include <string.h>
#include "XUSART/XUSART.h"
#include "XNRF24L01/XNRF24L01.h"
#include "renard.h"

#define USART_TXMODE    PORTD.OUTSET = PIN1_bm
#define USART_RXMODE    PORTD.OUTCLR = PIN1_bm
#define STATUS_LED_ON   PORTA.OUTCLR = PIN0_bm
#define STATUS_LED_OFF  PORTA.OUTSET = PIN0_bm

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
    .confbits = NRF_CBITS
};

uint64_t    addr_p0 = ADDR_P0;  /* default nRF address for TX and Pipe 0 RX */
uint64_t    addr_p1 = ADDR_P1;  /* default nRF address for Pipe 1 RX */    
RingBuff_t  ringbuff;           /* Ring buffer to hold our data */
volatile uint8_t rxbuff[32];    /* Packet buffer */
volatile bool DFLAG = false;

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
    RingBuffer_InitBuffer(&ringbuff);
        
    // Configure IO
    PORTA.DIRSET = PIN0_bm;             /* Status LED */
    PORTD.DIRSET = PIN1_bm | PIN3_bm;   /* USART Clock TX line (3) and direction control (1) */
    STATUS_LED_OFF;
    
    // Configure the nRF radio
    xnrf_init(&xnrf_config);                                /* initialize the XNRF driver */
    xnrf_set_channel(&xnrf_config, NRF_CHANNEL);            /* set our channel */
    xnrf_set_datarate(&xnrf_config, NRF_RATE);              /* set our data rate */
    xnrf_write_register(&xnrf_config, EN_AA, 0);            /* disable auto ack's */
    xnrf_write_register(&xnrf_config, SETUP_RETR, 0);       /* disable auto retries */
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
    _delay_ms(5);                   /* Let the radio stabilize - Section 6.1.7 - Tpd2stdby */
    
    // Enable interrupts and start listening
    PMIC.CTRL |= PMIC_LOLVLEN_bm;   /* Enable low interrupts */
    sei();                          /* Enable global interrupt flag */
    USART_RXMODE;                   /* USART Listen mode */
    xnrf_enable(&xnrf_config);      /* start listening on nRF */
}

/* Interrupt handler for nRF hardware interrupt on PC3 */
ISR(PORTC_INT_vect) {
    //TODO: Ring buffer is having some performance issues.
    //xnrf_read_payload_buffer(&xnrf_config, &ringbuff, xnrf_config.payload_width); /* retrieve the payload */
    xnrf_read_payload(&xnrf_config, rxbuff, xnrf_config.payload_width);     /* retrieve the payload */
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));            /* reset the RX_DR status */    
    
    // Keep coming back until FIFO is empty. -- no until ring buffer is replaced.
    //if((xnrf_read_register(&xnrf_config, FIFO_STATUS)) & (1 << RX_EMPTY))
        PORTC.INTFLAGS = PIN3_bm;                                               /* Clear interrupt flag for PC3 */
    DFLAG = true;
 }

/* Interrupt handler for USART RX on Port D - Every 1100 clock cycles @ 115,200? Polling instead? */
ISR(USARTD0_RXC_vect) {
    RingBuffer_Insert(&ringbuff, xusart_getchar(&USARTD0));   /* get the byte */
}

/* loop for one way nrf->rs485 bridge */
void nrf_to_rs485_loop() {
    USART_TXMODE;   /* Enable USART TX */

    while(1) {
        while(!ringbuff.Count);                   /* Spin our wheels until we have data */
        xusart_send_buffer(&USARTD0, &ringbuff);  /* Spit out the buffer */
        PORTA.OUTTGL = PIN0_bm;                 /* Toggle status LED */
    }    
}


/* loop for one way rs485->nrf bridge */
//TODO: Currently broken...
void rs485_to_nrf_loop() {
    USART_RXMODE;   /* Enable USART RX */

    xnrf_powerup_tx(&xnrf_config);  /* Configure the radio for TX mode */
    //_delay_ms(5);                   /* Let the radio stabilize - Section 6.1.7 - Tpd2stdby -- but we're already powered up so not needed */
    //xnrf_enable(&xnrf_config);      /* Enable and stay in Standby-II mode, auto-sending as data hits the TX FIFO */
    xnrf_flush_tx(&xnrf_config);
        
    while(1) {
        // This needs a timeout check or some way to handle packets < 32 bytes.  Timer facilty that resets from RX ISR?
        //^---- calculate timeout based on bitrate of a continuous stream of data + some padding.  if we timeout, terminate
        //      the packet and fire off the payload.  Use variable length payloads or add a header to the data?
        // Actually, this doesn't work at all right now. Big design flaw.. really didn't think this one through :)
        while(ringbuff.Count < 32);                               /* Spin our wheels until we have a full packet to forward */
        xnrf_write_payload_buffer(&xnrf_config, &ringbuff, 32);   /* Send the packet off */
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

/* Loop for receiving Renard RS485 data and sending as RFPixelControl packets */
void renard_to_rfp_loop() {
    uint8_t data;           /* Byte we're processing */
    uint8_t packet[32];     /* Packet buffer */
    uint8_t index = 0;      /* Current index of the packet buffer */
    uint8_t offset = 0;     /* Offset multiplier */
    renstate_t state = RENSTATE_NULL;
    
    USART_RXMODE;                   /* Enable USART RX */
    xnrf_powerup_tx(&xnrf_config);  /* Configure the radio for TX mode */
    xnrf_flush_tx(&xnrf_config);    /* Clean the TX pipe for good measure */

    while(1) {
        /* We have a full load? Send it off! */
        if (index == 29) {
            packet[index++] = offset++;                     /* Byte 31 defines offset */
            packet[index] = 0x00;                           /* Byte 32 is reserved */
            xnrf_write_payload(&xnrf_config, packet, 32);   /* Load the packet */
            xnrf_pulse_tx(&xnrf_config);                    /* Send it off */
            index = 0;                                      /* Reset our index */
        }
        
        while(!ringbuff.Count);               /* Spin our wheels until the buffer has data */
        STATUS_LED_ON;
        data = RingBuffer_Remove(&ringbuff);  /* Grab a byte off the buffer */

        /* Process special characters and set states if needed */
        switch(data) {
            case RENARD_PAD:                /* Ignore PAD bytes and wait for next byte */
                continue;
            case RENARD_SYNC:               /* Set our state to SYNC and process current packet if needed */
                state = RENSTATE_SYNC;
                if(index) {
                    memset(&packet[index], 0x00, 29 - index);   /* Null out rest of packet. PADs will get translated or would use them */
                    index = 29;                                 /* Set our index to trigger a TX */
                }                    
                continue;
            case RENARD_ESCAPE:             /* Set our state to ESCAPE and wait for next byte */
                state = RENSTATE_ESCAPE;
                continue;
            default:;
        }

        /* Decisions, decisions... */
        switch (state) {
            case RENSTATE_ESCAPE:           /* We're in ESCAPE state.  Process characters per Renard protocol and set state to NULL. */
                switch (data) {
                    case RENARD_ESC_7D:
                        data = 0x7D;
                        break;
                    case RENARD_ESC_7E:
                        data = 0x7E;
                        break;
                    case RENARD_ESC_7F:
                        data = 0x7F;
                        break;
                }
                state = RENSTATE_NULL;
                break;
            case RENSTATE_SYNC:             /* We're in SYNC state.  The byte being processed will be the address / command byte. */
                state = RENSTATE_NULL;      /* Normally, we'd look at this to see if the packet is for us. Ignored for our purposes. */
                offset = 0;                 /* Reset the offset multiplier */
                continue;
            default:;
        }            
            
        /* What's left is channel data, get to work! */
        packet[index++] = data;

        STATUS_LED_OFF;
    }
}

/* Loop for receiving RFPixelControl packets and sending as Renard RS485 data */
void rfp_to_renard_loop() {
    USART_TXMODE;       /* Enable USART TX */
    
    while(1) {
        while(!DFLAG);      /* Spin our wheels until we have a full packet */
        DFLAG = false;
        STATUS_LED_ON;

        if(rxbuff[30] == 0) {                         /* Check the offset byte to reset the Renard stream */
            xusart_putchar(&USARTD0, RENARD_SYNC);      /* Send the Renard SYNC byte */
            xusart_putchar(&USARTD0, RENARD_ADDR);      /* and the Command / Address byte */
        }
        
        /* Process and send channel data. Escape Renard special characters. */
        for (uint8_t i = 0; i < 30; i++) {
            switch (rxbuff[i]) {
                case RENARD_PAD:
                    xusart_putchar(&USARTD0, RENARD_ESCAPE);
                    xusart_putchar(&USARTD0, RENARD_ESC_7D);
                    break;
                case RENARD_SYNC:
                    xusart_putchar(&USARTD0, RENARD_ESCAPE);
                    xusart_putchar(&USARTD0, RENARD_ESC_7E);
                    break;
                case RENARD_ESCAPE:
                    xusart_putchar(&USARTD0, RENARD_ESCAPE);
                    xusart_putchar(&USARTD0, RENARD_ESC_7F);
                    break;
                default:
                    xusart_putchar(&USARTD0, rxbuff[i]);
            }                    
        }
        STATUS_LED_OFF;            
    }        
}    

void test_tx_loop() {
    uint8_t	alphatest[32] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50,
                             0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                             0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x5F};
    
    xnrf_powerup_tx(&xnrf_config);  /* Configure the radio for TX mode */

    while(1) {
        xnrf_write_payload(&xnrf_config, alphatest, 32);    /* Load the packet */
        xnrf_pulse_tx(&xnrf_config);                        /* Send it off */
        
        PORTA.OUTTGL = PIN0_bm;                             /* Toggle status LED */
        _delay_ms(1000);
    }
}

/* Test loop to send RFP packets */
void rfp_test_loop() {
    uint8_t packet[32];     /* Packet buffer */
    uint8_t index = 0;      /* Current index of the packet buffer */
        
    xnrf_powerup_tx(&xnrf_config);  /* Configure the radio for TX mode */
    xnrf_flush_tx(&xnrf_config);    /* Clean the TX pipe for good measure */

    while(1) {
        packet[index]++;
        xnrf_write_payload(&xnrf_config, packet, 32);   /* Load the packet */
        xnrf_pulse_tx(&xnrf_config);                    /* Send it off */
        if(packet[index] == 255)
            index++;
        PORTA.OUTTGL = PIN0_bm;
    }
}


int main(void) {
    init();
    
    //TODO: Add logic to determine bridge direction.  For now, we're going one direction: nRF->RS485.
    //nrf_to_rs485_loop();
    //rs485_to_nrf_loop();
    //bidirectional_loop();
    //renard_to_rfp_loop();
    rfp_to_renard_loop();
    //test_tx_loop();
    //rfp_test_loop();
}