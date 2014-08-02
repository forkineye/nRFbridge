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
#define STATUS_LED_TGL  PORTA.OUTTGL = PIN0_bm

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
    .ce_pin  = 2,
    .addr_width = 5,
    .payload_width = 32,
    .confbits = NRF_CBITS
};

/* XUSART configuration structure */
xusart_config_t xusart_config = {
    .usart = &USARTD0,
    .port = &PORTD,
    .tx_pin = 3
};

uint64_t addr_p0 = ADDR_P0;         /* default nRF address for TX and Pipe 0 RX */
uint64_t addr_p1 = ADDR_P1;         /* default nRF address for Pipe 1 RX */    
RingBuffer_t rbSerial;              /* Ring Buffer to hold incoming RS485 data */
uint8_t rbSerialData[BUFFER_SIZE];  /* Internal buffer for rbSerial */
volatile uint8_t rxbuff[32];        /* RX packet buffer */
uint8_t txbuff[32];                 /* TX packet buffer */
volatile uint8_t rxoffset = 0;      /* RX offset tracker for packet building */
volatile uint8_t txoffset = 0;      /* TX offset tracker for packet building */
uint8_t frame = 0;                  /* Frame tracker for packet building */
uint8_t command = 0;                /* Command byte */
volatile bool NRF_FLAG = false;     /* Flag for incoming NRF data */
volatile bool SERIAL_FLAG = false;  /* Flag for incoming serial data */
volatile bool DR_NRF = false;       /* Flag for NRF data ready */
volatile bool DR_SERIAL = false;    /* Flag for serial TX data ready */

bridge_mode_t mode = MODE_CONFIG;   /* Bridge mode */


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

    // Initialize ring buffer
    RingBuffer_InitBuffer(&rbSerial, rbSerialData, sizeof(rbSerialData));
        
    // Configure IO
    PORTA.DIRSET = PIN0_bm;     /* Status LED */
    PORTD.DIRSET = PIN1_bm;     /* USART direction control */
    STATUS_LED_OFF;
    
    //TODO: Load configuration from EEPROM here. For now, use defaults from config.h
    uint8_t         nrf_channel = NRF_CHANNEL;
    uint8_t         nrf_rate = NRF_RATE;
    uint32_t        rs485_baudrate = RS485_BAUDRATE;

    //TODO: Add configuration jumper check here to override mode
    mode = MODE_RENARD;

    //TODO: Configure DMA controller based on mode
    // rf->serial: read FRAME_SIZE to rxbuff, then trigger DMA serial to transmit rxbuff -- double buffer. issues with DMX due to timing (may need whole universe)??
    // serial->rf: DMA to rxbuff until FRAME_SIZE or timeout, then 
    
    //TODO:  Add support for different RF protocols and switch on config like USART below
    // Configure the nRF radio
    xnrf_init(&xnrf_config, &xspi_config);                  /* Initialize the XNRF driver */
    xnrf_set_channel(&xnrf_config, nrf_channel);            /* Set our channel */
    xnrf_set_datarate(&xnrf_config, nrf_rate);              /* Set our data rate */
    xnrf_write_register(&xnrf_config, EN_AA, 0);            /* Disable auto ack's */
    xnrf_write_register(&xnrf_config, SETUP_RETR, 0);       /* Disable auto retries */
    xnrf_write_register(&xnrf_config, EN_RXADDR, 3);        /* Listen on pipes 0 & 1 */
    xnrf_set_tx_address(&xnrf_config, (uint8_t*)&addr_p0);  /* Set TX address */
    xnrf_set_rx0_address(&xnrf_config, (uint8_t*)&addr_p0); /* Set Pipe 0 address */
    xnrf_set_rx1_address(&xnrf_config, (uint8_t*)&addr_p1); /* Set Pipe 1 address */
    
    // Configure the USART module based on mode
    xusart_init(&xusart_config);                                                /* Initialize the XUSART driver */
    switch(mode) {
        case MODE_CONFIG:
            xusart_set_format(xusart_config.usart, USART_CHSIZE_8BIT_gc,
                    USART_PMODE_DISABLED_gc, false);                            /* 8N1 on USARTD0 */
            xusart_set_baudrate(xusart_config.usart, CONFIG_BAUDRATE, F_CPU);   /* set config baud rate */
            break;
        case MODE_RS485:
        case MODE_RENARD:
            xusart_set_format(xusart_config.usart, USART_CHSIZE_8BIT_gc,
                    USART_PMODE_DISABLED_gc, false);                            /* 8N1 on USARTD0 */
            xusart_set_baudrate(xusart_config.usart, rs485_baudrate, F_CPU);    /* set RS485 / Renard baud rate */
            break;
        case MODE_DMX:
            xusart_set_format(xusart_config.usart, USART_CHSIZE_8BIT_gc,
                    USART_PMODE_DISABLED_gc, true);                             /* 8N2 on USARTD0 */
            xusart_set_baudrate(xusart_config.usart, DMX_BAUDRATE, F_CPU);      /* set DMX baud rate */
            break;
    }
    xusart_enable_tx(xusart_config.usart);                                      /* Enable USART TX */
    xusart_enable_rx(xusart_config.usart);                                      /* Enable USART RX */

// hack til timer is implemented    
#define timer_expired   0   
    // Setup TCD5 as serial RX watchdog.  We'll use this to trigger nRF transmissions on partial frames.
//     PORTD.DIRSET = PIN4_bm | PIN5_bm;                                   /* Enable output on PD4 & PD5 for compare channels */
//     TCD5.CTRLB = TC45_WGMODE_SINGLESLOPE_gc;                            /* Single Slope PWM */
//     TCD5.CTRLD = TC45_EVACT_RESTART_gc | TC45_EVSEL_CH7_gc;             /* Restart on CH7 pulse - rising clock edge */
//     TCD5.CTRLE = TC45_CCAMODE_COMP_gc | TC45_CCBMODE_COMP_gc;           /* Enable output compare on CCA & CCB */
//     TCD5.PER = 40 - 1;                                                  /* At 32MHz, 1 cycle = 31.25ns.  Define top of counter for a 1250ns pulse: (32MHz / 800KHz) */
//     TCD5.CCA = 8;                                                       /* Compare for 0 bit @ 250ns (31.25ns * 8). Output is on PD4 */
//     TCD5.CCB = 32;                                                      /* Compare for 1 bit @ 1000ns (31.25ns * 32). Output is on PD5 */
//     TCD5.CTRLA = TC45_CLKSEL_DIV1_gc | TC5_EVSTART_bm | TC5_UPSTOP_bm;  /* Start and stop the timer on each event occurrence, full speed clock */

    // Setup interrupts for USART and nRF
    xusart_config.usart->CTRLA =
            ((xusart_config.usart->CTRLA & ~USART_RXCINTLVL_gm)
            | USART_RXCINTLVL_LO_gc);       /* Setup USART RX interrupt handling */
    PORTC.INTCTRL = PORT_INTLVL_LO_gc;      /* Set Port C for low level interrupts */
    PORTC_PIN3CTRL = PORT_ISC_FALLING_gc;   /* Setup PC3 to sense falling edge */
    PORTC.INTMASK = PIN3_bm;                /* Enable pin change interrupt for PC3 */
    PMIC.CTRL |= PMIC_LOLVLEN_bm;           /* Enable low interrupts */
    
    // Initialize listening on nRF.
    xnrf_config_rx(&xnrf_config);   /* Configure nRF for RX mode */
    xnrf_powerup(&xnrf_config);     /* Power-up the nRF */
    _delay_ms(5);                   /* Let the radio stabilize - Section 6.1.7 - Tpd2stdby */
    xnrf_flush_tx(&xnrf_config);    /* Clean the TX/RX pipes for good measure */
    xnrf_flush_rx(&xnrf_config);
}

/* Interrupt handler for nRF hardware interrupt on PC3 */
ISR(PORTC_INT_vect) {
    xnrf_read_payload(&xnrf_config, rxbuff, xnrf_config.payload_width);     /* Retrieve the payload */
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));            /* Reset the RX_DR status */
    PORTC.INTFLAGS = PIN3_bm;                                               /* Clear interrupt flag for PC3 */
    NRF_FLAG = true;                                                        /* Set out data ready flag */
 }

/* Interrupt handler for USART RX on Port D - Every 1100 clock cycles @ 115,200? Polling instead? */
ISR(USARTD0_RXC_vect) {
    RingBuffer_Insert(&rbSerial, xusart_getchar(xusart_config.usart));      /* get the byte */
    SERIAL_FLAG = true;
}

/*************************************************/
/* Input Parsers                                 */
/*************************************************/

/* Generic RS485 input parser */
void parse_rs485(uint16_t count) {
    //TODO: This would be more efficient just passing the ringbuffer to a NRF routine if doing a transparent bridge.
    // for now, we're using RFSC format for NRF output and will pass to a generic TX buffer that all other parsers use
    while((count-- > 0) && (txoffset < FRAME_SIZE))    /* Build TX packet from ring buffer */
        txbuff[txoffset++] = RingBuffer_Remove(&rbSerial);
    
    /* Check for full frame or serial RX timeout */
    if((txoffset == FRAME_SIZE) || (timer_expired)) {
        frame = txoffset;   /* For RS485 passthru we don't track frames.  Use frame byte to store size of truncated frames */
        DR_NRF = true;      /* NRF Data Ready flag */
    }        
}

/* Renard input parser */
void parse_renard(uint16_t count) {
    uint8_t data;                               /* Byte we're processing */
    static renstate_t state = RENSTATE_NULL;    /* Renard state tracker */

    // Build TX packet from ring buffer
    while((count-- > 0) && (txoffset < FRAME_SIZE)) { 
        data = RingBuffer_Remove(&rbSerial);    /* Grab a byte from the ring buffer */

        switch(data) {                          /* Process special characters and set states if needed */
            case RENARD_PAD:                    /* Ignore PAD bytes and wait for next byte */
                continue;
            case RENARD_SYNC:                   /* Set our state to SYNC and process current packet if needed */
                state = RENSTATE_SYNC;
                if(txoffset) {
                    memset(&txbuff[txoffset], 0x00,
                            FRAME_SIZE - txoffset);    /* Null out rest of packet. PADs will get translated or would use them */
                    txoffset = FRAME_SIZE;             /* Set our index to trigger a TX */
                }
                continue;
            case RENARD_ESCAPE:                 /* Set our state to ESCAPE and wait for next byte */
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
                frame = 0;                  /* Reset the offset multiplier */
                continue;
            default:;
        }
        
        /* What's left is channel data, get to work! */
        txbuff[txoffset++] = data;
    }    
    
    /* Check serial RX timeout and finish frame if needed */
    if (timer_expired) {
        memset(&txbuff[txoffset], 0x00, FRAME_SIZE - txoffset); /* Null out rest of packet. PADs will get translated or would use them */
        txoffset = FRAME_SIZE;
    }
    
    /* Check for full frame */
    if(txoffset == FRAME_SIZE)
        DR_NRF = true;  /* NRF Data Ready flag */
}

/* DMX Input Parser */
void parse_dmx(uint16_t count) {
    
}

/* RFShowControl Input Parser */
void parse_rfsc() {
    //TODO: Add check for command byte and check for protocol version?
    frame = rxbuff[RFSC_FRAME];
    DR_SERIAL = true;
}

/*************************************************/
/* Output Generators                             */
/*************************************************/
void gen_config() {
    
}

/* Generate generics RS485 stream */
void gen_rs485() {
    DR_SERIAL = false;  /* Clear our flag */
    USART_TXMODE;       /* Enable USART TX */
    xusart_send_packet(xusart_config.usart, rxbuff, frame);     /* Send rxbuff packet */
}

/* Generate Renard stream */
void gen_renard() {
    DR_SERIAL = false;  /* Clear our flag */
    USART_TXMODE;       /* Enable USART TX */

    if(frame == 0) {                                        /* Check the frame byte to reset the Renard stream if needed */
        xusart_putchar(xusart_config.usart, RENARD_SYNC);   /* Send the Renard SYNC byte */
        xusart_putchar(xusart_config.usart, RENARD_ADDR);   /* and the Command / Address byte */
    }
        
    /* Process and send channel data. Escape Renard special characters. */
    for (uint8_t i = 0; i < FRAME_SIZE; i++) {
        switch (rxbuff[i]) {
            case RENARD_PAD:
                xusart_putchar(xusart_config.usart, RENARD_ESCAPE);
                xusart_putchar(xusart_config.usart, RENARD_ESC_7D);
                break;
            case RENARD_SYNC:
                xusart_putchar(xusart_config.usart, RENARD_ESCAPE);
                xusart_putchar(xusart_config.usart, RENARD_ESC_7E);
                break;
            case RENARD_ESCAPE:
                xusart_putchar(xusart_config.usart, RENARD_ESCAPE);
                xusart_putchar(xusart_config.usart, RENARD_ESC_7F);
                break;
            default:
                xusart_putchar(xusart_config.usart, rxbuff[i]);
        }
    }
}

/* Generate DMX stream */
void gen_dmx() {
    
}

/* Generate RFShowControl packet */
void gen_rfsc() {
    DR_NRF = false;                                 /* Clear our DR flag */
    xnrf_config_tx(&xnrf_config);                   /* Configure the radio for TX mode */
    txbuff[RFSC_FRAME] = frame;                     /* Set the Frame byte */
    txbuff[RFSC_CMD] = command;                     /* Set the Command byte */
    xnrf_write_payload(&xnrf_config, txbuff, 32);   /* Load the packet */
    xnrf_pulse_tx(&xnrf_config);                    /* Send it off */
    frame++;                                        /* Increment frame counter */
    txoffset = 0;                                   /* Reset our offset tracker */
}

/*************************************************/
/* Bi-Directional Loops                          */
/*************************************************/
/* loop for bidirectional bridge - not binary safe due to flow control. implement XMODEM or something? */
//TODO: Second thoughts on this and not even started.  How will we handle flow control? XON/OFF?  Should note for terminal / config usage only.
void passthru_loop() {

}

void configuration_loop() {
    
}

void test_tx_loop() {
    uint8_t	alphatest[32] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                             0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50,
                             0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                             0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x5F};
    
    xnrf_config_tx(&xnrf_config);                           /* Configure the radio for TX mode */

    while(1) {
        xnrf_write_payload(&xnrf_config, alphatest, 32);    /* Load the packet */
        xnrf_pulse_tx(&xnrf_config);                        /* Send it off */
        
        STATUS_LED_TGL;                                     /* Toggle status LED */
        _delay_ms(1000);
    }
}

int main(void) {
    init();                     /* Initialize the board */
    sei();                      /* Enable global interrupt flag */
    USART_RXMODE;               /* USART Listen mode */
    xnrf_enable(&xnrf_config);  /* start listening on nRF */

    void (*parse_serial_func)(uint16_t) = parse_rs485;  /* Serial parser function pointer based on mode */
    void (*gen_serial_func)(void) = gen_config;         /* Serial generator function pointer based on mode */
    switch(mode)  {
        case MODE_RENARD:
            parse_serial_func = parse_renard;
            gen_serial_func = gen_renard;
            break;
        case MODE_DMX:
            parse_serial_func = parse_dmx;
            gen_serial_func = gen_dmx;
            break;
        case MODE_RS485:
            parse_serial_func = parse_rs485;
            gen_serial_func = gen_rs485;
            break;
        case MODE_CONFIG:
            parse_serial_func = parse_rs485;
            gen_serial_func = gen_config;
            break;
    }            
            
    while (1) {
        if (SERIAL_FLAG) {                                      /* Parse incoming serial data if needed - set by ISR / cleared by parser */
            SERIAL_FLAG = false;                                /* Clear our data flag */
            parse_serial_func(RingBuffer_GetCount(&rbSerial));  /* Get current count of our ring buffer */
        }                    

        if (NRF_FLAG)           /* Parse incoming NRF data if needed - set by ISR / cleared by parser */
            parse_rfsc();

        if (DR_SERIAL)          /* Send serial data if ready - set by parsers / cleared by generators */
            gen_serial_func();
        
        if (DR_NRF)             /* Send NRF data if ready - set by parsers / cleared by generator */
            gen_rfsc();
    }
}