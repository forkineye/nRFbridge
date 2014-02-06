/*
 * XNRF24L01.c
 *
 * Project: XNRF24L01
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

#include "XNRF24L01.h"
#include <avr/io.h>
#include <util/delay.h>

//TODO: Change xnrf_init so it doesn't assume a 32MHz clock, or change xspi_master_init to reference baud rates.
//TODO: Change this to xnrf_init_spi and add xnrf_init_usart??
void xnrf_init(xnrf_config_t *config) {
    // Make sure our nRF powered and stabilized, per the datasheet for power-on state transition. 
    _delay_ms(100);
    
    // Initialize SPI to 4Mhz, assume a 32Mhz clock
    config->ss_port->DIRSET = (1 << config->ss_pin);
    config->ce_port->DIRSET = (1 << config->ce_pin);
    xspi_master_init(config->spi_port, config->spi, SPI_MODE_0_gc, false, SPI_PRESCALER_DIV16_gc, true);
    //xspi_usart_master_init(&PORTC, &USARTC0, SPI_MODE_0_gc, 4000000);
    
    // configure address width
    xnrf_set_address_width(config, config->addr_width);
    
    //TODO: change this to only set default width when pipe is enabled? Does it matter?
    // configure default payload widths for all pipes
    xnrf_write_register(config, RX_PW_P0, config->payload_width);
    xnrf_write_register(config, RX_PW_P1, config->payload_width);
    xnrf_write_register(config, RX_PW_P2, config->payload_width);
    xnrf_write_register(config, RX_PW_P3, config->payload_width);
    xnrf_write_register(config, RX_PW_P4, config->payload_width);
    xnrf_write_register(config, RX_PW_P5, config->payload_width);
}

void xnrf_read_register_buffer(xnrf_config_t *config, uint8_t reg, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, (R_REGISTER | (REGISTER_MASK & reg)));
    while (len--)
        *data++ = xspi_transfer_byte(config->spi, NRF_NOP);
    xnrf_deselect(config);	
}

void xnrf_write_register_buffer(xnrf_config_t *config, uint8_t reg, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, (W_REGISTER | (REGISTER_MASK & reg)));
    while (len--)
        xspi_transfer_byte(config->spi, *data++);
    xnrf_deselect(config);	
}

void xnrf_read_payload(xnrf_config_t *config, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, R_RX_PAYLOAD);
    while (len--)
        *data++ = xspi_transfer_byte(config->spi, NRF_NOP);
    xnrf_deselect(config);	
}

void xnrf_read_payload_buffer(xnrf_config_t *config, RingBuff_t *buffer, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, R_RX_PAYLOAD);
    while (len--)
        RingBuffer_Insert(buffer, xspi_transfer_byte(config->spi, NRF_NOP));
    xnrf_deselect(config);	
}

void xnrf_write_payload(xnrf_config_t *config, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD);
    while (len--)
        xspi_transfer_byte(config->spi, *data++);
    xnrf_deselect(config);
}

void xnrf_write_payload_buffer(xnrf_config_t *config, RingBuff_t *buffer, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD);
    while (len--)
    xspi_transfer_byte(config->spi, RingBuffer_Remove(buffer));
    xnrf_deselect(config);
}

void xnrf_write_payload_noack(xnrf_config_t *config, uint8_t *data, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD_NOACK);
    while (len--)
    xspi_transfer_byte(config->spi, *data++);
    xnrf_deselect(config);
}

void xnrf_write_payload_buffer_noack(xnrf_config_t *config, RingBuff_t *buffer, uint8_t len) {
    xnrf_select(config);
    xspi_transfer_byte(config->spi, W_TX_PAYLOAD_NOACK);
    while (len--)
    xspi_transfer_byte(config->spi, RingBuffer_Remove(buffer));
    xnrf_deselect(config);
}

void xnrf_set_datarate(xnrf_config_t *config, xnrf_datarate_t rate) {
    uint8_t setup = xnrf_read_register(config, RF_SETUP);

    switch (rate) {
        case XNRF_250KBPS:
            setup |= (1 << RF_DR_LOW);
            setup &= ~(1 << RF_DR_HIGH);
            break;
        case XNRF_1MBPS:
            setup &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));
            break;
        case XNRF_2MBPS:
            setup &= ~(1 << RF_DR_LOW);
            setup |= (1 << RF_DR_HIGH);
            break;
    }
    xnrf_write_register(config, RF_SETUP, setup);	
}

void xnrf_set_address_width(xnrf_config_t *config, uint8_t width) {
    if (width == 3)
        xnrf_write_register(config, SETUP_AW, 1);
    else if (width == 4)
        xnrf_write_register(config, SETUP_AW, 2);
    else
        xnrf_write_register(config, SETUP_AW, 3);
}
